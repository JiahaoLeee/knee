#include <SPI.h>
#include <Adafruit_BNO08x.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_timer.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// SH2 report IDs (fallback for some toolchains)
#ifndef SH2_ACCELEROMETER
#define SH2_ACCELEROMETER 0x01
#endif
#ifndef SH2_GYROSCOPE_CALIBRATED
#define SH2_GYROSCOPE_CALIBRATED 0x02
#endif
#ifndef SH2_ROTATION_VECTOR
#define SH2_ROTATION_VECTOR 0x05
#endif
#ifndef SH2_STEP_COUNTER
#define SH2_STEP_COUNTER 0x11
#endif
#ifndef SH2_STEP_DETECTOR
#define SH2_STEP_DETECTOR 0x18
#endif

// SPI pins
#define SCK_PIN  6
#define MISO_PIN 5
#define MOSI_PIN 4
#define CS_PIN   7
#define INT_PIN  15
#define RST_PIN  16

// BLE UUIDs
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define STEPS_CHAR_UUID           "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define FRAME_CHAR_UUID           "c0f2a6b8-90d3-4d2b-9a8a-6a2f3c1c4b10"

// Optional: disable MTU call if your BLE library doesn't support it
#ifndef ENABLE_BLE_SET_MTU
#define ENABLE_BLE_SET_MTU 1
#endif

Adafruit_BNO08x bno(RST_PIN);
sh2_SensorValue_t v;

// -------------------- Types --------------------
struct Quat { float w, x, y, z; };

struct __attribute__((packed)) FrameV1 {
  uint32_t seq;        // frame sequence
  uint32_t t_ms;       // timestamp (ms)

  int16_t roll_cd;     // roll (centi-deg)
  int16_t pitch_cd;    // pitch (centi-deg)
  int16_t yaw_cd;      // yaw (centi-deg)

  uint16_t cadence_x10;// cadence * 10
  uint32_t steps;      // step counter

  uint16_t flags;      // status bitfield
  uint8_t  quat_acc;   // SH2 status for rotation vector
  uint8_t  reserved;
};

// flags bits
// bit0 calib_ok
// bit1 stale_q
// bit2 stale_g
// bit3 stale_a
// bit4 connected
// bit5 quat_acc_low (status < 2)

// Arduino prototype ordering workaround
static void sendStepCount(bool forceNotify);
static void sendFrameIfNeeded(const FrameV1 &f);
static void initBLE();
static bool initBNO();
static bool enableReports();
static void handleSerialCommands();
static void printFrameToSerial(const FrameV1 &f);

// -------------------- BLE state --------------------
static BLECharacteristic *pStepsChar = nullptr;
static BLECharacteristic *pFrameChar = nullptr;
static bool deviceConnected = false;

// -------------------- IMU cache --------------------
static bool q_valid=false, g_valid=false, a_valid=false;
static int64_t q_t_us=0, g_t_us=0, a_t_us=0;
static Quat q_latest{1,0,0,0};
static float gx=0, gy=0, gz=0;
static float ax=0, ay=0, az=0;
static uint8_t q_accuracy = 0;

// -------------------- Calibration --------------------
static bool calib_ok = false;
static Quat q_ref{1,0,0,0};
static int64_t calib_capture_at_us = 0;

// -------------------- Steps / cadence --------------------
static uint32_t stepCount = 0;
static uint32_t lastSentStepCount = 0;
static int64_t lastStepUs = 0;
static float cadence_spm = 0.0f;

// -------------------- Framing --------------------
static uint32_t frame_seq = 0;
static int64_t next_frame_us = 0;
static const int64_t FRAME_PERIOD_US = 10000; // 100Hz
static const uint8_t BLE_FRAME_DIV = 2;       // 2 => 50Hz notify; 1 => 100Hz
static const uint8_t SERIAL_FRAME_DIV = 5;    // 5 => 20Hz serial output
static const float CADENCE_EMA_ALPHA = 0.2f;
static const float ANGLE_EMA_ALPHA = 0.15f;

// -------------------- Smoothed outputs --------------------
static float roll_f = 0.0f;
static float pitch_f = 0.0f;
static float yaw_f = 0.0f;
static bool angle_filter_initialized = false;

// -------------------- Quaternion utilities --------------------
static inline Quat quatConj(const Quat &q) { return {q.w, -q.x, -q.y, -q.z}; }

static inline Quat quatMul(const Quat &a, const Quat &b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static inline void quatNormalize(Quat &q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n > 1e-6f) { q.w/=n; q.x/=n; q.y/=n; q.z/=n; }
}

static inline void quatToEulerDeg(const Quat &q, float &roll, float &pitch, float &yaw) {
  float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
  float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
  roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q.w*q.y - q.z*q.x);
  if (fabsf(sinp) >= 1.0f) pitch = copysignf((float)M_PI/2.0f, sinp);
  else pitch = asinf(sinp);

  float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
  yaw = atan2f(siny_cosp, cosy_cosp);

  const float rad2deg = 57.2957795f;
  roll *= rad2deg; pitch *= rad2deg; yaw *= rad2deg;
}

static inline int16_t clampCentiDeg(float deg) {
  float cd = deg * 100.0f;
  if (cd > 32767.0f) cd = 32767.0f;
  if (cd < -32768.0f) cd = -32768.0f;
  return (int16_t)lrintf(cd);
}

// -------------------- BLE helpers --------------------
static void sendStepCount(bool forceNotify) {
  if (!pStepsChar) return;
  String data = String(stepCount);
  pStepsChar->setValue(data.c_str());
  if (deviceConnected && (forceNotify || stepCount != lastSentStepCount)) {
    pStepsChar->notify();
    lastSentStepCount = stepCount;
  }
}

static void sendFrameIfNeeded(const FrameV1 &f) {
  if (!pFrameChar || !deviceConnected) return;
  pFrameChar->setValue((uint8_t*)&f, sizeof(FrameV1));
  pFrameChar->notify();
}

// -------------------- BLE server callbacks --------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE_EVENT,connected=1");
    sendStepCount(true);
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE_EVENT,connected=0");
    BLEDevice::startAdvertising();
  }
};

// -------------------- BLE init --------------------
static void initBLE() {
  BLEDevice::init("ESP32_BNO085_1IMU");

#if ENABLE_BLE_SET_MTU
  // Larger MTU helps carry the 24-byte frame reliably.
  BLEDevice::setMTU(185);
#endif

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Debug-friendly steps (ASCII)
  pStepsChar = pService->createCharacteristic(
    STEPS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pStepsChar->addDescriptor(new BLE2902());

  // Binary telemetry frame
  pFrameChar = pService->createCharacteristic(
    FRAME_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pFrameChar->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);

  // Prefer shorter connection intervals on some phones.
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();
}

// -------------------- BNO reports --------------------
static bool enableReports() {
  // 100Hz motion reports
  if (!bno.enableReport(SH2_ROTATION_VECTOR,      10000)) return false;
  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) return false;
  if (!bno.enableReport(SH2_ACCELEROMETER,        10000)) return false;

  // Step reports (lower rate is enough)
  if (!bno.enableReport(SH2_STEP_COUNTER,  20000)) return false;
  if (!bno.enableReport(SH2_STEP_DETECTOR, 20000)) return false;

  return true;
}

// -------------------- BNO init --------------------
static bool initBNO() {
  // Keep CS high before SPI init.
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Use pull-up if your board doesn't provide external pull-up.
  pinMode(INT_PIN, INPUT_PULLUP);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  delay(50);

  if (!bno.begin_SPI(CS_PIN, INT_PIN, &SPI)) return false;
  if (!enableReports()) return false;

  return true;
}

// -------------------- Serial commands --------------------
static void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'c' || c == 'C') {
      // Manual zero capture
      if (q_valid) {
        q_ref = q_latest;
        quatNormalize(q_ref);
        calib_ok = true;
        angle_filter_initialized = false;
        Serial.println("CALIB_EVENT,source=manual,status=ok");
      }
    }
  }
}

static void printFrameToSerial(const FrameV1 &f) {
  Serial.print("FRAME");
  Serial.print(",seq="); Serial.print(f.seq);
  Serial.print(",t_ms="); Serial.print(f.t_ms);
  Serial.print(",roll_deg="); Serial.print(f.roll_cd / 100.0f, 2);
  Serial.print(",pitch_deg="); Serial.print(f.pitch_cd / 100.0f, 2);
  Serial.print(",yaw_deg="); Serial.print(f.yaw_cd / 100.0f, 2);
  Serial.print(",cadence_spm="); Serial.print(f.cadence_x10 / 10.0f, 1);
  Serial.print(",steps="); Serial.print(f.steps);
  Serial.print(",flags=0x"); Serial.print(f.flags, HEX);
  Serial.print(",quat_acc="); Serial.println(f.quat_acc);
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  bool ok = initBNO();
  initBLE();

  Serial.print("IMU_EVENT,init_ok=");
  Serial.println(ok ? 1 : 0);

  // Initialize state even if IMU init fails (BLE still works for debugging).
  stepCount = 0;
  sendStepCount(true);

  int64_t now = esp_timer_get_time();
  next_frame_us = now + FRAME_PERIOD_US;
  calib_capture_at_us = now + 2000000; // auto zero after 2s

  Serial.println("BOOT_EVENT,version=2.0,frame_hz=100,ble_hz=50,serial_hz=20");
}

void loop() {
  handleSerialCommands();

  // Re-enable reports if the sensor reset.
  if (bno.wasReset()) {
    enableReports();
  }

  // Read only a limited number of events per loop to avoid blocking.
  for (int i = 0; i < 16; i++) {
    if (!bno.getSensorEvent(&v)) break;

    int64_t t = esp_timer_get_time();

    if (v.sensorId == SH2_ROTATION_VECTOR) {
      q_latest = { v.un.rotationVector.real,
                   v.un.rotationVector.i,
                   v.un.rotationVector.j,
                   v.un.rotationVector.k };
      quatNormalize(q_latest);
      q_accuracy = v.status;
      q_valid = true;
      q_t_us = t;

    } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      gx = v.un.gyroscope.x;
      gy = v.un.gyroscope.y;
      gz = v.un.gyroscope.z;
      g_valid = true;
      g_t_us = t;

    } else if (v.sensorId == SH2_ACCELEROMETER) {
      ax = v.un.accelerometer.x;
      ay = v.un.accelerometer.y;
      az = v.un.accelerometer.z;
      a_valid = true;
      a_t_us = t;

    } else if (v.sensorId == SH2_STEP_COUNTER) {
      stepCount = v.un.stepCounter.steps;
      sendStepCount(false);

    } else if (v.sensorId == SH2_STEP_DETECTOR) {
      // Cadence from step-to-step interval
      if (lastStepUs != 0) {
        int64_t dt = t - lastStepUs;
        if (dt > 250000 && dt < 2000000) {
          float cadence_new = 60.0f * 1000000.0f / (float)dt;
          cadence_spm = cadence_spm * (1.0f - CADENCE_EMA_ALPHA) + cadence_new * CADENCE_EMA_ALPHA;
        }
      }
      lastStepUs = t;
    }
  }

  int64_t now = esp_timer_get_time();

  // One-shot auto zero
  if (!calib_ok && q_valid && now >= calib_capture_at_us) {
    q_ref = q_latest;
    quatNormalize(q_ref);
    calib_ok = true;
    angle_filter_initialized = false;
    Serial.println("CALIB_EVENT,source=auto,status=ok");
  }

  // Fixed-rate framing at 100Hz
  if (now >= next_frame_us) {
    next_frame_us += FRAME_PERIOD_US;
    frame_seq++;

    bool stale_q = (!q_valid) || (now - q_t_us > 2*FRAME_PERIOD_US);
    bool stale_g = (!g_valid) || (now - g_t_us > 2*FRAME_PERIOD_US);
    bool stale_a = (!a_valid) || (now - a_t_us > 2*FRAME_PERIOD_US);

    float roll=0, pitch=0, yaw=0;
    if (q_valid) {
      Quat q_rel = q_latest;
      if (calib_ok) {
        Quat q_inv = quatConj(q_ref);
        q_rel = quatMul(q_inv, q_latest);
        quatNormalize(q_rel);
      }
      quatToEulerDeg(q_rel, roll, pitch, yaw);

      if (!angle_filter_initialized) {
        roll_f = roll;
        pitch_f = pitch;
        yaw_f = yaw;
        angle_filter_initialized = true;
      } else {
        roll_f = roll_f * (1.0f - ANGLE_EMA_ALPHA) + roll * ANGLE_EMA_ALPHA;
        pitch_f = pitch_f * (1.0f - ANGLE_EMA_ALPHA) + pitch * ANGLE_EMA_ALPHA;
        yaw_f = yaw_f * (1.0f - ANGLE_EMA_ALPHA) + yaw * ANGLE_EMA_ALPHA;
      }

      roll = roll_f;
      pitch = pitch_f;
      yaw = yaw_f;
    }

    FrameV1 f{};
    f.seq = frame_seq;
    f.t_ms = (uint32_t)(now / 1000);

    f.roll_cd  = clampCentiDeg(roll);
    f.pitch_cd = clampCentiDeg(pitch);
    f.yaw_cd   = clampCentiDeg(yaw);

    f.cadence_x10 = (uint16_t)(
      cadence_spm < 0 ? 0 :
      (cadence_spm > 6553.5f ? 65535 : (uint16_t)lrintf(cadence_spm * 10.0f))
    );
    f.steps = stepCount;

    uint16_t flags = 0;
    if (calib_ok)        flags |= (1u<<0);
    if (stale_q)         flags |= (1u<<1);
    if (stale_g)         flags |= (1u<<2);
    if (stale_a)         flags |= (1u<<3);
    if (deviceConnected) flags |= (1u<<4);
    if (q_accuracy < 2)  flags |= (1u<<5);
    f.flags = flags;

    f.quat_acc = q_accuracy;

    // Send at a lower BLE rate for stability if desired.
    if (deviceConnected && (frame_seq % BLE_FRAME_DIV == 0)) {
      sendFrameIfNeeded(f);
    }

    if (frame_seq % SERIAL_FRAME_DIV == 0) {
      printFrameToSerial(f);
    }
  }

  delay(0);
}
