# Serial Monitor Output Explanation

This document explains every line type printed by `V2.0.ino` to Arduino Serial Monitor.

## 1) Boot and initialization events

### `BOOT_EVENT`
Example:

```text
BOOT_EVENT,version=2.0,frame_hz=100,ble_hz=50,serial_hz=20
```

Meaning:
- `version`: firmware version label.
- `frame_hz`: local frame generation rate (100 Hz).
- `ble_hz`: BLE frame notification rate (`frame_hz / BLE_FRAME_DIV`, currently 50 Hz).
- `serial_hz`: serial frame print rate (`frame_hz / SERIAL_FRAME_DIV`, currently 20 Hz).

### `IMU_EVENT`
Example:

```text
IMU_EVENT,init_ok=1
```

Meaning:
- `init_ok=1`: BNO085 initialization and report subscription succeeded.
- `init_ok=0`: BNO085 init failed; BLE may still run for debug.

## 2) BLE connection events

### `BLE_EVENT`
Examples:

```text
BLE_EVENT,connected=1
BLE_EVENT,connected=0
```

Meaning:
- `connected=1`: a phone/client connected to BLE server.
- `connected=0`: phone/client disconnected; device returns to advertising.

## 3) Calibration events

### `CALIB_EVENT`
Examples:

```text
CALIB_EVENT,source=auto,status=ok
CALIB_EVENT,source=manual,status=ok
```

Meaning:
- `source=auto`: startup auto-zero captured after the configured delay.
- `source=manual`: user pressed `c` / `C` in serial monitor to recapture zero reference.
- `status=ok`: reference quaternion was captured successfully.

## 4) Realtime telemetry frame

### `FRAME`
Example:

```text
FRAME,seq=1250,t_ms=60234,roll_deg=-1.28,pitch_deg=4.36,yaw_deg=12.51,cadence_spm=93.4,steps=218,flags=0x11,quat_acc=3
```

Field meanings:
- `seq`: incrementing frame sequence number (generated locally at 100 Hz).
- `t_ms`: local timestamp from `esp_timer_get_time()/1000`.
- `roll_deg`, `pitch_deg`, `yaw_deg`: zero-referenced relative orientation angles in degrees.
- `cadence_spm`: smoothed cadence in steps per minute (EMA filtered).
- `steps`: cumulative step counter from BNO085 step counter report.
- `flags`: compact bit mask status field.
- `quat_acc`: BNO085 rotation vector accuracy/status byte.

## 5) `flags` bit mapping

`flags` is printed in hexadecimal and uses the following bits:

- bit0 (`0x01`): `calib_ok` (startup/manual zero is valid).
- bit1 (`0x02`): `stale_q` (rotation vector data timeout).
- bit2 (`0x04`): `stale_g` (gyro data timeout).
- bit3 (`0x08`): `stale_a` (accel data timeout).
- bit4 (`0x10`): `connected` (BLE client currently connected).
- bit5 (`0x20`): `quat_acc_low` (`quat_acc < 2`).

Example: `flags=0x11` means `calib_ok=1` and `connected=1`, with no stale bits set.
