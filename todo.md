做一条“IMU→ESP32 本地计算→BLE→手机展示/记录”的链路，在不上传云的前提下，实时输出康复相关指标。由于时间紧，阶段 1 只做能跑通、可展示、可复用到双 IMU 的最小实现：固定频率组帧 + 启动零位校准 + 最基础平滑 + BLE 结构化输出。

项目拆分（两阶段都沿用同一拆法）
1）传感器与总线：BNO085 初始化、SPI 读事件、必要时重订阅。
2）采样与组帧：100Hz 读入，10ms 生成一帧（seq、时间戳、数据、基本标志）。
3）校准：只做“启动时零位”，不做复杂动作流程、不做融合可靠性判断。
4）计算：从单/双 IMU 得到目标指标；阶段 1 只算单 IMU 可算的。
5）BLE：把“结果帧”发给手机（手机只负责显示/记录/趋势）。
6）验证：长时间运行不卡死、帧率稳定、手机端 seq 基本连续。

两阶段定义（你要的）
阶段 1：单 IMU 所有数据获得（包括计算）
输入数据（来自一颗 BNO085）：

Rotation Vector（四元数，100Hz）

Gyro（100Hz）

Accel（100Hz）

Step Counter/Step Detector（低频即可）

阶段 1 输出（本地算完通过 BLE 发手机）：

段姿态变化（相对启动零位）：roll/pitch/yaw（或你指定的三个角）

步数 step_count（用 BNO step counter）

步频 cadence（用 step detector 的步间隔）

flags（只保留最必要的：calib_ok、stale 等）

阶段 2：双 IMU 所有数据获得（包括对照/相对计算）
输入数据（两颗 BNO085 对称订阅）：

两边都订阅 quat/gyro/accel（100Hz），step 可只开小腿那颗

阶段 2 关键计算：

膝关节相对姿态：q_rel = conj(q_thigh) ⊗ q_shank

膝屈伸角/ROM：先用欧拉近似从 q_rel 取某一轴角（最省时间的版本），后续如有时间再升级到 hinge-axis/swing-twist

其余（步频/脚滚转趋势）继续以小腿为主，必要时做两侧一致性对照（可后置）

阶段 1 需要做什么（按“最小可交付”列清楚）
1）保持你现在的“启动校准”
做法就是你代码现在这样：开机后延时（例如 2s）抓一帧四元数作为 q_ref，设 calib_ok=true。保留串口按键/命令重置零位，方便调试即可。不做站直/90°等流程。

2）固定 100Hz 组帧（你已经做对了）
每 10ms 生成一帧：frame_seq + t_ms（或 t_us）+ 计算结果 + flags。BLE 为了稳定可以先发 50Hz（你现在 BLE_FRAME_DIV=2），手机端展示不会受影响。

3）阶段 1 的“必须计算项”和“怎么计算”

相对姿态角：q_rel = conj(q_ref) ⊗ q_latest；再 quat→Euler（roll/pitch/yaw）

步数：stepCount = BNO stepCounter

步频：cadence_spm = 60e6 / (t_step_now - t_step_prev)

4）只做最初级的平滑（不做异常剔除/复杂质量）
建议做两处最简单的平滑就够用：

cadence：EMA 平滑，例如 cadence = 0.8cadence + 0.2cadence_new

角度：对 roll/pitch/yaw 做轻度 EMA，或者只对你关心的那个角做 EMA（避免曲线抖动）

5）保留最小 flags（别扩展到融合可靠性）
你可以只留：

calib_ok（是否完成启动零位）

stale_q / stale_g / stale_a（超时没更新就置位）

connected（是否已连接）
其余比如磁干扰、融合精度，不做。

阶段 1 目前“缺失的算法”在你的约束下，实际上只有这些
1）最初级平滑：cadence（强烈建议）和姿态角（可选）。
2）可选的“角度连续性处理”：欧拉角跨 ±180° 时会跳，时间紧可以先不管；如果你发现跳变影响展示，再加一个最简单的 unwrap（只对 yaw/roll 中会跳的那一路做）。
