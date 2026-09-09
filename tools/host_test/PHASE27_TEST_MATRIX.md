# Phase 2.7 / Phase 3.0 测试矩阵

## 构建矩阵

使用 `tools/build_motor_matrix.ps1` 覆盖以下组合：

- `default`: 当前默认能力集。
- `app-all`: 打开 `USERAPP_ENABLE_MOTION_COORDINATOR`、`USERAPP_ENABLE_ENERGY_COORDINATOR`、`USERAPP_ENABLE_SYNC_COORDINATOR`。
- `position-ff-held`: 打开位置环、速度前馈和断流保持。
- `fifo-on`: 打开 `MOTOR_BUILD_ENABLE_SETPOINT_FIFO_PLAYBACK`。
- `impedance-torqueff`: 只做阻抗/力矩前馈编译检查，不进入硬件调参。

每个组合都跑 `FULL` 和 `CORE` source set，并记录 RAM/FLASH 增量。

## HostTest 优先用例

- `writeSetpoint`: TORQUE/VELOCITY/POSITION/IMPEDANCE 字段落地，`Mode::NONE` 不激活 stream。
- `StreamFSM`: ACTIVE 到 HELD、fault suspend、user stop suspend、恢复写入。
- `Position runtime`: `pos_ref > angle` 时 `iq_ref_command` 极性正确，`vel_ff` 叠加方向正确。
- `Setpoint FIFO`: append 顺序、overflow、armed、immediate trigger、abort、播完、非 RUN 不消费、单 tick 单拍。
- `BuildCfg`: 关闭能力时返回 `NotSupported`，开启后相关分支可编译。

## On-target 验证

- 低压/低流/空载验证 start/stop/emergencyStop/clearFault。
- 用 VOFA 或串口观察 `MotorDebugData.target_pos_rad`、`vel_ff_rad_s`、`torque_ff_a`、`stream_state`、`playback_state`、`playback_count`。
- FIFO 固定段测试：未 trigger 不动，trigger 后逐拍消费，abort 后停止消费。
- 单 MCU 多轴先跑 XY 圆轨迹；多 MCU 后续再接 CAN/EtherCAT 的 ACK、水位和 SYNC 广播。
