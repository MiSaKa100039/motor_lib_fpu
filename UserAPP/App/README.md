# UserAPP/App 应用协调层骨架

本目录放在 `UserAPP` 下, 介于通信层和 `MotorAPI` 之间。

## 当前边界

- `BSP/Communication`: 只负责 UART/CAN/SPI/DMA 等硬件收发。
- `Platform/Communication`: 只负责本板通信端口初始化和协议实例绑定。
- `UserAPP/App`: 负责把业务消息转换为 `MotorAPI` 调用。
- `Lib/Motor`: 只执行单轴控制、状态机、安全保护和能量限制。

通信层未来收到一帧业务消息后, 应解析成 `AppCommand`, 再调用
`App_PostCommand(cmd)`。不要在通信中断或 DMA 回调里直接调用 `MotorAPI`。

## 目录说明

- `Core/`: 应用层通用入口、命令结构、命令 FIFO 和裁切开关。
- `Axis/`: 单轴服务, 包装一个 `MotorAPI` 实例。
- `Motion/`: 轨迹流、多轴运动分发入口, 由 `USERAPP_ENABLE_MOTION_COORDINATOR` 控制。
- `Energy/`: BMS、制动电阻、回收比例到能量预算的入口, 由 `USERAPP_ENABLE_ENERGY_COORDINATOR` 控制。
- `Sync/`: 多轴同步预加载和触发入口, 由 `USERAPP_ENABLE_SYNC_COORDINATOR` 控制。

## 裁切开关

默认配置在 `Core/App_BuildConfig.h`:

- `USERAPP_ENABLE_SETPOINT_STREAM = 1`: 单轴 setpoint 流入口。
- `USERAPP_ENABLE_MOTION_COORDINATOR = 0`: 多轴/轨迹协调器默认关闭。
- `USERAPP_ENABLE_ENERGY_COORDINATOR = 0`: 能量协调器默认关闭。
- `USERAPP_ENABLE_SYNC_COORDINATOR = 0`: 分布式同步协调器默认关闭。

普通应用可以只保留 `Core + Axis`。车辆回收、贴片机、机器狗等高级应用再按需打开
`Energy/Motion/Sync`。这些开关只裁切 `UserAPP` 上层应用代码, 不改变 `Lib/Motor` 的
公共 API 和内部控制环。

对应到底层 `Platform/Motor/BuildCfg` 时, 不按产品名裁切, 而按执行原语裁切:

- 实时反应式轨迹: `MOTOR_BUILD_ENABLE_STREAM_HOLD_WHEN_IDLE`, 用于实时 setpoint 流断流后的安全持仓。
- 可预计轨迹: `MOTOR_BUILD_ENABLE_SETPOINT_FIFO_PLAYBACK`, 用于预加载 FIFO 和同步触发回放。
- 位置/阻抗/前馈等控制能力仍由 `MOTOR_BUILD_ENABLE_POSITION_CONTROL`、
  `MOTOR_BUILD_ENABLE_IMPEDANCE_CONTROL`、`MOTOR_BUILD_ENABLE_VELOCITY_FEEDFORWARD`
  等控制。

这些宏也接入了 CMake cache option, 可在配置时覆盖:

```bash
cmake -S . -B build/app-sync -G Ninja ^
  -DCMAKE_BUILD_TYPE=RelWithDebInfo ^
  -DUSERAPP_ENABLE_MOTION_COORDINATOR=ON ^
  -DUSERAPP_ENABLE_SYNC_COORDINATOR=ON
```

## 多轴同步思路

同一 MCU 多轴:

1. 一个固定周期调度器产生所有轴本拍 `MotionSetpoint`。
2. 在同一个调度 tick 内依次调用每个轴的 `writeSetpoint()`。
3. 各轴底层 ISR 使用同一时基, 同步误差主要来自函数调用顺序, 通常是微秒级。

多 MCU 分布式多轴:

1. 主控制器提前计算未来一段路径。
2. 主控制器把每个执行机构未来 N 拍 setpoint 预发送到本地 FIFO。
3. 每个执行机构校验序号和缓存水位后回复 ready。
4. 主控制器广播 SYNC 或带时间戳的触发命令。
5. 所有执行机构从同一个触发 tick 开始, 每个控制周期从 FIFO 取一拍执行。

FIFO 是 First-In First-Out, 即先进先出队列。它常用环形缓冲实现, 不需要动态分配。
在运动同步里, FIFO 的作用是抵抗通信抖动: 路径提前到达执行机构, 真正运行时只依赖本地时基和同步触发。
