# Templates/Coordinator/ -- 上层应用协调器范例 (Phase 2)

本目录是给"上层应用开发者"的 API 使用范例骨架, **不参与 lib 编译**。
lib 编译时不会 `#include` 本目录任何文件, 复制使用即可。

## 与 lib 的关系

- lib 是单轴跟踪, 永远只持有一个流式 setpoint。lib 自己不知道"另一根轴"。
- 多轴同步 (XY 走斜线、关节协同、贴片机左右双电机) 由 lib 之上的 Coordinator 层完成。
- Coordinator 通过两种形态接 lib:
  - **单 MCU 多实例** (MAX_INSTANCES>1, 如多个风扇一台板): 同一 ISR tick 调多次 `MotorAPI::writeSetpoint`/`appendSetpointSeg`
  - **多 MCU 分布式** (你的高级应用规划): 每轴一块独立 MCU, 各自跑一个 lib 实例, 主机经通信总线 (CAN/EtherCAT) 给每个从 MCU 下发预加载段, 再广播一次 SYNC 信号触发同步回放

## 调用序 (分布式)

```
1. 主机 (G-code 解释器 / 关节 PVT 源 / MPC) 拟定本拍各轴 setpoint
2. 对每个 LocalAxis (本 MCU 直接驱动的电机):
       MotorAPI::appendSetpointSeg(&sp, 1)
3. 对每个 RemoteAxis (远端从 MCU):
       CommBus::sendSegment(remote_id, &sp, 1)
4. 等所有从 MCU 应答就绪: CommBus::pollSyncAck()
5. 广播 SYNC: CommBus::signalSync()
   + 本地 MotorAPI::triggerSegPlayback(IMMEDIATE)
6. 每块从 MCU 收到 SYNC 后, lib tick() 自动每拍消费 FIFO 第一拍
```

## 范例文件

- `SyncedAxisGroup_Template.h` -- 主机端协调器抽象基类 (AxisHandle / CommBus / scheduleSegment)
- `SyncedAxisGroup_Template.cpp` -- tick 派发骨架 (Phase 3 lib 接通后直接取掉 TODO)
- `DualAxisSinDemo.h` -- X 走 sin、Y 走 cos 的圆轨迹 demo, 验证 XY 走斜线/圆

## 与 lib API 的对应

| Coordinator 调用 | lib 单 MCU | lib 多 MCU 主机端 | lib 多 MCU 从端 |
|---|---|---|---|
| `MotorAPI::writeSetpoint(sp)` | 立即生效 | 每拍喂数据到主机上的本地 lib | 经 sync 后写入从 MCU 的 lib |
| `MotorAPI::appendSetpointSeg(&sp, n)` | 写入本地 FIFO | 写入主机本地 lib FIFO + 同时 sendSegment 到远端 | 接收解析后写入从 MCU 本地 FIFO |
| `MotorAPI::triggerSegPlayback(IMMEDIATE)` | 单 MCU 立即开始 tick-by-tick 消费 | 主机本地立即触发 || 不适用 |
| `MotorAPI::triggerSegPlayback(AWAIT_SYNC)` | 不必用, 只多 MCU 才有意义 | 标记主机本地 lib 等 sync 触发 || 由从 MCU 收到 SYNC 信号调用 |
| (远端) 接 SYNC 后调本地 `triggerSegPlayback(IMMEDIATE)` 启播 | — | 主机端用 CommBus 触发广播 || 每从 MCU 各自执行 |

## Phase 3.0 状态

- `writeSetpoint` 已可用, 适合单 MCU 单轴或单 MCU 多实例即时写入。
- `MOTOR_BUILD_ENABLE_SETPOINT_FIFO_PLAYBACK` 关闭时, `appendSetpointSeg` / `triggerSegPlayback` / `abortSegPlayback` 返回 `NotSupported`。
- `MOTOR_BUILD_ENABLE_SETPOINT_FIFO_PLAYBACK` 开启后, lib 内使用固定容量本地 FIFO + trigger latch 状态机, 每个控制 tick 最多消费一拍。
- 通信层、协议帧、ACK、水位、SYNC 广播和 G-code/轨迹规划仍保持在 lib 外。

## 改造指南

把本目录拷到你的 `UserAPP/Coordinator/`, 改:
1. `CommBus` 子类实现你的真实通信底层 (CAN / EtherCAT / SPI / 自定义 RS485)
2. 继承 `SyncedAxisGroupBase`, 在 `scheduleSegment` 把你的产品轨迹解析输出为各轴 setpoint
3. `DualAxisSinDemo` 仅作为圆轨迹自检 demo, 真产品可删除

## 接电机验证的 checklist

- [ ] BuildCfg 开启 `MOTOR_BUILD_ENABLE_POSITION_CONTROL` 且 `MOTOR_BUILD_ENABLE_VELOCITY_FEEDFORWARD`
- [ ] MotorConfig.control.feedforward.enable_vel_ff = true, vel_ff_gain = 1.0
- [ ] 单轴 `motor.selectMode(POSITION_CONTROL); motor.start();`
- [ ] 单轴单拍 `writeSetpoint({mode=POSITION_CONTROL, pos_ref=0, vel_ff=0})`, 电机应回到 0 位
- [ ] 单轴外加正弦轨迹, 检查跟随误差
- [ ] 双轴: 接两个 MotorAPI 实例 (单 MCU 扩 MAX_INSTANCES=2 或双 MCU), 跑 DualAxisSinDemo
- [ ] 在示波器 (VOFA 等) 上观 X-Y 相平面应画一个圆, 表示同步没失锁
