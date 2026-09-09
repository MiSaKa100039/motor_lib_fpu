#pragma once

/*
 * SyncedAxisGroup_Template.h -- 分布式多轴协调器主机端抽象基类 (骨架)
 *
 * 用途: 这是给"上层应用开发者"的 Coordinator 范例代码骨架, 不进 lib 编译树。
 *       本文件示范真正实用的多轴同步在分布式多 MCU 架构下的标准调用序。
 *
 * 适用架构 (与你的产品规划对齐):
 *
 *   [主机 / 上位机]
 *        |
 *        | G-code / 关节 PVT 流 / 机器狗 MPC 输出
 *        v
 *   [本 MCU (主机端)] -- Coordinator: 拆解 PVT 到各轴的 setpoint 段
 *        |           |
 *        |           +-- 本 MCU 直接驱动的电机: MotorAPI::appendSetpointSeg + triggerSegPlayback
 *        |
 *        +-- 远端从 MCU (X 轴 / Y 轴 / 各关节驱动器) 经 CAN / EtherCAT
 *                每块从 MCU 各自接:
 *                  - 收到 setpoint 段 -> 本地 lib appendSetpointSeg
 *                  - 收到 SYNC trigger -> 本地 lib triggerSegPlayback(AWAIT_SYNC)
 *                  - lib tick() -> 每 tick 自动消费 FIFO 第一拍 -> 控制环跟踪
 *
 * 同步语义:
 *   1. 主机分发 setpoint 段时所有从 MCU 都进入 AWAIT_SYNC 状态 (lib 已预加载 FIFO)
 *   2. 主机广播一次 SYNC 信号 -> 所有从 MCU 同步 latch 第一拍 -> 周期同步开始
 *   3. 每拍各从 MCU 的 lib.tick() 自动消费 FIFO 下一拍
 *   4. 后续: 主机继续 append 下一段, 从 MCU 同时消费当前段 (典型 Slide-Ring)
 *
 * 与单 MCU 多轴的关系:
 *   单 MCU + MAX_INSTANCES > 1 (如风扇类) 走 Coordinator 的 "LocalAxis" 路径即可,
 *   不需要 RemoteAxis 与通信层, 但 SYNC 信号仍可在本 MCU 内部用同一个 ISR tick 充当。
 *
 * Phase 2 状态:
 *   lib 的 appendSetpointSeg / triggerSegPlayback API 当前为占位, 直接 return NotSupported。
 *   Phase 3 接通通信层与 lib 内部 SetpointFifo 后, 本模板即立即可用。
 *
 * 使用方法 (拿真硬件时):
 *   1. 复制本目录到你的 UserAPP/Coordinator/
 *   2. 实现 CommBus_*: 替换 CommBus_Dummy 命名为你的 BSP 实际函数 (CAN 发送/EtherCAT CiA402 等)
 *   3. 继承 SyncedAxisGroupBase, 在 scheduleSegment() 中按你的产品语义轨迹拆解为 N 个 setpoint
 *   4. 真双轴应用可直接用 DualAxisSinDemo (验证 XY 走斜线/圆)
 */

#include <stdint.h>
#include <vector>
#include "Lib_Motor/Public/Motor_Definitions.h"
#include "Lib_Motor/Public/Motor_API.h"

namespace Coordinator_Template
{

using Lib_Motor::MotionSetpoint;
using Lib_Motor::MotorAPI;
using Lib_Motor::Result;

/*
 * PlaybackTrigger -- 触发回放时机的本地副本
 *
 * 注: lib 的 MotorAPI::triggerSegPlayback 也会用同名 enum;
 *     在 Coordinator 名称空间内重复声明仅为模板自包含, 避免不直接 include lib 而出现错指。
 */
enum class PlaybackTrigger : uint8_t
{
    IMMEDIATE  = 0,   /* 立即开始 tick-by-tick 消费 */
    AWAIT_SYNC = 1,   /* 预加载后等通信层 SYNC 信号触发 */
};

/*
 * AxisHandle -- 抽象一个被协调的"轴位置"
 *
 * 两种实例化形态:
 *   - LocalAxis: 本 MCU 上的 MotorAPI 实例
 *   - RemoteAxis: 通过 CommBus_* 控制的远端 MCU 驱动器
 *
 * 设计意图: 主机端 Coordinator 用同一调用序同时管两种;
 *           scheduleSegment 输出 AxisSetpoint 列表后, dispatch 本身不区分 Local/Remote。
 */
struct AxisHandle
{
    uint8_t    axis_id;     /* 主机层定义的逻辑轴号 (X=0, Y=1, ...) */
    MotorAPI*  local_api;   /* 非空时为 LocalAxis; 否则为 RemoteAxis */
    uint32_t   remote_id;   /* RemoteAxis 时使用的通信地址 (CAN ID / EtherCAT slave 等) */
};

/*
 * AxisSetpointSeg -- 单轴一段 setpoint (写入 MotorAPI::appendSetpointSeg 的一个元素)
 */
struct AxisSetpointSeg
{
    uint8_t         axis_id;
    MotionSetpoint  sp;
};

/*
 * CommBus -- 通信底层抽象 (BSP 替换点)
 *
 * 主机端把"对远端从 MCU 的操作"全归并到这三个函数:
 *   sendSegment(remote_id, seg*): 把 setpoint 段打过去
 *   signalSync():                 广播 SYNC, 触发所有从 MCU 同时 latch 开始播放
 *   pollSyncAck():                Phase 3 占位, 等所有从 MCU 应答就绪, 返回 true 后再 signalSync
 */
class CommBus
{
public:
    virtual ~CommBus() = default;
    virtual bool sendSegment(uint32_t remote_id, const AxisSetpointSeg* seg, uint16_t count) = 0;
    virtual bool signalSync() = 0;
    virtual bool pollSyncAck() = 0;
};

/*
 * SyncedAxisGroupBase -- 主机端协调器抽象基类
 *
 * 子类只需实现 scheduleSegment(t_sec) -> std::vector<AxisSetpointSeg>:
 *   该函数把"主机 G-code 等轨迹源"在指定时刻对应的"本拍各轴 setpoint" 全部生成出来。
 *
 * 调用序:
 *   attachAxis(X)  / attachAxis(Y) / ...
 *   setCommBus(can_driver)
 *   tick()  -- 每 ISR 或上层控制周期调一次
 *
 * tick 内部:
 *   1. t_sec 推进
 *   2. 调 scheduleSegment 取下一段
 *   3. 对每个 AxisSetpointSeg:
 *        - LocalAxis:  MotorAPI::appendSetpointSeg(&sp, 1)
 *        - RemoteAxis: CommBus::sendSegment(...)
 *   4. 若达到 sync 触发点: CommBus::pollSyncAck() -> CommBus::signalSync()
 *                       + 本地 MotorAPI::triggerSegPlayback(IMMEDIATE)
 */
class SyncedAxisGroupBase
{
public:
    virtual ~SyncedAxisGroupBase() = default;

    void attachAxis(const AxisHandle& ax)
    {
        axes_.push_back(ax);
    }

    void setCommBus(CommBus* bus) { comm_ = bus; }

    /* 子类实现: 给时刻 t, 生成本拍各轴 setpoint 段 */
    virtual bool scheduleSegment(double t_sec,
                                 AxisSetpointSeg* out_segments,
                                 uint16_t max_count,
                                 uint16_t& out_count) = 0;

    /* 主循环入口, 由上层协调 ISR 周期调用 */
    void tick(double dt_sec);

protected:
    std::vector<AxisHandle> axes_;
    CommBus*                comm_     = nullptr;
    double                  t_sec_    = 0.0;
    bool                    armed_    = false;   /* 是否已预加载完毕, 等信号触发 */
};

} // namespace Coordinator_Template