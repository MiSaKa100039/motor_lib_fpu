/*
 * SyncedAxisGroup_Template.cpp -- 分布式多轴协调器主机端抽象基类实现 (骨架)
 *
 * 注: 本文件不参与 lib 编译, 仅作为上层应用开发范本。
 *     真接入时把这文件拷到 UserAPP/Coordinator/ 下改造后参与应用编译即可。
 */

#include "SyncedAxisGroup_Template.h"

namespace Coordinator_Template
{

/*
 * tick -- 每 ISR 或上层周期调用一次
 *
 * 协调器内部 t_sec 推进, 调子类 scheduleSegment 拿到本拍 setpoint,
 * 分发到本地 lib API 或远端 MCU。
 *
 * 注: 本骨架里 append/trigger 等 API 在 Phase 2 lib 中还没接通实现,
 *     因此这里把"接口调用"用注释表示; Phase 3 lib 接通后直接对应取掉 TODO
 *     即可。t_sec 增量 dt_sec 来自调用者提供, 跨越任何具体 ISR 频率。
 */
void SyncedAxisGroupBase::tick(double dt_sec)
{
    AxisSetpointSeg segments[8];   /* 假设最多 8 轴, 真用例扩容 */
    uint16_t         actual_count  = 0;

    if (!scheduleSegment(t_sec_, segments, 8, actual_count))
    {
        return;
    }

    /* 1. 分发本拍 setpoint 到每个轴 */
    for (uint16_t i = 0; i < actual_count; ++i)
    {
        /* 在 axes_ 中按 axis_id 找对应 AxisHandle */
        const AxisHandle* found = nullptr;
        for (const auto& ax : axes_)
        {
            if (ax.axis_id == segments[i].axis_id)
            {
                found = &ax;
                break;
            }
        }
        if (!found) continue;

        if (found->local_api != nullptr)
        {
            /* LocalAxis: 直接调本 MCU lib 的 MotorAPI */
            /* TODO Phase 3: Result r = found->local_api->appendSetpointSeg(&segments[i].sp, 1); */
            (void)segments[i].sp;
        }
        else if (comm_ != nullptr)
        {
            /* RemoteAxis: 经通信总线发到远端 MCU 驱动器 */
            /* Phase 3: comm_->sendSegment(found->remote_id, &segments[i], 1); */
        }
    }

    /* 2. 到达同步触发边界 (按上层调度策略)
     *    本骨架仅给示意: 若 armed_=false 表示"首段下完进入 armed",
     *    然后等所有从 MCU 接收 ack 后再 broadcast SYNC。 */
    if (!armed_)
    {
        bool all_ready = (comm_ != nullptr) ? comm_->pollSyncAck() : true;
        if (all_ready)
        {
            if (comm_ != nullptr) comm_->signalSync();
            for (const auto& ax : axes_)
            {
                if (ax.local_api != nullptr)
                {
                    /* TODO Phase 3: ax.local_api->triggerSegPlayback(PlaybackTrigger::IMMEDIATE); */
                }
            }
            armed_ = true;
        }
    }

    t_sec_ += dt_sec;
}

} // namespace Coordinator_Template