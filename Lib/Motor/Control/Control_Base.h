/*
 * Control_Base.h — 控制策略抽象基类 (vtable)
 *
 * 预留扩展: 未来可将 FOC/VF/IF 封装为不同控制策略
 *   通过 vtable → process() 统一调用
 *   替代当前的 switch(mode) 分支
 */

#pragma once

namespace Lib_Motor
{

struct ControlVTable
{
    void (*init)(void* self, const class MotorConfig* cfg);
    void (*process)(void* self, float id_ref, float iq_ref,
                    float theta_e, float vbus, const float i_abc[3],
                    float* vd_out, float* vq_out);
    void (*reset)(void* self);
};

} // namespace Lib_Motor
