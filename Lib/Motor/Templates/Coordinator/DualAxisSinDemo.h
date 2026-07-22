#pragma once

/*
 * DualAxisSinDemo.h -- 双轴 sin/cos 圆轨迹演示协调器 (XY 走斜线/圆)
 *
 * 用途: 验证 SyncedAxisGroupBase 抽象的最小可执行示例, 给开发者示范
 *       "主机协调器如何把一段笛卡尔轨迹拆解成各轴 setpoint 段"。
 *
 * 演示轨迹:
 *   X 轴 pos_ref = A * sin(omega * t)
 *   Y 轴 pos_ref = A * cos(omega * t)
 *   即 XY 在相平面画一个圆 (半径 A, 角速度 omega)。
 *
 * 速度前馈 vel_ff = A * omega * cos/sin 沿轨迹切线方向。
 * 调 MotorAPI::writeSetpoint 而不是 appendSetpointSeg 时, 此模板就是单 MCU
 * 即时同步的 two-axis demo (Phase 2 已可用)。
 *
 * 走斜线测试:
 *   把 t 设为线性 ramp (pos_ref_X = v*t, pos_ref_Y = v*t), 即得 45° 斜线。
 *
 * 接电机:
 *   attachAxis({axis_id=0, local_api=&motor_x, remote_id=0});
 *   attachAxis({axis_id=1, local_api=&motor_y, remote_id=0});
 *   setAmplitude(0.5);  // rad
 *   setOmega(2.0 * 3.14159);  // rad/s, 一秒一圈
 *   while (running) { tick(1e-3); }
 *   预期: X-Y 在示波器上画一个圆。
 */

#include "SyncedAxisGroup_Template.h"

namespace Coordinator_Template
{

class DualAxisSinDemo : public SyncedAxisGroupBase
{
public:
    void setAmplitude(float amp) { amplitude_ = amp; }
    void setOmega(float omega)  { omega_   = omega; }

    bool scheduleSegment(double t_sec,
                         AxisSetpointSeg* out_segments,
                         uint16_t max_count,
                         uint16_t& out_count) override
    {
        if (max_count < 2) return false;

        /* X 轴 (axis_id=0): sin */
        MotionSetpoint sp_x;
        sp_x.mode    = Lib_Motor::Mode::POSITION_CONTROL;
        sp_x.pos_ref = amplitude_ * sinf(omega_ * static_cast<float>(t_sec));
        sp_x.vel_ff  = amplitude_ * omega_ * cosf(omega_ * static_cast<float>(t_sec));
        out_segments[0].axis_id = 0;
        out_segments[0].sp      = sp_x;

        /* Y 轴 (axis_id=1): cos */
        MotionSetpoint sp_y;
        sp_y.mode    = Lib_Motor::Mode::POSITION_CONTROL;
        sp_y.pos_ref = amplitude_ * cosf(omega_ * static_cast<float>(t_sec));
        sp_y.vel_ff  = -amplitude_ * omega_ * sinf(omega_ * static_cast<float>(t_sec));
        out_segments[1].axis_id = 1;
        out_segments[1].sp      = sp_y;

        out_count = 2;
        return true;
    }

private:
    float amplitude_ = 0.0f;
    float omega_    = 0.0f;
};

} // namespace Coordinator_Template