/*
 * Motor_SignalHealth.cpp -- 位置传感器健康检查与一致性判断
 *
 * 负责两类工作:
 *   1. 每个控制 tick 更新主传感器 / 冗余传感器 / 输出轴传感器状态
 *   2. 根据配置执行角度跳变、冗余一致性等健康检查
 */

#include "Motor_SignalHealth.h"

#include "../Manager/Motor_Manager.h"
#include "../../Common/Math/FocMath.h"

#include <cmath>

namespace Lib_Motor
{

namespace
{

/*
 * 将任意角度折返到 [-PI, PI] 区间, 便于做差值比较。
 */
float wrapSignedAngle(float angle)
{
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

} // namespace

/*
 * 更新位置相关传感器状态。
 *
 * 顺序:
 *   [1] 主转子传感器
 *   [2] 冗余转子传感器
 *   [3] 输出轴传感器
 *   [4] 派生量: 冗余角误差、传动挠度
 */
void MotorSignalHealth::updatePositionSensors(MotorManager& m)
{
    const bool primary_was_ready = m.ctx_.sensor_ready;
    const float previous_primary_angle = m.ctx_.angle_mech_sensor;
    m.ctx_.sensor_ready = false;
    m.ctx_.sensor_health = m.config_.position.rotor_sensor == nullptr
        ? SensorHealth::NOT_CONFIGURED
        : SensorHealth::NOT_READY;

    /*
     * [1] 主转子传感器:
     *   - 调用 SensorObserver::update() 获取当前角度与速度
     *   - 若开启角度跳变检查, 则与上一拍机械角比较
     *   - 仅在健康状态为 OK 时回写 ctx
     */
    if (m.config_.position.rotor_sensor != nullptr)
    {
        float angle_elec = m.ctx_.angle_elec_sensor;
        float speed_mech_rad_s = 0.0f;
        float angle_mech = m.ctx_.angle_mech_sensor;
        int32_t raw = m.ctx_.sensor_raw;
        const bool updated = m.rotor_sensor_obs_.update(
            &angle_elec, &speed_mech_rad_s, &angle_mech, &raw);

        // 若 BSP 提供 get_health, 优先使用 BSP 健康结果; 否则用 update() 是否成功作为兜底。
        m.ctx_.sensor_health = m.config_.position.rotor_sensor->get_health != nullptr
            ? m.config_.position.rotor_sensor->get_health(m.config_.position.rotor_sensor->ctx)
            : (updated ? SensorHealth::OK : SensorHealth::NOT_READY);

        if (updated)
        {
            // 角度跳变检测。
            if (m.ctx_.sensor_health == SensorHealth::OK &&
                m.config_.position.health.enable_angle_step_check &&
                m.config_.position.health.max_mech_angle_step_rad > 0.0f &&
                primary_was_ready &&
                fabsf(wrapSignedAngle(angle_mech - previous_primary_angle)) >
                    m.config_.position.health.max_mech_angle_step_rad)
            {
                m.ctx_.sensor_health = SensorHealth::ANGLE_JUMP;
            }

            if (m.ctx_.sensor_health == SensorHealth::OK)
            {
                m.ctx_.sensor_ready = true;
                m.ctx_.angle_elec_sensor = angle_elec;
                m.ctx_.angle_mech_sensor = angle_mech;
                m.ctx_.sensor_raw = raw;
                m.ctx_.speed_rpm_sensor = speed_mech_rad_s * 60.0f / TWO_PI;
            }
        }
    }

    /*
     * [2] 冗余转子传感器:
     *   - 仅在 BuildCfg 与 MotorCfg 都启用时参与更新
     *   - 当前只采集角度与健康状态, 一致性检查放在后续步骤
     */
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    m.ctx_.redundant_sensor_ready = false;
    m.ctx_.redundant_sensor_health = m.config_.position.enable_redundant_rotor_sensor
        ? SensorHealth::NOT_READY
        : SensorHealth::NOT_CONFIGURED;
    if (m.config_.position.enable_redundant_rotor_sensor &&
        m.config_.position.redundant_rotor_sensor != nullptr)
    {
        float angle_elec = m.ctx_.angle_elec_redundant_sensor;
        float speed_unused = 0.0f;
        float angle_mech = m.ctx_.angle_mech_redundant_sensor;
        int32_t raw = m.ctx_.redundant_sensor_raw;
        const bool updated = m.redundant_sensor_obs_.update(
            &angle_elec, &speed_unused, &angle_mech, &raw);

        m.ctx_.redundant_sensor_health =
            m.config_.position.redundant_rotor_sensor->get_health != nullptr
                ? m.config_.position.redundant_rotor_sensor->get_health(
                    m.config_.position.redundant_rotor_sensor->ctx)
                : (updated ? SensorHealth::OK : SensorHealth::NOT_READY);

        if (updated && m.ctx_.redundant_sensor_health == SensorHealth::OK)
        {
            m.ctx_.redundant_sensor_ready = true;
            m.ctx_.angle_elec_redundant_sensor = angle_elec;
            m.ctx_.angle_mech_redundant_sensor = angle_mech;
            m.ctx_.redundant_sensor_raw = raw;
        }
    }
#endif

    /*
     * [3] 输出轴传感器:
     *   - 用于减速后 / 负载侧位置反馈
     *   - 当前只更新机械角与健康状态
     */
#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    m.ctx_.output_sensor_ready = false;
    m.ctx_.output_sensor_health = m.config_.position.enable_output_sensor
        ? SensorHealth::NOT_READY
        : SensorHealth::NOT_CONFIGURED;
    if (m.config_.position.enable_output_sensor &&
        m.config_.position.output_sensor != nullptr)
    {
        float angle_unused = 0.0f;
        float speed_unused = 0.0f;
        float angle_mech = m.ctx_.angle_mech_output_sensor;
        int32_t raw = m.ctx_.output_sensor_raw;
        const bool updated = m.output_sensor_obs_.update(
            &angle_unused, &speed_unused, &angle_mech, &raw);

        m.ctx_.output_sensor_health = m.config_.position.output_sensor->get_health != nullptr
            ? m.config_.position.output_sensor->get_health(m.config_.position.output_sensor->ctx)
            : (updated ? SensorHealth::OK : SensorHealth::NOT_READY);

        if (updated && m.ctx_.output_sensor_health == SensorHealth::OK)
        {
            m.ctx_.output_sensor_ready = true;
            m.ctx_.angle_mech_output_sensor = angle_mech;
            m.ctx_.output_sensor_raw = raw;
        }
    }
#endif

    /*
     * [4] 派生量:
     *   - rotor_redundancy_error: 主传感器与冗余传感器的电角误差
     *   - transmission_deflection_rad: 电机侧折算角与输出轴角的差值
     */
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    m.ctx_.rotor_redundancy_error =
        (m.ctx_.sensor_ready && m.ctx_.redundant_sensor_ready)
            ? wrapSignedAngle(
                m.ctx_.angle_elec_sensor - m.ctx_.angle_elec_redundant_sensor)
            : 0.0f;
#endif

#if MOTOR_BUILD_ENABLE_OUTPUT_SENSOR
    m.ctx_.transmission_deflection_rad =
        (m.ctx_.sensor_ready && m.ctx_.output_sensor_ready &&
         m.config_.position.transmission_ratio > 0.0f)
            ? wrapSignedAngle(
                m.ctx_.angle_mech_sensor / m.config_.position.transmission_ratio -
                m.ctx_.angle_mech_output_sensor)
            : 0.0f;
#endif
}

/*
 * 检查主传感器与冗余传感器的一致性。
 *
 * 当电角误差持续超过阈值并达到确认 tick 数后:
 *   - 置位 Fault::SENSOR_MISMATCH
 *   - 将冗余传感器健康状态标记为 REDUNDANT_MISMATCH
 */
void MotorSignalHealth::checkPositionConsistency(MotorManager& m)
{
#if MOTOR_BUILD_ENABLE_REDUNDANT_SENSOR
    if (m.config_.position.health.enable_redundant_check &&
        m.ctx_.sensor_ready &&
        m.ctx_.redundant_sensor_ready &&
        fabsf(m.ctx_.rotor_redundancy_error) >
            m.config_.position.health.max_redundant_error_rad)
    {
        if (++m.redundant_error_ticks_ >=
            m.config_.position.health.redundant_confirm_ticks)
        {
            m.setFault(Fault::SENSOR_MISMATCH);
            m.ctx_.redundant_sensor_health = SensorHealth::REDUNDANT_MISMATCH;
        }
    }
    else
    {
        m.redundant_error_ticks_ = 0U;
    }
#else
    (void)m;
#endif
}

} // namespace Lib_Motor
