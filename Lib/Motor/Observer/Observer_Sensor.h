/*
 * Observer_Sensor.h — 外部位置传感器观测器 (最小封装)
 *
 * 职责:
 *   将任意 SensorInterface_t (HALL3/ABZ/AS5600) 的标准接口输出
 *   统一转换为 mechanical_angle + electrical_angle + speed + raw 四维数据。
 *
 * 核心公式:
 *   angle_elec = mech_angle × pole_pairs × direction + offset_elec
 *
 * 此观测器不包含任何滤波、插值、预测逻辑,
 * 只负责从传感器接口提取数据并做极对数和角度偏置转换。
 */

#pragma once

#include "../Public/Motor_Sensor_Interface.h"
#include <cmath>

namespace Lib_Motor
{

class SensorObserver
{
public:
    SensorObserver() = default;

    /*
     * init — 绑定传感器接口和极对数
     *
     * @param sensor     传感器接口指针 (含 is_ready / read_angle / read_velocity / get_raw 等回调)
     * @param pole_pairs 电机极对数, 用于机械角→电角度的倍率换算
     */
    void init(SensorInterface_t* sensor, uint8_t pole_pairs)
    {
        sensor_     = sensor;
        pole_pairs_ = pole_pairs;
    }

    /*
     * update — 每个控制 tick 调用, 从传感器读取最新数据
     *
     * 执行顺序:
     *   1. 传感器就绪检查 (nullptr / 接口回调缺失 / 传感器自身无效)
     *   2. 读取机械角度 (传感器输出, 已内部处理了 CPR 换算和插值)
     *   3. 读取速度 (可选, 无速度回调则返回 0)
     *   4. 读取原始值 (可选, 用于调试)
     *   5. 机械角 → 电角度: angle_elec = mech × P × dir + offset
     *   6. 角度归一化到 [0, 2π)
     *
     * @param angle_elec_out   [out] 电角度 (rad), 归一化到 [0, 2π)
     * @param speed_mech_out   [out] 机械转速 (rad/s), 若无速度回调则返回 0
     * @param angle_mech_out   [out] 机械角度 (rad), [0, 2π)
     * @param raw_out          [out] 传感器原始值 (如扇区号/ABZ 计数), 调试用
     *
     * @return true  传感器就绪, 输出值有效
     * @return false 传感器未就绪或不可用, 输出值保持旧值不变
     */
    bool update(float* angle_elec_out, float* speed_mech_out,
                float* angle_mech_out, int32_t* raw_out)
    {
        // ── [1] 传感器就绪检查 ──
        if (sensor_ == nullptr ||
            sensor_->is_ready == nullptr ||
            sensor_->read_angle == nullptr ||
            !sensor_->is_ready(sensor_->ctx))
        {
            return false;                            // 传感器不可用 → 保持旧值, 由调用方决定是否停机
        }

        // ── [2] 读取机械角度 ──
        // read_angle 返回已换算的机械角度 (传感器内部已处理 CPR/扇区到角度的转换)
        float mech_angle = sensor_->read_angle(sensor_->ctx);

        // ── [3] 读取速度 (可选) ──
        float speed_mech = 0.0f;
        if (sensor_->read_velocity != nullptr)
        {
            speed_mech = sensor_->read_velocity(sensor_->ctx);  // rad/s (机械)
        }

        // ── [4] 读取原始值 (可选) ──
        int32_t raw = 0;
        if (sensor_->get_raw != nullptr)
        {
            raw = sensor_->get_raw(sensor_->ctx);                // 如霍尔扇区号、ABZ 脉冲计数等
        }

        // ── [5] 机械角 → 电角度 ──
        //   angle_elec = mech × pole_pairs × direction + offset
        //   direction = 1 或 -1 (来自传感器 spec, 用于修正安装方向)
        //   offset    = 装配偏差 (来自传感器 spec, 对齐霍尔扇区 0 与 A 相磁场零点)
        int8_t dir = sensor_->spec.direction;
        float angle_elec = mech_angle * pole_pairs_ * dir + sensor_->spec.offset_elec;

        // ── [6] 归一化到 [0, 2π) ──
        while (angle_elec >= TWO_PI) angle_elec -= TWO_PI;
        while (angle_elec < 0.0f) angle_elec += TWO_PI;

        // 输出四维数据
        *angle_elec_out = angle_elec;
        *speed_mech_out = speed_mech;
        *angle_mech_out = mech_angle;
        *raw_out = raw;
        return true;
    }

private:
    SensorInterface_t* sensor_     = nullptr;    // 传感器接口指针, init() 时绑定
    uint8_t            pole_pairs_ = 0;          // 极对数, wheel→电角度的倍率
    static constexpr float TWO_PI = 6.28318530717958647692f;  // 2π
};

} // namespace Lib_Motor
