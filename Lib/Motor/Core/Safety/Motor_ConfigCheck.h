#pragma once

#include "../../Public/Motor_Config.h"

namespace Lib_Motor
{

/*
 * MotorConfigCheck -- 配置安全校验器
 *
 * 职责: 上电前验证 MotorConfig 参数完整性, 防止硬件/参数配置错误导致失控。
 * 所有方法返回 Fault::NONE 表示通过, 其他表示具体故障码。
 */
class MotorConfigCheck
{
public:
    /* validateBuildConfig -- 校验 BuildCfg 宏配置合法性 (采样模式/NTC 通道数/占位能力) */
    static Fault validateBuildConfig(MotorConfigFaultDetail* detail = nullptr);

    /* validateBase -- 校验基础硬件配置 (HAL 接口/极对数/控制频率/传感器/LPF/电压采样) */
    static Fault validateBase(const MotorConfig& cfg,
                              MotorConfigFaultDetail* detail = nullptr);

    /* validateFeedback -- 校验反馈路径配置 (观测器/传感器/冗余参数), 重载版本 */
    static Fault validateFeedback(const MotorConfig& cfg,
                                  MotorConfigFaultDetail* detail = nullptr);
    static Fault validateFeedback(const MotorConfig& cfg,
                                  const MotorRunPolicy& policy,
                                  MotorConfigFaultDetail* detail = nullptr);
    static Fault validateFeedback(const MotorHardwareConfig& hardware,
                                  const MotorAlgorithmParam& algorithm,
                                  const MotorRunPolicy& policy,
                                  MotorConfigFaultDetail* detail = nullptr);

    /* validateStartupTargetMode -- 校验启动目标模式是否合法 (速度环/力矩环/位置环) */
    static Fault validateStartupTargetMode(const MotorConfig& cfg,
                                           MotorConfigFaultDetail* detail = nullptr);

    /* validSmoObserverParams -- 校验 SMO 观测器自身参数, 供闭环和 shadow 调试共用 */
    static bool validSmoObserverParams(const MotorPhysicalParam& physical,
                                       const MotorObserverParam& observer);

    /* validIFStartupProfile -- 校验 IF 对齐首段和后续拖动段 */
    static bool validIFStartupProfile(const MotorIFStartupProfile* profile);

    /* validateInitConfig -- 总入口: 依次调用 BuildConfig/Base/Feedback/TargetMode 全量校验 */
    static Fault validateInitConfig(const MotorConfig& cfg,
                                    MotorConfigFaultDetail* detail = nullptr);

    /* isSteadySourceCompiled -- 查询某稳态角度源是否已编译进固件 */
    static bool  isSteadySourceCompiled(SteadyAngleSource source);

private:
    /* validSensor -- 检查传感器接口指针有效性 (非空 + read_angle/is_ready 非空) */
    static bool validSensor(const SensorInterface_t* sensor);
};

} // namespace Lib_Motor
