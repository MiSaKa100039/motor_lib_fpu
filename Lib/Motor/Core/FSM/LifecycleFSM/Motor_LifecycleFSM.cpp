#include "Motor_LifecycleFSM.h"

#include "../../Feature/Motor_FeatureRegistry.h"
#include "../../Manager/Motor_Manager.h"
#include "../AngleFSM/Motor_AngleFSM.h"
#include "../RunPhaseFSM/Motor_RunPhaseFSM.h"
#include "../StopFSM/Motor_StopFSM.h"
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
#include "../EnergyFSM/Motor_EnergyFSM.h"
#endif

namespace Lib_Motor
{

Motor_LifecycleFSM::StartRequestKind Motor_LifecycleFSM::classifyStartRequest(MotorManager& m)
{
    const Mode target_mode = m.targetMode();

    if (!MotorFeatureRegistry::supportsMode(target_mode))
    {
        return StartRequestKind::Invalid;
    }

    switch (target_mode)
    {
        case Mode::DEBUG_PWM_MANUAL:
        case Mode::DEBUG_CURRENT_LOCK:
        case Mode::DEBUG_HFI_OBSERVER:
        case Mode::DEBUG_VF_DRAG:
        case Mode::DEBUG_IF_DRAG:
        case Mode::DEBUG_IF_SMO_OBSERVER:
        case Mode::DEBUG_IF_HFI_OBSERVER:
            return StartRequestKind::Debug;

        case Mode::TORQUE_CONTROL:
        case Mode::VELOCITY_CONTROL:
        case Mode::POSITION_CONTROL:
#if LIB_MOTOR_ENABLE_IMPEDANCE_CONTROL
        case Mode::IMPEDANCE_CONTROL:
#endif
            return StartRequestKind::Control;

        case Mode::CALIB_RL_IDENTIFY:
            return StartRequestKind::Calibration;

        case Mode::NONE:
            return StartRequestKind::None;

        default:
            return StartRequestKind::Invalid;
    }
}

void Motor_LifecycleFSM::enterRun(MotorManager& m, int initial_phase_value)
{
    Motor_StopFSM::clear(m);
#if LIB_MOTOR_ENABLE_BRAKE_ENERGY_FSM
    Motor_EnergyFSM::clear(m);
#else
    m.energy_state_ = EnergyState::IDLE;
#endif
    Motor_AngleFSM::clear(m);

    if (m.config_.hal != nullptr && m.config_.hal->pwm_enable != nullptr)
    {
        m.config_.hal->pwm_enable();
    }

    m.ctx_.fsm_timer_ticks = 0U;

#if LIB_MOTOR_ENABLE_IF_STARTUP || LIB_MOTOR_ENABLE_DANGEROUS_TEST_API
    m.if_angle_gen_.reset();
#endif

    m.setRunPhase(static_cast<RunPhase>(initial_phase_value));
    m.setState(State::RUN);
}

bool Motor_LifecycleFSM::shouldStopRun(const MotorManager& m)
{
    return !m.run_requested_ ||
           m.command_state_ == CommandState::STOP_REQUESTED ||
           m.targetMode() == Mode::NONE;
}

void Motor_LifecycleFSM::step(MotorManager& m)
{
    switch (m.state())
    {
        case State::INIT:
            handleInit(m);
            break;

        case State::ADC_CAL:
            handleAdcCal(m);
            break;

        case State::STOP:
            handleStop(m);
            break;

        case State::RUN:
            handleRun(m);
            break;

        case State::STOPPING:
            handleStopping(m);
            break;

        case State::ERROR:
            handleError(m);
            break;

        case State::ISR_DIAGNOSTIC:
            break;

        default:
            handleInit(m);
            break;
    }
}

void Motor_LifecycleFSM::handleInit(MotorManager& m)
{
    m.ctx_.calib_counter = 0U;
    m.ctx_.calib_accum_ia = 0.0f;
    m.ctx_.calib_accum_ib = 0.0f;
    m.ctx_.calib_accum_ic = 0.0f;
    m.ctx_.calib_accum_single_shunt_first = 0.0f;
    m.ctx_.calib_accum_single_shunt_second = 0.0f;
#if MOTOR_BUILD_HAS_BUS_CURRENT
    m.ctx_.calib_accum_ibus = 0.0f;
#endif
    m.setState(State::ADC_CAL);
}

void Motor_LifecycleFSM::handleAdcCal(MotorManager& m)
{
    if (m.ctx_.calib_counter >= m.config_.sensor.adc_calibration_samples)
    {
        m.setState(State::STOP);
    }
}

void Motor_LifecycleFSM::handleStop(MotorManager& m)
{
    if (!m.run_requested_)
    {
        return;
    }

    switch (classifyStartRequest(m))
    {
        case StartRequestKind::Control:
            if (!m.validateControlFeedbackSource())
            {
                return;
            }
            enterRun(m, static_cast<int>(Motor_RunPhaseFSM::selectInitialPhase(m, false)));
            return;

        case StartRequestKind::Debug:
            enterRun(m, static_cast<int>(Motor_RunPhaseFSM::selectInitialPhase(m, true)));
            return;

        case StartRequestKind::Calibration:
            enterRun(m, static_cast<int>(RunPhase::ALIGNMENT));
            return;

        case StartRequestKind::None:
            m.run_requested_ = false;
            return;

        case StartRequestKind::Invalid:
        default:
            m.stopForFault(Fault::PARAM_ERROR);
            return;
    }
}

void Motor_LifecycleFSM::handleRun(MotorManager& m)
{
    if (shouldStopRun(m))
    {
        m.transitionToStop(m.stop_mode_, true);
        return;
    }

#if LIB_MOTOR_ENABLE_AUTO_IDENTIFY
    if (m.targetMode() == Mode::CALIB_RL_IDENTIFY)
    {
        if (m.event_.rl_identify_done != 0U)
        {
            m.transitionToStop(StopMode::COAST, true, false);
        }
        return;
    }
#endif

    Motor_RunPhaseFSM::step(m);
}

void Motor_LifecycleFSM::handleStopping(MotorManager& m)
{
    Motor_StopFSM::step(m);
}

void Motor_LifecycleFSM::handleError(MotorManager& m)
{
    (void)m;
}

} // namespace Lib_Motor
