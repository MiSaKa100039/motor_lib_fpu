add_library(lca039_motor STATIC)

target_sources(lca039_motor PRIVATE
    Core/API/Motor_API.cpp
    Core/Debug/Motor_ManagerDebug.cpp
    Core/Debug/Motor_OzoneTuning.cpp
    Core/Feature/Motor_FeatureRegistry.cpp
    Core/FSM/Motor_FSM.cpp
    Core/FSM/AngleFSM/Motor_AngleFSM.cpp
    Core/FSM/CommandFSM/Motor_CommandFSM.cpp
    Core/FSM/EnergyFSM/Motor_EnergyFSM.cpp
    Core/FSM/LifecycleFSM/Motor_LifecycleFSM.cpp
    Core/FSM/RunPhaseFSM/Motor_RunPhaseFSM.cpp
    Core/FSM/SafetyFSM/Motor_SafetyFSM.cpp
    Core/FSM/StopFSM/Motor_StopFSM.cpp
    Core/FSM/StreamFSM/Motor_StreamFSM.cpp
    Core/Identify/Motor_RLIdentifyRoutine.cpp
    Core/Limit/Motor_TargetLimiter.cpp
    Core/Manager/Motor_Manager.cpp
    Core/Manager/Motor_ManagerAPI.cpp
    Core/Monitor/Motor_ManagerMonitor.cpp
    Core/Numeric/Motor_FixedNumeric.cpp
    Core/RunPlan/Motor_RunPlan.cpp
    Core/Runtime/Motor_ManagerRuntime.cpp
    Core/Safety/Motor_CommandGuard.cpp
    Core/Safety/Motor_ConfigCheck.cpp
    Core/Safety/Motor_FaultDetailString.cpp
    Core/Safety/Motor_ManagerSafety.cpp
    Core/Safety/Motor_RuntimeMonitor.cpp
    Core/Safety/Motor_SignalHealth.cpp
    Core/Sampling/Motor_ManagerSampling.cpp
    Core/Sampling/Motor_SingleShunt.cpp
    Core/Startup/Motor_ManagerStartup.cpp
    Core/Startup/FlyingStart/Motor_FlyingStartRoutine.cpp)

target_include_directories(lca039_motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Public
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Filter
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Limit
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Math
    ${CMAKE_CURRENT_SOURCE_DIR}/Control
    ${CMAKE_CURRENT_SOURCE_DIR}/Control/Utils
    ${CMAKE_CURRENT_SOURCE_DIR}/Observer
    ${CMAKE_CURRENT_SOURCE_DIR}/Safety/Basic
    ${CMAKE_SOURCE_DIR}/Platform/Motor
    ${CMAKE_SOURCE_DIR}/Platform/Motor/BuildCfg
    ${CMAKE_SOURCE_DIR}/Platform/Motor/MotorCfg
    ${CMAKE_SOURCE_DIR}/Platform/Motor/TestCfg)

target_compile_options(lca039_motor PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>)
