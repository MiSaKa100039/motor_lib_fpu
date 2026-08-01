cmake_minimum_required(VERSION 3.20)

project(motor_host_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(motor_host_test
    test_main.cpp
    ../../Lib/Motor/Core/API/Motor_API.cpp
    ../../Lib/Motor/Core/Debug/Motor_ManagerDebug.cpp
    ../../Lib/Motor/Core/Debug/Motor_OzoneTuning.cpp
    ../../Lib/Motor/Core/Feature/Motor_FeatureRegistry.cpp
    ../../Lib/Motor/Core/FSM/Motor_FSM.cpp
    ../../Lib/Motor/Core/FSM/AngleFSM/Motor_AngleFSM.cpp
    ../../Lib/Motor/Core/FSM/CommandFSM/Motor_CommandFSM.cpp
    ../../Lib/Motor/Core/FSM/EnergyFSM/Motor_EnergyFSM.cpp
    ../../Lib/Motor/Core/FSM/LifecycleFSM/Motor_LifecycleFSM.cpp
    ../../Lib/Motor/Core/FSM/RunPhaseFSM/Motor_RunPhaseFSM.cpp
    ../../Lib/Motor/Core/FSM/SafetyFSM/Motor_SafetyFSM.cpp
    ../../Lib/Motor/Core/FSM/StopFSM/Motor_StopFSM.cpp
    ../../Lib/Motor/Core/FSM/StreamFSM/Motor_StreamFSM.cpp
    ../../Lib/Motor/Core/Identify/Motor_RLIdentifyRoutine.cpp
    ../../Lib/Motor/Core/Limit/Motor_TargetLimiter.cpp
    ../../Lib/Motor/Core/Manager/Motor_Manager.cpp
    ../../Lib/Motor/Core/Manager/Motor_ManagerAPI.cpp
    ../../Lib/Motor/Core/Monitor/Motor_ManagerMonitor.cpp
    ../../Lib/Motor/Core/Numeric/Motor_FixedNumeric.cpp
    ../../Lib/Motor/Core/RunPlan/Motor_RunPlan.cpp
    ../../Lib/Motor/Core/Runtime/Motor_ManagerRuntime.cpp
    ../../Lib/Motor/Core/Safety/Motor_CommandGuard.cpp
    ../../Lib/Motor/Core/Safety/Motor_ConfigCheck.cpp
    ../../Lib/Motor/Core/Safety/Motor_FaultDetailString.cpp
    ../../Lib/Motor/Core/Safety/Motor_ManagerSafety.cpp
    ../../Lib/Motor/Core/Safety/Motor_RuntimeMonitor.cpp
    ../../Lib/Motor/Core/Safety/Motor_SignalHealth.cpp
    ../../Lib/Motor/Core/Sampling/Motor_ManagerSampling.cpp
    ../../Lib/Motor/Core/Sampling/Motor_SingleShunt.cpp
    ../../Lib/Motor/Core/Startup/Motor_ManagerStartup.cpp
    ../../Lib/Motor/Core/Startup/FlyingStart/Motor_FlyingStartRoutine.cpp
)

target_compile_definitions(motor_host_test PRIVATE
    MOTOR_LIB_HOST_TEST=1
    MOTOR_BUILD_CURRENT_SENSE_MODE=1
    MOTOR_BUILD_ENABLE_SENSOR
    MOTOR_BUILD_ENABLE_SMO
    MOTOR_BUILD_ENABLE_HFI
    MOTOR_BUILD_ENABLE_DANGEROUS_TEST_API
    MOTOR_BUILD_ENABLE_VELOCITY_CONTROL
)

target_include_directories(motor_host_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Public
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Common/Filter
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Common/Limit
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Common/Math
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Control
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Control/Utils
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Observer
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Core/Feature
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Core/Manager
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Core/Runtime
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Core/Safety
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Core/Sampling
    ${CMAKE_CURRENT_LIST_DIR}/../../Lib/Motor/Safety/Basic
    ${CMAKE_CURRENT_LIST_DIR}/../../Platform/Motor
    ${CMAKE_CURRENT_LIST_DIR}/../../Platform/Motor/BuildCfg
    ${CMAKE_CURRENT_LIST_DIR}/../../Platform/Motor/MotorCfg
    ${CMAKE_CURRENT_LIST_DIR}/../../Platform/Motor/TestCfg
)
