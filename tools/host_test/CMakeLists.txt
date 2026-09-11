cmake_minimum_required(VERSION 3.20)

project(motor_host_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_executable(motor_host_test
    test_main.cpp
    ../../Lib/Motor/Core/API/Motor_API.cpp
    ../../Lib/Motor/Core/Debug/Motor_ManagerDebug.cpp
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
    ../../Lib/Motor/Core/Runtime/Motor_Runtime.cpp
    ../../Lib/Motor/Core/Safety/Motor_CommandGuard.cpp
    ../../Lib/Motor/Core/Safety/Motor_ConfigCheck.cpp
    ../../Lib/Motor/Core/Safety/Motor_FaultDetailString.cpp
    ../../Lib/Motor/Core/Safety/Motor_ManagerSafety.cpp
    ../../Lib/Motor/Core/Safety/Motor_RuntimeMonitor.cpp
    ../../Lib/Motor/Core/Safety/Motor_SignalHealth.cpp
    ../../Lib/Motor/Core/Sampling/Motor_ManagerSampling.cpp
    ../../Lib/Motor/Core/Startup/Motor_ManagerStartup.cpp
    ../../Lib/Motor/Core/Startup/FlyingStart/Motor_FlyingStartRoutine.cpp
)

target_compile_definitions(motor_host_test PRIVATE
    LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15=0
    LIB_MOTOR_NUMERIC_BACKEND_FLOAT32=1
    MOTOR_LIB_HOST_TEST=1
    MOTOR_BUILD_CURRENT_SENSE_MODE=2
    MOTOR_BUILD_HAS_BUS_CURRENT=1
    MOTOR_BUILD_NTC_SLOTS=1
    MOTOR_BUILD_ENABLE_SENSOR
    MOTOR_BUILD_ENABLE_HFI
    MOTOR_BUILD_ENABLE_DANGEROUS_TEST_API
    LIB_MOTOR_ENABLE_SMO=1
    LIB_MOTOR_ENABLE_VELOCITY_CONTROL=1
    LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER=1
)

# 原有测试使用浮点物理量；定点回归单独编译，避免启用尚未移植的 HFI/有感组合。
get_target_property(motor_test_sources motor_host_test SOURCES)
list(REMOVE_ITEM motor_test_sources test_main.cpp)
add_executable(motor_q15_test ${motor_test_sources}
    test_entry.cpp test_smo_q15.cpp test_stop.cpp test_if_startup.cpp)
target_compile_definitions(motor_q15_test PRIVATE
    MOTOR_LIB_HOST_TEST=1
    LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15=1
    LIB_MOTOR_NUMERIC_BACKEND_FLOAT32=0
    LIB_MOTOR_ENABLE_IF_STARTUP=0
    LIB_MOTOR_ENABLE_SMO=0
    LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL=0
    LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER=1
    LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER=0
    LIB_MOTOR_ENABLE_SENSOR=0
    LIB_MOTOR_ENABLE_HFI=0
    LIB_MOTOR_ENABLE_VELOCITY_CONTROL=0
    LIB_MOTOR_ENABLE_STALL_PROTECTION=0)

target_sources(motor_host_test PRIVATE test_stop.cpp test_if_startup.cpp)

foreach(if_backend IN ITEMS float q15)
    add_executable(motor_if_startup_${if_backend}_test
        ${motor_test_sources}
        test_entry.cpp
        test_if_startup.cpp)
    target_compile_definitions(motor_if_startup_${if_backend}_test PRIVATE
        MOTOR_LIB_HOST_TEST=1
        MOTOR_BUILD_CURRENT_SENSE_MODE=2
        MOTOR_BUILD_HAS_BUS_CURRENT=1
        MOTOR_BUILD_NTC_SLOTS=1
        LIB_MOTOR_ENABLE_IF_STARTUP=1
        LIB_MOTOR_ENABLE_SMO=1
        LIB_MOTOR_ENABLE_VELOCITY_CONTROL=1
        LIB_MOTOR_ENABLE_DEBUG_IF_CONTROL=0
        LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER=0
        LIB_MOTOR_ENABLE_DEBUG_IF_HFI_OBSERVER=0
        LIB_MOTOR_ENABLE_SENSOR=0
        LIB_MOTOR_ENABLE_HFI=0
        LIB_MOTOR_ENABLE_STALL_PROTECTION=0)
endforeach()

target_compile_definitions(motor_if_startup_float_test PRIVATE
    LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15=0
    LIB_MOTOR_NUMERIC_BACKEND_FLOAT32=1)
target_compile_definitions(motor_if_startup_q15_test PRIVATE
    LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15=1
    LIB_MOTOR_NUMERIC_BACKEND_FLOAT32=0)

add_executable(motor_if_debug_float_test
    ${motor_test_sources}
    test_entry.cpp
    test_if_startup.cpp)
target_compile_definitions(motor_if_debug_float_test PRIVATE
    MOTOR_LIB_HOST_TEST=1
    MOTOR_BUILD_CURRENT_SENSE_MODE=2
    MOTOR_BUILD_HAS_BUS_CURRENT=1
    MOTOR_BUILD_NTC_SLOTS=1
    LIB_MOTOR_NUMERIC_BACKEND_FIXED_Q15=0
    LIB_MOTOR_NUMERIC_BACKEND_FLOAT32=1
    LIB_MOTOR_ENABLE_IF_STARTUP=0
    LIB_MOTOR_ENABLE_SMO=0
    LIB_MOTOR_ENABLE_DEBUG_IF_SMO_OBSERVER=1
    LIB_MOTOR_ENABLE_SENSOR=0
    LIB_MOTOR_ENABLE_HFI=0
    LIB_MOTOR_ENABLE_STALL_PROTECTION=0)

add_executable(motor_pwm_output_test test_pwm_outputs.cpp test_entry.cpp)
target_include_directories(motor_pwm_output_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/mocks
    ${CMAKE_CURRENT_LIST_DIR}/../../Core/Inc)

enable_testing()
add_test(NAME motor_float COMMAND motor_host_test)
add_test(NAME motor_q15 COMMAND motor_q15_test)
add_test(NAME motor_if_startup_float COMMAND motor_if_startup_float_test)
add_test(NAME motor_if_startup_q15 COMMAND motor_if_startup_q15_test)
add_test(NAME motor_if_debug_float COMMAND motor_if_debug_float_test)
add_test(NAME motor_pwm_outputs COMMAND motor_pwm_output_test)

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

get_target_property(motor_test_includes motor_host_test INCLUDE_DIRECTORIES)
target_include_directories(motor_q15_test PRIVATE ${motor_test_includes})
target_include_directories(motor_if_startup_float_test PRIVATE ${motor_test_includes})
target_include_directories(motor_if_startup_q15_test PRIVATE ${motor_test_includes})
target_include_directories(motor_if_debug_float_test PRIVATE ${motor_test_includes})
