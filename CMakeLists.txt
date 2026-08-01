cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME lca039_motorlib)
include(cmake/gcc-arm-none-eabi.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
include(cmake/lca039/clock_config.cmake)

set(LCA039_FIRMWARE_MODE "NORMAL" CACHE STRING "Firmware mode: NORMAL or TEST")
set_property(CACHE LCA039_FIRMWARE_MODE PROPERTY STRINGS NORMAL TEST)
set(LCA039_TARGET_TEST_CASE "SAFE_SELF_TEST" CACHE STRING "Target test case")
set_property(CACHE LCA039_TARGET_TEST_CASE PROPERTY STRINGS
    SAFE_SELF_TEST ADC_RAW PWM_DRY_RUN TIM_ADC_VOFA CURRENT_OFFSET PWM_MANUAL
    CURRENT_LOCK VF_CONTROL IF_CONTROL IF_SMO_OBSERVER RL_IDENTIFY)
option(LCA039_TEST_ENABLE_DANGEROUS "Compile power-stage target tests" OFF)
option(MOTOR_PLATFORM_ENABLE_VOFA "Build VOFA telemetry adapters" ON)

set(LCA039_COMMUNICATION_CONFIG_PATH
    "${CMAKE_SOURCE_DIR}/Platform/Communication/Communication_Config.h")
set(LCA039_COMMUNICATION_VOFA_ENABLED OFF)
if(MOTOR_PLATFORM_ENABLE_VOFA AND EXISTS "${LCA039_COMMUNICATION_CONFIG_PATH}")
    file(READ "${LCA039_COMMUNICATION_CONFIG_PATH}" LCA039_COMMUNICATION_CONFIG_TEXT)
    string(REGEX MATCH "(^|\n)[ \t]*#[ \t]*define[ \t]+BSP_VOFA_ENABLE([ \t\r\n]|$)"
        LCA039_COMMUNICATION_VOFA_DEFINE
        "${LCA039_COMMUNICATION_CONFIG_TEXT}")
    if(LCA039_COMMUNICATION_VOFA_DEFINE)
        set(LCA039_COMMUNICATION_VOFA_ENABLED ON)
    endif()
endif()

string(TOUPPER "${LCA039_FIRMWARE_MODE}" LCA039_FIRMWARE_MODE_SELECTED)
string(TOUPPER "${LCA039_TARGET_TEST_CASE}" LCA039_TARGET_TEST_CASE_SELECTED)
set(LCA039_FIRMWARE_MODES NORMAL TEST)
if(NOT LCA039_FIRMWARE_MODE_SELECTED IN_LIST LCA039_FIRMWARE_MODES)
    message(FATAL_ERROR "Unsupported LCA039_FIRMWARE_MODE='${LCA039_FIRMWARE_MODE}'")
endif()
set(LCA039_TARGET_TEST_CASES
    SAFE_SELF_TEST ADC_RAW PWM_DRY_RUN TIM_ADC_VOFA CURRENT_OFFSET PWM_MANUAL
    CURRENT_LOCK VF_CONTROL IF_CONTROL IF_SMO_OBSERVER RL_IDENTIFY)
if(NOT LCA039_TARGET_TEST_CASE_SELECTED IN_LIST LCA039_TARGET_TEST_CASES)
    message(FATAL_ERROR "Unsupported LCA039_TARGET_TEST_CASE='${LCA039_TARGET_TEST_CASE}'")
endif()
set(LCA039_POWER_STAGE_TEST_CASES
    CURRENT_OFFSET PWM_MANUAL CURRENT_LOCK VF_CONTROL IF_CONTROL IF_SMO_OBSERVER RL_IDENTIFY)
if(LCA039_FIRMWARE_MODE_SELECTED STREQUAL "TEST" AND
   LCA039_TARGET_TEST_CASE_SELECTED IN_LIST LCA039_POWER_STAGE_TEST_CASES AND
   NOT LCA039_TEST_ENABLE_DANGEROUS)
    message(FATAL_ERROR "${LCA039_TARGET_TEST_CASE_SELECTED} requires LCA039_TEST_ENABLE_DANGEROUS=ON")
endif()
if(LCA039_TARGET_TEST_CASE_SELECTED STREQUAL "TIM_ADC_VOFA" AND
   NOT LCA039_COMMUNICATION_VOFA_ENABLED)
    message(FATAL_ERROR "TIM_ADC_VOFA requires BSP_VOFA_ENABLE and MOTOR_PLATFORM_ENABLE_VOFA=ON")
endif()

#add_subdirectory(Lib/FixedPoint)
add_subdirectory(Lib/Motor)
#add_subdirectory(Lib/LIN)
#add_subdirectory(Lib/PwmInput)
if(LCA039_COMMUNICATION_VOFA_ENABLED)
    add_subdirectory(Lib/Communication/VOFA)
endif()

add_executable(${CMAKE_PROJECT_NAME}
    Core/Src/main.c
    Core/Src/lca039_clock.c
    Core/Src/syscalls.c
    Core/Src/lcm32f039_it.c
    startup_lcm32f039_gcc.S
    BSP/Motor/BSP_Motor.cpp
    Platform/Motor/Platform_Motor.cpp
    Platform/Motor/TestCfg/Platform_MotorTestConfig.cpp)

if(LCA039_FIRMWARE_MODE_SELECTED STREQUAL "NORMAL")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        UserAPP/Usermain.cpp)
else()
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Tests/Target/TargetTest.cpp)
endif()

if(LCA039_COMMUNICATION_VOFA_ENABLED)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        BSP/Communication/VOFA/BSP_VOFA.cpp
        Platform/Communication/VOFA/Platform_VOFA.cpp)
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    UserAPP
    BSP/Motor
    BSP/Communication/VOFA
    BSP/Sensor/BSP_HALL3
    BSP/Sensor/BSP_ABZ
    BSP/Sensor/BSP_AS5600
    Lib/Motor
    Lib/Motor/Core/Manager
    Lib/Motor/Core/Runtime
    Lib/Motor/Core/Sampling
    Lib/Motor/Public
    Lib/Motor/Control/Utils
    Lib/Motor/Common/Math
    Lib/Sensor
    Lib/Sensor/HALL3
    Lib/Sensor/ABZ
    Lib/Sensor/AS5600
    Platform/Sensor
    Platform/Sensor/HALL3
    Platform/Sensor/ABZ
    Platform/Sensor/AS5600
    Platform/Motor
    Platform/Motor/BuildCfg
    Platform/Motor/MotorCfg
    Platform/Motor/TestCfg
    Platform/Communication
    Platform/Communication/VOFA
    Tests/Target
    ${LCA039_GENERATED_INCLUDE_DIR})

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    LCA039_FIRMWARE_MODE_${LCA039_FIRMWARE_MODE_SELECTED}=1
    LCA039_TARGET_TEST_ENABLE_DANGEROUS=$<BOOL:${LCA039_TEST_ENABLE_DANGEROUS}>
    MOTOR_PLATFORM_ENABLE_VOFA=$<BOOL:${LCA039_COMMUNICATION_VOFA_ENABLED}>
    CORE_ENABLE_USART1_TX_DMA=$<BOOL:${LCA039_COMMUNICATION_VOFA_ENABLED}>)
if(LCA039_FIRMWARE_MODE_SELECTED STREQUAL "TEST")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        LCA039_TARGET_TEST_${LCA039_TARGET_TEST_CASE_SELECTED}=1)
endif()

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Werror>
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Werror>)

add_subdirectory(cmake/lca039)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${LCA039_ARCH_OPTIONS}
    "-T${CMAKE_SOURCE_DIR}/LCM32F039_FLASH.ld"
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,--no-warn-execstack
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map")
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS "${CMAKE_SOURCE_DIR}/LCM32F039_FLASH.ld")

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    lca039_motor
    lcm32f039_stdperiph
    lcm32f039_cmsis
    m)
if(LCA039_COMMUNICATION_VOFA_ENABLED)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE lca039_vofa)
endif()

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES
        "${CMAKE_PROJECT_NAME}.map;${CMAKE_PROJECT_NAME}.hex;${CMAKE_PROJECT_NAME}.bin")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating HEX/BIN images and reporting image size")

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Firmware mode: ${LCA039_FIRMWARE_MODE_SELECTED}")
message(STATUS "Debug tuning: enabled except in Release")
