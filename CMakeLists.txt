cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME motor_lib)

include(cmake/gcc-arm-none-eabi.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

option(USERAPP_ENABLE_SETPOINT_STREAM "Enable UserAPP setpoint stream dispatch" ON)
option(USERAPP_ENABLE_MOTION_COORDINATOR "Enable UserAPP multi-axis motion coordinator" OFF)
option(USERAPP_ENABLE_ENERGY_COORDINATOR "Enable UserAPP energy coordinator" OFF)
option(USERAPP_ENABLE_SYNC_COORDINATOR "Enable UserAPP distributed sync coordinator" OFF)

include(cmake/motor_source_sets.cmake)
motor_configure_source_set(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    USERAPP_ENABLE_SETPOINT_STREAM=$<BOOL:${USERAPP_ENABLE_SETPOINT_STREAM}>
    USERAPP_ENABLE_MOTION_COORDINATOR=$<BOOL:${USERAPP_ENABLE_MOTION_COORDINATOR}>
    USERAPP_ENABLE_ENERGY_COORDINATOR=$<BOOL:${USERAPP_ENABLE_ENERGY_COORDINATOR}>
    USERAPP_ENABLE_SYNC_COORDINATOR=$<BOOL:${USERAPP_ENABLE_SYNC_COORDINATOR}>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# ---------------- 头文件路径 ----------------
set(Include_Dirs
        # === Lib: Motor ===
        Lib/Motor
        Lib/Motor/Public
        Lib/Motor/Common
        Lib/Motor/Common/Math
        Lib/Motor/Common/Filter
        Lib/Motor/Common/Limit
        Lib/Motor/Core
        Lib/Motor/Core/API
        Lib/Motor/Core/Feature
        Lib/Motor/Core/RunPlan
        Lib/Motor/Core/Limit
        Lib/Motor/Core/Manager
        Lib/Motor/Core/Monitor
        Lib/Motor/Core/Runtime
        Lib/Motor/Core/Sampling
        Lib/Motor/Core/Startup
        Lib/Motor/Core/Debug
        Lib/Motor/Core/Safety
        Lib/Motor/Control
        Lib/Motor/Control/Utils
        Lib/Motor/Observer
        Lib/Motor/Safety
        Lib/Motor/Safety/Basic
        Lib/Motor/Utils

        # === Lib: Sensor ===
        Lib/Sensor

        # === Lib: VOFA ===
        Lib/Communication/VOFA
        Lib/Communication/VOFA/Public
        Lib/Communication/VOFA/Core

        # === Platform ===
        Platform/Motor
        Platform/Motor/BuildCfg
        Platform/Motor/MotorCfg
        Platform/Motor/TestCfg
        Platform/Sensor
        Platform/Sensor/AS5600
        Platform/Sensor/ABZ
        Platform/Sensor/HALL3
        Platform/Communication/VOFA

        # === BSP ===
        BSP/Motor
        BSP/Sensor/BSP_AS5600
        BSP/Sensor/BSP_ABZ
        BSP/Sensor/BSP_HALL3
        BSP/Communication/VOFA

        # === UserAPP ===
        UserAPP
)

# ---------------- 应用层源码 ----------------
motor_collect_application_sources(Application_Src Application_CXX_Src)

target_sources(${CMAKE_PROJECT_NAME}
        PRIVATE ${Application_Src}
        PRIVATE ${Application_CXX_Src}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${Include_Dirs})
