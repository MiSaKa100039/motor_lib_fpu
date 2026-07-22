set(MOTOR_LIB_SOURCE_SET "FULL" CACHE STRING "Motor library source set: FULL or CORE")
set_property(CACHE MOTOR_LIB_SOURCE_SET PROPERTY STRINGS FULL CORE)

if(DEFINED MOTOR_LIB_PROFILE)
    string(TOUPPER "${MOTOR_LIB_PROFILE}" _motor_legacy_profile)
    if(_motor_legacy_profile STREQUAL "FULL")
        set(MOTOR_LIB_SOURCE_SET "FULL" CACHE STRING "Motor library source set: FULL or CORE" FORCE)
    else()
        set(MOTOR_LIB_SOURCE_SET "CORE" CACHE STRING "Motor library source set: FULL or CORE" FORCE)
    endif()
    message(WARNING "MOTOR_LIB_PROFILE is deprecated; use MOTOR_LIB_SOURCE_SET=FULL or CORE.")
endif()

string(TOUPPER "${MOTOR_LIB_SOURCE_SET}" MOTOR_LIB_SOURCE_SET_SELECTED)

function(motor_configure_source_set target_name)
    set(_motor_source_set_target ${target_name})
    unset(_motor_source_set_target)
    message(STATUS "Motor library source set: ${MOTOR_LIB_SOURCE_SET_SELECTED}")
endfunction()

function(motor_filter_generated_lists source_list_var cxx_source_list_var)
    list(FILTER ${source_list_var} EXCLUDE REGEX "Templates/")
    list(FILTER ${cxx_source_list_var} EXCLUDE REGEX "Templates/")
    list(FILTER ${source_list_var} EXCLUDE REGEX "Lib/Motor/Profile/")
    list(FILTER ${cxx_source_list_var} EXCLUDE REGEX "Lib/Motor/Profile/")

    set(${source_list_var} ${${source_list_var}} PARENT_SCOPE)
    set(${cxx_source_list_var} ${${cxx_source_list_var}} PARENT_SCOPE)
endfunction()

function(motor_collect_platform_sources out_c_sources out_cxx_sources)
    file(GLOB_RECURSE _motor_platform_c_sources CONFIGURE_DEPENDS
        "BSP/*.c"
        "Lib/Sensor/*.c"
        "Lib/Communication/*.c"
        "Platform/*.c"
        "UserAPP/*.c"
    )

    file(GLOB_RECURSE _motor_platform_cxx_sources CONFIGURE_DEPENDS
        "BSP/*.cpp"
        "Lib/Sensor/*.cpp"
        "Lib/Communication/*.cpp"
        "Platform/*.cpp"
        "UserAPP/*.cpp"
    )

    list(FILTER _motor_platform_c_sources EXCLUDE REGEX "Templates/")
    list(FILTER _motor_platform_cxx_sources EXCLUDE REGEX "Templates/")

    set(${out_c_sources} ${_motor_platform_c_sources} PARENT_SCOPE)
    set(${out_cxx_sources} ${_motor_platform_cxx_sources} PARENT_SCOPE)
endfunction()

set(MOTOR_CORE_CXX_SOURCES
    Lib/Motor/Core/API/Motor_API.cpp
    Lib/Motor/Core/Feature/Motor_FeatureRegistry.cpp
    Lib/Motor/Core/RunPlan/Motor_RunPlan.cpp
    Lib/Motor/Core/Limit/Motor_TargetLimiter.cpp
    Lib/Motor/Core/Manager/Motor_Manager.cpp
    Lib/Motor/Core/Manager/Motor_ManagerAPI.cpp
    Lib/Motor/Core/Monitor/Motor_ManagerMonitor.cpp
    Lib/Motor/Core/Runtime/Motor_ManagerRuntime.cpp
    Lib/Motor/Core/Sampling/Motor_ManagerSampling.cpp
    Lib/Motor/Core/Startup/Motor_ManagerRLIdentify.cpp
    Lib/Motor/Core/Startup/Motor_ManagerStartup.cpp
    Lib/Motor/Core/Debug/Motor_ManagerDebug.cpp
    Lib/Motor/Core/Safety/Motor_ManagerSafety.cpp
    Lib/Motor/Core/Safety/Motor_CommandGuard.cpp
    Lib/Motor/Core/Safety/Motor_ConfigCheck.cpp
    Lib/Motor/Core/Safety/Motor_RuntimeMonitor.cpp
    Lib/Motor/Core/Safety/Motor_SignalHealth.cpp
    Lib/Motor/Core/FSM/Motor_FSM.cpp
    Lib/Motor/Core/FSM/AngleFSM/Motor_AngleFSM.cpp
    Lib/Motor/Core/FSM/CommandFSM/Motor_CommandFSM.cpp
    Lib/Motor/Core/FSM/EnergyFSM/Motor_EnergyFSM.cpp
    Lib/Motor/Core/FSM/LifecycleFSM/Motor_LifecycleFSM.cpp
    Lib/Motor/Core/FSM/RunPhaseFSM/Motor_RunPhaseFSM.cpp
    Lib/Motor/Core/FSM/SafetyFSM/Motor_SafetyFSM.cpp
    Lib/Motor/Core/FSM/StopFSM/Motor_StopFSM.cpp
    Lib/Motor/Core/FSM/StreamFSM/Motor_StreamFSM.cpp
)

function(motor_collect_full_sources out_c_sources out_cxx_sources)
    file(GLOB_RECURSE _motor_full_c_sources CONFIGURE_DEPENDS
        "BSP/*.c"
        "Lib/*.c"
        "Platform/*.c"
        "UserAPP/*.c"
    )

    file(GLOB_RECURSE _motor_full_cxx_sources CONFIGURE_DEPENDS
        "BSP/*.cpp"
        "Lib/*.cpp"
        "Platform/*.cpp"
        "UserAPP/*.cpp"
    )

    motor_filter_generated_lists(_motor_full_c_sources _motor_full_cxx_sources)

    set(${out_c_sources} ${_motor_full_c_sources} PARENT_SCOPE)
    set(${out_cxx_sources} ${_motor_full_cxx_sources} PARENT_SCOPE)
endfunction()

function(motor_collect_core_sources out_c_sources out_cxx_sources)
    motor_collect_platform_sources(
        _motor_platform_c_sources
        _motor_platform_cxx_sources
    )

    set(${out_c_sources}
        ${_motor_platform_c_sources}
        PARENT_SCOPE
    )
    set(${out_cxx_sources}
        ${_motor_platform_cxx_sources}
        ${MOTOR_CORE_CXX_SOURCES}
        PARENT_SCOPE
    )
endfunction()

function(motor_collect_application_sources out_c_sources out_cxx_sources)
    if(MOTOR_LIB_SOURCE_SET_SELECTED STREQUAL "FULL")
        motor_collect_full_sources(_motor_c_sources _motor_cxx_sources)
    elseif(MOTOR_LIB_SOURCE_SET_SELECTED STREQUAL "CORE")
        motor_collect_core_sources(_motor_c_sources _motor_cxx_sources)
    else()
        message(FATAL_ERROR "Unsupported MOTOR_LIB_SOURCE_SET=${MOTOR_LIB_SOURCE_SET}. Supported source sets: FULL, CORE")
    endif()

    set(${out_c_sources} ${_motor_c_sources} PARENT_SCOPE)
    set(${out_cxx_sources} ${_motor_cxx_sources} PARENT_SCOPE)
endfunction()
