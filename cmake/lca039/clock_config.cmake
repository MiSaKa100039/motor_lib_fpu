include_guard(GLOBAL)

set(LCA039_CLOCK_SOURCE "RCH" CACHE STRING "System clock source: RCH or OSCH")
set_property(CACHE LCA039_CLOCK_SOURCE PROPERTY STRINGS RCH OSCH)
set(LCA039_SYSCLK_HZ "96000000" CACHE STRING "Requested system clock in Hz")
set(LCA039_OSCH_HZ "16000000" CACHE STRING "External high-speed oscillator frequency in Hz")
set(LCA039_APB0_DIV "1" CACHE STRING "APB0 divider from 1 to 16")
set(LCA039_APB1_DIV "1" CACHE STRING "APB1 divider from 1 to 16")
set(LCA039_VDD_MV "5000" CACHE STRING "Target supply voltage in millivolts")

string(TOUPPER "${LCA039_CLOCK_SOURCE}" LCA039_CLOCK_SOURCE_NORMALIZED)

foreach(variable IN ITEMS LCA039_SYSCLK_HZ LCA039_OSCH_HZ LCA039_APB0_DIV LCA039_APB1_DIV LCA039_VDD_MV)
    if(NOT "${${variable}}" MATCHES "^[0-9]+$")
        message(FATAL_ERROR "${variable} must be a positive integer, got '${${variable}}'")
    endif()
endforeach()

if(LCA039_CLOCK_SOURCE_NORMALIZED STREQUAL "RCH")
    set(LCA039_CLOCK_SOURCE_ID 0)
    set(LCA039_INPUT_CLOCK_HZ 16000000)
elseif(LCA039_CLOCK_SOURCE_NORMALIZED STREQUAL "OSCH")
    set(LCA039_CLOCK_SOURCE_ID 1)
    set(LCA039_INPUT_CLOCK_HZ ${LCA039_OSCH_HZ})
    if(LCA039_OSCH_HZ LESS 4000000 OR LCA039_OSCH_HZ GREATER 16000000)
        message(FATAL_ERROR "LCA039_OSCH_HZ must be between 4000000 and 16000000 Hz")
    endif()
else()
    message(FATAL_ERROR "LCA039_CLOCK_SOURCE must be RCH or OSCH, got '${LCA039_CLOCK_SOURCE}'")
endif()

if(LCA039_SYSCLK_HZ LESS 1 OR LCA039_SYSCLK_HZ GREATER 96000000)
    message(FATAL_ERROR "LCA039_SYSCLK_HZ must be between 1 and 96000000 Hz")
endif()

foreach(variable IN ITEMS LCA039_APB0_DIV LCA039_APB1_DIV)
    if(${variable} LESS 1 OR ${variable} GREATER 16)
        message(FATAL_ERROR "${variable} must be between 1 and 16")
    endif()
endforeach()

if(LCA039_SYSCLK_HZ EQUAL 72000000 AND LCA039_VDD_MV LESS_EQUAL 2800)
    message(FATAL_ERROR "72 MHz operation requires LCA039_VDD_MV greater than 2800")
endif()

if(LCA039_SYSCLK_HZ GREATER 72000000 AND LCA039_VDD_MV LESS 4500)
    message(FATAL_ERROR "Above 72 MHz operation requires LCA039_VDD_MV at least 4500 for the 4.0 V LVR level")
endif()

if(LCA039_SYSCLK_HZ GREATER_EQUAL 64000000 AND LCA039_VDD_MV LESS 2500)
    message(FATAL_ERROR "64 MHz and above require at least 2500 mV for the configured LVR level")
endif()

set(LCA039_USE_PLL 0)
set(LCA039_PLL_DM 1)
set(LCA039_PLL_DN 1)
set(LCA039_PLL_OD 1)
set(LCA039_PLL_VCO_HZ ${LCA039_INPUT_CLOCK_HZ})
set(LCA039_PLL_CFG_VALUE 0x0000115E)

if(NOT LCA039_SYSCLK_HZ EQUAL LCA039_INPUT_CLOCK_HZ)
    set(LCA039_USE_PLL 1)
    set(_lca039_pll_found FALSE)
    set(_lca039_nearest_lower 0)
    set(_lca039_nearest_any 0)
    set(_lca039_nearest_delta 2147483647)

    foreach(_od IN ITEMS 2 1)
        foreach(_dm RANGE 1 4)
            foreach(_dn RANGE 1 32)
                math(EXPR _numerator "${LCA039_INPUT_CLOCK_HZ} * ${_dn}")
                math(EXPR _denominator "${_dm} * ${_od} * 2")
                math(EXPR _remainder "${_numerator} % ${_denominator}")
                math(EXPR _vco_denominator "${_dm} * 2")
                math(EXPR _vco_remainder "${_numerator} % ${_vco_denominator}")

                if(_remainder EQUAL 0 AND _vco_remainder EQUAL 0)
                    math(EXPR _output "${_numerator} / ${_denominator}")
                    math(EXPR _vco "${_numerator} / ${_vco_denominator}")

                    if(_vco GREATER_EQUAL 30000000 AND _vco LESS_EQUAL 192000000 AND _output LESS_EQUAL 96000000)
                        if(_output LESS_EQUAL LCA039_SYSCLK_HZ AND _output GREATER _lca039_nearest_lower)
                            set(_lca039_nearest_lower ${_output})
                        endif()

                        math(EXPR _delta "${_output} - ${LCA039_SYSCLK_HZ}")
                        if(_delta LESS 0)
                            math(EXPR _delta "0 - ${_delta}")
                        endif()
                        if(_delta LESS _lca039_nearest_delta)
                            set(_lca039_nearest_delta ${_delta})
                            set(_lca039_nearest_any ${_output})
                        endif()

                        if(NOT _lca039_pll_found AND _output EQUAL LCA039_SYSCLK_HZ)
                            set(LCA039_PLL_DM ${_dm})
                            set(LCA039_PLL_DN ${_dn})
                            set(LCA039_PLL_OD ${_od})
                            set(LCA039_PLL_VCO_HZ ${_vco})
                            set(_lca039_pll_found TRUE)
                        endif()
                    endif()
                endif()
            endforeach()
        endforeach()
    endforeach()

    if(NOT _lca039_pll_found)
        if(_lca039_nearest_lower GREATER 0)
            set(_lca039_hint "nearest legal frequency not above the request is ${_lca039_nearest_lower} Hz")
        else()
            set(_lca039_hint "nearest legal frequency is ${_lca039_nearest_any} Hz")
        endif()
        message(FATAL_ERROR
            "No exact PLL solution for ${LCA039_INPUT_CLOCK_HZ} Hz -> ${LCA039_SYSCLK_HZ} Hz; ${_lca039_hint}")
    endif()

    math(EXPR _pll_dm_field "${LCA039_PLL_DM} - 1")
    math(EXPR _pll_dn_field "${LCA039_PLL_DN} - 1")
    math(EXPR _pll_od_field "${LCA039_PLL_OD} - 1")
    math(EXPR LCA039_PLL_CFG_VALUE
        "(${_pll_od_field} << 12) | (${_pll_dm_field} << 8) | (${_pll_dn_field} << 2) | 2 | ${LCA039_CLOCK_SOURCE_ID}"
        OUTPUT_FORMAT HEXADECIMAL)
    string(TOUPPER "${LCA039_PLL_CFG_VALUE}" LCA039_PLL_CFG_VALUE)
endif()

if(LCA039_SYSCLK_HZ LESS 32000000)
    set(LCA039_FLASH_WAIT_STATES 0)
elseif(LCA039_SYSCLK_HZ LESS 64000000)
    set(LCA039_FLASH_WAIT_STATES 1)
else()
    set(LCA039_FLASH_WAIT_STATES 2)
endif()

if(LCA039_SYSCLK_HZ GREATER_EQUAL 64000000)
    set(LCA039_HIGH_SPEED_POWER_REQUIRED 1)
else()
    set(LCA039_HIGH_SPEED_POWER_REQUIRED 0)
endif()

math(EXPR LCA039_PCLK0_HZ "${LCA039_SYSCLK_HZ} / ${LCA039_APB0_DIV}")
math(EXPR LCA039_PCLK1_HZ "${LCA039_SYSCLK_HZ} / ${LCA039_APB1_DIV}")

set(LCA039_ADC_SYNC_DIV_MIN 16)
foreach(_adc_div IN ITEMS 1 2 4 6 8 10 12 16)
    math(EXPR _adc_limit "24000000 * ${_adc_div}")
    if(LCA039_PCLK1_HZ LESS_EQUAL _adc_limit)
        set(LCA039_ADC_SYNC_DIV_MIN ${_adc_div})
        break()
    endif()
endforeach()

set(LCA039_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${LCA039_GENERATED_INCLUDE_DIR}")
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/lca039_clock_config.h.in"
    "${LCA039_GENERATED_INCLUDE_DIR}/lca039_clock_config.h"
    @ONLY
)

message(STATUS "LCA039 clock source: ${LCA039_CLOCK_SOURCE_NORMALIZED} at ${LCA039_INPUT_CLOCK_HZ} Hz")
if(LCA039_USE_PLL)
    message(STATUS
        "LCA039 PLL: DM=${LCA039_PLL_DM}, DN=${LCA039_PLL_DN}, OD=${LCA039_PLL_OD}, "
        "VCO=${LCA039_PLL_VCO_HZ} Hz, CFG=${LCA039_PLL_CFG_VALUE}")
else()
    message(STATUS "LCA039 PLL: bypassed")
endif()
message(STATUS
    "LCA039 clocks: SYSCLK/HCLK=${LCA039_SYSCLK_HZ} Hz, PCLK0=${LCA039_PCLK0_HZ} Hz, "
    "PCLK1=${LCA039_PCLK1_HZ} Hz")
message(STATUS
    "LCA039 Flash wait states: ${LCA039_FLASH_WAIT_STATES}; "
    "minimum synchronous ADC divider: ${LCA039_ADC_SYNC_DIV_MIN}")
