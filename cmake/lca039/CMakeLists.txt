cmake_minimum_required(VERSION 3.22)

set(LCA039_CMSIS_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
set(LCA039_DEVICE_DIR ${LCA039_CMSIS_DIR}/Device/LCMicro/LCM32F039)
set(LCA039_DRIVER_DIR ${CMAKE_SOURCE_DIR}/Drivers/LCM32F039_StdPeriph_Driver)

set(LCA039_DRIVER_SOURCES
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_acmp.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_adc.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_crc.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_dac.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_dbgmcu.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_div.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_dma.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_eeprom.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_exti.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_flash.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_gpio.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_i2c.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_I2cEmulation.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_iwdg.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_lcd.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_opa.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_pwr.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_rcc.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_ssp.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_tim.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_uart.c
    ${LCA039_DRIVER_DIR}/Src/lcm32f039_wt.c
)

add_library(lcm32f039_cmsis INTERFACE)
target_include_directories(lcm32f039_cmsis SYSTEM INTERFACE
    ${LCA039_CMSIS_DIR}/Include
    ${LCA039_DEVICE_DIR}/Include
    ${LCA039_DRIVER_DIR}/Inc
)
target_compile_definitions(lcm32f039_cmsis INTERFACE
    LCM32F039
    _LCM32_GBL_HSE_nHZ=${LCA039_OSCH_HZ}UL
    $<$<CONFIG:Debug>:DEBUG>
)

add_library(lcm32f039_stdperiph OBJECT)
target_sources(lcm32f039_stdperiph PRIVATE ${LCA039_DRIVER_SOURCES})
target_link_libraries(lcm32f039_stdperiph PUBLIC lcm32f039_cmsis)
