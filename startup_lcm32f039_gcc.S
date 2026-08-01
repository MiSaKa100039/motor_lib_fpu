/*
 * Project-owned GNU assembler port for LCM32F039.
 * The vector order and handler names are derived from the vendor ARMASM
 * template in Drivers/CMSIS/Device/LCMicro/LCM32F039/Source/Templates/arm.
 */

.syntax unified
.cpu cortex-m0
.thumb

.extern SystemInit
.extern __libc_init_array
.extern main

.section .isr_vector,"a",%progbits
.align 2
.global g_pfnVectors
.global __Vectors
.global __Vectors_End
.global __Vectors_Size
.type g_pfnVectors, %object

g_pfnVectors:
__Vectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word 0
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

    .word WWDG_Handler
    .word EXTI16_LVD_Handler
    .word WT_Handler
    .word EFLS_Handler
    .word CHIPCTRL_Handler
    .word EXTI0_1_Handler
    .word EXTI2_3_Handler
    .word EXTI4_15_Handler
    .word LCD_ACMP0_2_Handler
    .word DMAC_CH0_Handler
    .word DMAC_CH1_2_Handler
    .word DMAC_CH3_Handler
    .word ADC_Handler
    .word TIM1_NON_CC_Handler
    .word TIM1_CC_Handler
    .word TIM2_Handler
    .word TIM3_Handler
    .word DAC0_1_Handler
    .word TIM7_DAC3_5_PLC_Handler
    .word TIM14_Handler
    .word TIM15_Handler
    .word TIM16_Handler
    .word TIM17_Handler
    .word I2C0_Handler
    .word I2C1RVE_Handler
    .word SSP0_Handler
    .word SSP2_Handler
    .word UART0_Handler
    .word UART1_Handler
    .word UART3_Handler
    .word CAN_Handler
    .word DIV_Handler

__Vectors_End:
.set __Vectors_Size, __Vectors_End - __Vectors
.size g_pfnVectors, . - g_pfnVectors

.section .text.Reset_Handler,"ax",%progbits
.align 2
.weak Reset_Handler
.type Reset_Handler, %function
.thumb_func
Reset_Handler:
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata

1:
    cmp r1, r2
    bcc 2f
    b 3f
2:
    ldr r3, [r0]
    str r3, [r1]
    adds r0, r0, #4
    adds r1, r1, #4
    b 1b

3:
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0

4:
    cmp r0, r1
    bcc 5f
    b 6f
5:
    str r2, [r0]
    adds r0, r0, #4
    b 4b

6:
    bl SystemInit
    bl __libc_init_array
    bl main

7:
    b 7b

.size Reset_Handler, . - Reset_Handler

.section .text.Default_Handler,"ax",%progbits
.align 2
.type Default_Handler, %function
.thumb_func
Default_Handler:
    b .
.size Default_Handler, . - Default_Handler

.macro WEAK_DEFAULT_HANDLER handler
    .weak \handler
    .thumb_set \handler, Default_Handler
.endm

WEAK_DEFAULT_HANDLER NMI_Handler
WEAK_DEFAULT_HANDLER HardFault_Handler
WEAK_DEFAULT_HANDLER SVC_Handler
WEAK_DEFAULT_HANDLER PendSV_Handler
WEAK_DEFAULT_HANDLER SysTick_Handler
WEAK_DEFAULT_HANDLER WWDG_Handler
WEAK_DEFAULT_HANDLER EXTI16_LVD_Handler
WEAK_DEFAULT_HANDLER WT_Handler
WEAK_DEFAULT_HANDLER EFLS_Handler
WEAK_DEFAULT_HANDLER CHIPCTRL_Handler
WEAK_DEFAULT_HANDLER EXTI0_1_Handler
WEAK_DEFAULT_HANDLER EXTI2_3_Handler
WEAK_DEFAULT_HANDLER EXTI4_15_Handler
WEAK_DEFAULT_HANDLER LCD_ACMP0_2_Handler
WEAK_DEFAULT_HANDLER DMAC_CH0_Handler
WEAK_DEFAULT_HANDLER DMAC_CH1_2_Handler
WEAK_DEFAULT_HANDLER DMAC_CH3_Handler
WEAK_DEFAULT_HANDLER ADC_Handler
WEAK_DEFAULT_HANDLER TIM1_NON_CC_Handler
WEAK_DEFAULT_HANDLER TIM1_CC_Handler
WEAK_DEFAULT_HANDLER TIM2_Handler
WEAK_DEFAULT_HANDLER TIM3_Handler
WEAK_DEFAULT_HANDLER DAC0_1_Handler
WEAK_DEFAULT_HANDLER TIM7_DAC3_5_PLC_Handler
WEAK_DEFAULT_HANDLER TIM14_Handler
WEAK_DEFAULT_HANDLER TIM15_Handler
WEAK_DEFAULT_HANDLER TIM16_Handler
WEAK_DEFAULT_HANDLER TIM17_Handler
WEAK_DEFAULT_HANDLER I2C0_Handler
WEAK_DEFAULT_HANDLER I2C1RVE_Handler
WEAK_DEFAULT_HANDLER SSP0_Handler
WEAK_DEFAULT_HANDLER SSP2_Handler
WEAK_DEFAULT_HANDLER UART0_Handler
WEAK_DEFAULT_HANDLER UART1_Handler
WEAK_DEFAULT_HANDLER UART3_Handler
WEAK_DEFAULT_HANDLER CAN_Handler
WEAK_DEFAULT_HANDLER DIV_Handler

.section .note.GNU-stack,"",%progbits
