;/********************************** (C) COPYRIGHT *******************************
;* File Name          : startup_CH564.S
;* Author             : WCH
;* Version            : V1.0.0
;* Date               : 2024/05/05
;* Description        : vector table for eclipse toolchain for CH564.
;*********************************************************************************
;* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
;* Attention: This software (modified or not) and binary are used for 
;* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

	.section  .init, "ax", @progbits
	.globl    _start
	.align    2

_start:
	j	handle_reset

	.section    .vector,"ax",@progbits
	.align  2
_vector_base:
	.option   norvc;
    .word   _start
    .word   0
    .word   NMI_Handler
    .word   HardFault_Handler
    .word   0x50000000
    .word   Ecall_M_Handler
    .word   0
    .word   0
    .word   Ecall_U_Handler
    .word   BreakPoint_Handler
    .word   0
    .word   0
    .word   SysTick_Handler
    .word   0
    .word   SW_Handler
    .word   0
    /*External Interrupts*/
    .word   I2C_EV_IRQHandler
    .word   I2C_ER_IRQHandler
    .word   ETH_IRQHandler
    .word   USBPD_IRQHandler
    .word   TIM0_IRQHandler
    .word   TIM1_IRQHandler
    .word   TIM2_IRQHandler
    .word   TIM3_IRQHandler
    .word   SPI0_IRQHandler
    .word   SPI1_IRQHandler
    .word   UART0_IRQHandler
    .word   UART1_IRQHandler
    .word   PA_IRQHandler
    .word   PB_IRQHandler
    .word   PD_IRQHandler
    .word   ADC_IRQHandler
    .word   SLV_IRQHandler
    .word   USBHS_HOST_IRQHandler
    .word   USBHS_DEV_IRQHandler
    .word   UART2_IRQHandler
    .word   UART3_IRQHandler
    .word   ETHWakeUp_IRQHandler
    .word   USBHSWakeUp_IRQHandler
    .word   USBPDWakeUp_IRQHandler

    .option rvc;
    .section    .text.vector_handler, "ax", @progbits
    .weak   NMI_Handler
    .weak   HardFault_Handler
    .weak   Ecall_M_Handler
    .weak   Ecall_U_Handler
    .weak   BreakPoint_Handler
    .weak   SysTick_Handler
    .weak   SW_Handler
    .weak   I2C_EV_IRQHandler
    .weak   I2C_ER_IRQHandler
    .weak   ETH_IRQHandler
    .weak   USBPD_IRQHandler
    .weak   TIM0_IRQHandler
    .weak   TIM1_IRQHandler
    .weak   TIM2_IRQHandler
    .weak   TIM3_IRQHandler
    .weak   SPI0_IRQHandler
    .weak   SPI1_IRQHandler
    .weak   UART0_IRQHandler
    .weak   UART1_IRQHandler
    .weak   PA_IRQHandler
    .weak   PB_IRQHandler
    .weak   PD_IRQHandler
    .weak   ADC_IRQHandler
    .weak   SLV_IRQHandler
    .weak   USBHS_HOST_IRQHandler
    .weak   USBHS_DEV_IRQHandler
    .weak   UART2_IRQHandler
    .weak   UART3_IRQHandler
    .weak   ETHWakeUp_IRQHandler
    .weak   USBHSWakeUp_IRQHandler
    .weak   USBPDWakeUp_IRQHandler

NMI_Handler:
HardFault_Handler:
Ecall_M_Handler:
Ecall_U_Handler:
BreakPoint_Handler:
SysTick_Handler:
SW_Handler:
I2C_EV_IRQHandler:
I2C_ER_IRQHandler:
ETH_IRQHandler:
USBPD_IRQHandler:
TIM0_IRQHandler:
TIM1_IRQHandler:
TIM2_IRQHandler:
TIM3_IRQHandler:
SPI0_IRQHandler:
SPI1_IRQHandler:
UART0_IRQHandler:
UART1_IRQHandler:
PA_IRQHandler:
PB_IRQHandler:
PD_IRQHandler:
ADC_IRQHandler:
SLV_IRQHandler:
USBHS_HOST_IRQHandler:
USBHS_DEV_IRQHandler:
UART2_IRQHandler:
UART3_IRQHandler:
ETHWakeUp_IRQHandler:
USBHSWakeUp_IRQHandler:
USBPDWakeUp_IRQHandler:
1:
	j 1b

	.section	.text.handle_reset,"ax",@progbits
	.weak	handle_reset
	.align	1
handle_reset:
.option push
.option	norelax
    csrw mepc, t0
	la gp, __global_pointer$
.option	pop
1:
	la sp, _eusrstack
2:
/* Load data section from flash to RAM */
    la a0, _data_lma
    la a1, _data_vma
    la a2, _edata
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b

2:
/* Clear bss section */
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 2f
1:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, 1b

2:
/* Configure pipelining and instruction prediction */
    li t0, 0x1f
    csrw 0xbc0, t0
/* Enable interrupt nesting and hardware stack */
	li t0, 0x1f
#   csrw 0x804, zero
/* Enable global interrupt and configure privileged mode */
   	li t0, 0x7800
   	csrw mstatus, t0
/* Configure the interrupt vector table recognition mode and entry address mode */
 	la t0, _vector_base
    ori t0, t0, 3
	csrw mtvec, t0
/*Enable the cache to cache the code from _cache_beg to _cache_end */
    /* PMP TOR(pmpaddr0 - pmpaddr1) */
    la t0, _cache_beg
    srli t0, t0, 2
    csrw pmpaddr0, t0

    la t0, _cache_end
    srli t0, t0, 2
    csrw pmpaddr1, t0

    li t0, 0x10
    csrw 0xbc3, t0

    li t0, 0xAD00
    csrw 0x3a0, t0

    /* Enable ICache */
    li t0, 0x4
    csrw 0xbd0, t0
    li t0, 0x03000002
    csrc 0xbc2, t0
    
/* Comfigure systemclock */
    jal  SystemInit
/* Jump main */
	la t0, main
	csrw mepc, t0
	mret
