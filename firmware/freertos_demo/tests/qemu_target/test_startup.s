
/*
 * 文件：test_startup.s
 *
 * Cortex-M4 独立测试固件的启动入口。
 *
 * 当前用于不涉及硬件中断的测试：
 * Protocol、Ring Buffer。
 *
 * 后续 UART / Timer IRQ 测试应使用
 * 正式固件的实际中断向量和处理函数，
 * 不使用本精简启动文件代替。
 */

.syntax unified
.cpu cortex-m4
.thumb

.extern _estack
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss
.extern main

/* =========================================================
 * Interrupt Vector Table
 * ========================================================= */

.section .isr_vector, "a", %progbits
.align 2

.word _estack
.word Reset_Handler

/* Cortex-M 系统异常向量 2~15。 */
.rept 14
.word Default_Handler
.endr


/* =========================================================
 * Reset Handler
 * ========================================================= */

.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function
.thumb_func

Reset_Handler:

    /* 将 .data 初始值从 Flash 复制到 RAM。 */
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata

CopyData:
    cmp r1, r2
    bcs ZeroBss
    ldr r3, [r0], #4
    str r3, [r1], #4
    b CopyData

ZeroBss:
    /* 清零未初始化的全局、静态变量。 */
    ldr r1, =_sbss
    ldr r2, =_ebss
    movs r3, #0

ZeroBssLoop:
    cmp r1, r2
    bcs StartMain
    str r3, [r1], #4
    b ZeroBssLoop

StartMain:
    bl main

LoopForever:
    b LoopForever


/* =========================================================
 * Default Exception Handler
 * ========================================================= */

.section .text.Default_Handler, "ax", %progbits
.global Default_Handler
.type Default_Handler, %function
.thumb_func

Default_Handler:
    b Default_Handler
