.syntax unified

/* 目标处理器 */
.cpu cortex-m4

/* Cortex-M 使用 Thumb 指令集 */
.thumb


/* ---------------------------------------------------------
 * 链接脚本提供的内存符号
 * --------------------------------------------------------- */
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss
.extern _estack

/* C 程序入口 */
.extern main

/* UART0 中断处理函数 */
.extern UART0_RX_IRQHandler
.extern UART0_TX_IRQHandler

/* Timer1 周期中断处理函数 */
.extern TIMER1_IRQHandler

/* ---------------------------------------------------------
 * FreeRTOS Cortex-M 移植层提供的异常处理函数
 * --------------------------------------------------------- */
.extern vPortSVCHandler
.extern xPortPendSVHandler
.extern xPortSysTickHandler


.global Reset_Handler


/* =========================================================
 * Cortex-M 中断向量表
 * ========================================================= */
.section .isr_vector, "a", %progbits
.align 2

.word _estack                    /* 0  Initial Stack Pointer */
.word Reset_Handler              /* 1  Reset */
.word Default_Handler            /* 2  NMI */
.word Default_Handler            /* 3  HardFault */
.word Default_Handler            /* 4  MemManage */
.word Default_Handler            /* 5  BusFault */
.word Default_Handler            /* 6  UsageFault */

.word 0                          /* 7  Reserved */
.word 0                          /* 8  Reserved */
.word 0                          /* 9  Reserved */
.word 0                          /* 10 Reserved */

/* FreeRTOS：启动第一个任务 */
.word vPortSVCHandler            /* 11 SVCall */

.word Default_Handler            /* 12 Debug Monitor */
.word 0                          /* 13 Reserved */

/* FreeRTOS：任务切换 */
.word xPortPendSVHandler         /* 14 PendSV */

/* FreeRTOS：系统 Tick */
.word xPortSysTickHandler        /* 15 SysTick */


/* =========================================================
 * Cortex-M4 External IRQ
 *
 * Vector 16 开始对应 External IRQ 0。
 * ========================================================= */

/* IRQ 0：UART0 RX */
.word UART0_RX_IRQHandler

/* IRQ 1：UART0 TX */
.word UART0_TX_IRQHandler

/* IRQ 2：UART1 RX */
.word Default_Handler

/* IRQ 3：UART1 TX */
.word Default_Handler

/* IRQ 4：UART2 RX */
.word Default_Handler

/* IRQ 5：UART2 TX */
.word Default_Handler

/* IRQ 6 */
.word Default_Handler

/* IRQ 7 */
.word Default_Handler

/* IRQ 8：Timer0
 *
 * 当前 Timer0 只用于自由运行性能计时，
 * 不开启中断。
 */
.word Default_Handler

/* IRQ 9：Timer1 Periodic Interrupt */
.word TIMER1_IRQHandler

/* IRQ 10：Dual Timer */
.word Default_Handler

/* IRQ 11 */
.word Default_Handler

/* IRQ 12 */
.word Default_Handler

/* IRQ 13 */
.word Default_Handler

/* IRQ 14 */
.word Default_Handler

/* IRQ 15 */
.word Default_Handler

/* IRQ 16 */
.word Default_Handler

/* IRQ 17 */
.word Default_Handler

/* IRQ 18 */
.word Default_Handler

/* IRQ 19 */
.word Default_Handler

/* IRQ 20 */
.word Default_Handler

/* IRQ 21 */
.word Default_Handler

/* IRQ 22 */
.word Default_Handler

/* IRQ 23 */
.word Default_Handler

/* IRQ 24 */
.word Default_Handler

/* IRQ 25 */
.word Default_Handler

/* IRQ 26 */
.word Default_Handler

/* IRQ 27 */
.word Default_Handler

/* IRQ 28 */
.word Default_Handler

/* IRQ 29 */
.word Default_Handler

/* IRQ 30 */
.word Default_Handler

/* IRQ 31 */
.word Default_Handler


/* =========================================================
 * Reset Handler
 * ========================================================= */
.section .text.Reset_Handler
.type Reset_Handler, %function
.thumb_func

Reset_Handler:

    /* -----------------------------------------------------
     * 1. 把 .data 的初始值从 FLASH 复制到 RAM
     * ----------------------------------------------------- */

    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata

CopyDataLoop:

    cmp r1, r2
    bcs ZeroBss

    ldr r3, [r0], #4
    str r3, [r1], #4

    b CopyDataLoop


    /* -----------------------------------------------------
     * 2. 把 .bss 区域全部清零
     * ----------------------------------------------------- */

ZeroBss:

    ldr r1, =_sbss
    ldr r2, =_ebss

    movs r3, #0

ZeroBssLoop:

    cmp r1, r2
    bcs CallMain

    str r3, [r1], #4

    b ZeroBssLoop


    /* -----------------------------------------------------
     * 3. C 运行环境准备完成，进入 main()
     * ----------------------------------------------------- */

CallMain:

    bl main


    /* main() 理论上不应该返回 */
LoopForever:

    b LoopForever


/* =========================================================
 * 未实现异常的默认处理
 * ========================================================= */
.section .text.Default_Handler
.type Default_Handler, %function
.thumb_func

Default_Handler:

    /*
     * 出问题时停在这里，
     * 可以使用 GDB 检查当前 Vector / PC。
     */
    b Default_Handler