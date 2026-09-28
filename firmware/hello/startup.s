.syntax unified

/* 指定目标 CPU 为 Cortex-M4 */
.cpu cortex-m4

/* Cortex-M 系列只运行 Thumb 指令 */
.thumb

/* 把这两个符号暴露给链接器 */
.global Reset_Handler
.global _estack

/* 中断向量表 */
.section .isr_vector, "a", %progbits

/* 按 4 字节对齐 */
.align 2

/* 向量表第一个值：
 * CPU 复位后使用的初始栈顶地址
 */
.word _estack

/* 向量表第二个值：
 * CPU 复位后首先执行的函数地址
 */
.word Reset_Handler

/* Reset_Handler 代码区域 */
.section .text.Reset_Handler

/* 告诉汇编器 Reset_Handler 是一个函数 */
.type Reset_Handler, %function

/* 明确说明这是 Thumb 函数 */
.thumb_func

Reset_Handler:
    /* 跳转到 C 语言的 main() 函数 */
    bl main

LoopForever:
    /* 如果 main() 意外返回，就停在这里无限循环 */
    b LoopForever