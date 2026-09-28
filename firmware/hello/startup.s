.syntax unified
.cpu cortex-m4
.thumb

.global Reset_Handler
.global _estack

.section .isr_vector, "a", %progbits
.word _estack
.word Reset_Handler

.section .text.Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    bl main

LoopForever:
    b LoopForever
