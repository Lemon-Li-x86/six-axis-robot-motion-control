/* Cortex-M4 程序从 Reset_Handler 开始执行 */
ENTRY(Reset_Handler)

/* QEMU MPS2-AN386 的基础内存布局 */
MEMORY
{
    /* 程序代码、常量等放在 FLASH 区域 */
    FLASH (rx) :
        ORIGIN = 0x00000000,
        LENGTH = 4M

    /* 运行时变量、堆、栈等放在 RAM */
    RAM (rwx) :
        ORIGIN = 0x20000000,
        LENGTH = 4M
}

/* Cortex-M 启动时使用的初始栈顶地址 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* 中断向量表必须位于程序最前面 */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    /* 程序代码和只读常量 */
    .text :
    {
        *(.text*)
        *(.rodata*)

        /* 保持 4 字节对齐 */
        . = ALIGN(4);
    } > FLASH

    /*
     * .data 中是“有初始值的全局/静态变量”。
     *
     * 初始值存放在 FLASH 中，
     * 程序运行时需要复制到 RAM。
     *
     * _sidata：FLASH 中 .data 初始值的位置
     * _sdata ：RAM 中 .data 开始位置
     * _edata ：RAM 中 .data 结束位置
     */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;

        *(.data*)

        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /*
     * .bss 中是“没有显式初始值的全局/静态变量”。
     *
     * 按 C 语言规则，它们在程序开始前必须全部清零。
     */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;

        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;
    } > RAM
}