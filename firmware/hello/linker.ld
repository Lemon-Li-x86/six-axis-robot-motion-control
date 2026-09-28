/* 指定程序入口函数 */
ENTRY(Reset_Handler)

/* 定义目标系统中的内存区域 */
MEMORY
{
    /* FLASH：
     * 用来放程序代码和只读数据。
     * rx = readable + executable
     */
    FLASH (rx) :
        ORIGIN = 0x00000000,
        LENGTH = 4M

    /* RAM：
     * 用来放运行时数据。
     * rwx = readable + writable + executable
     */
    RAM (rwx) :
        ORIGIN = 0x20000000,
        LENGTH = 4M
}

/* 设置初始栈顶地址。
 * 栈从 RAM 顶部开始向下增长。
 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 指定不同代码/数据段应该放到哪里 */
SECTIONS
{
    /* 中断向量表放到 FLASH 最前面 */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    /* 程序代码和只读数据放到 FLASH */
    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH

    /* 已初始化的全局/静态变量放到 RAM */
    .data :
    {
        *(.data*)
    } > RAM

    /* 未初始化的全局/静态变量放到 RAM */
    .bss :
    {
        *(.bss*)
        *(COMMON)
    } > RAM
}