
0x40004000是我们当前选的 QEMU mps2-an386 平台里的 UART 外设地址之一。

startup.s
负责“从复位跳到 main”

main.c
负责真正程序逻辑

linker.ld
负责“代码和数据放到哪里”
