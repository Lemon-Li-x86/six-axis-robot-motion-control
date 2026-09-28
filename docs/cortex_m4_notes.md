# Cortex-M4 Notes

-ARM：设计 CPU 架构/核心的一家公司和体系。

-Cortex-M：ARM 面向微控制器的一系列 CPU 核心。

-Cortex-M4：Cortex-M 家族里的一个型号，适合嵌入式控制，带较强的数字信号处理能力和可选 FPU。

-MCU：不只是 CPU。通常把 CPU、Flash、RAM、UART、GPIO、Timer 等全塞进一颗芯片。

-STM32F4：ST 做的一系列 MCU，很多型号内部用 Cortex-M4 核心。

-QEMU：在电脑上模拟 ARM/MCU 平台，让我们没有真实开发板也能先跑固件。

Cortex-M4 ≠ 一颗完整芯片

STM32F4

├── Cortex-M4 CPU core

├── Flash

├── SRAM

├── UART

├── GPIO

├── Timer

└── 其他外设
