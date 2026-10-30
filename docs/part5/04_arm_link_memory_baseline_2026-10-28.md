# Stage5 5.3 · ARM/QEMU 链接内存基线（2026-10-28）

## 证据与构建条件

- 源提交：`abbc6caaabb8405548477ef0262dba25686ff62c`，分支 `feat/stage5-memory-audit-20261028`。
- 自动构建：[GitHub Actions #40700431434](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40700431434)，任务 `link-map-parser` 与 `arm-link-map` 均成功。
- 原始工件：`stage5-arm-memory-map-evidence`，包含 ARM ELF、GNU ld map 和 JSON 解析报告。
- 构建：`cmake -S . -B build/stage5-arm -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DROBOT_ENABLE_STAGE5_LINK_MAP=ON -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF`，目标 `freertos_demo`。其他选项均为该分支默认值，包括 **Stage4 motion tasks 默认 OFF**。
- 虚拟平台：QEMU MPS2-AN386 相关 linker.ld，FLASH 4 MiB、RAM 4 MiB。容量不是实体 STM32 上的可用空间。

## 链接期实测

| 分配项 | 大小（bytes） | 说明 |
|---|---:|---|
| FLASH 已分配 | **85,424** | 输出段 FLASH VMA 85,416，加 RAM 初始化数据镜像 8 |
| RAM 已分配 | **33,512** | .data 8，.bss 33,504 |
| .isr_vector | 192 | FLASH |
| .ARM.exidx | 8 | FLASH |
| .text | 85,216 | FLASH |
| .data | 8 | RAM，FLASH 镜像另占 8 |
| .bss | 33,504 | RAM，ELF `NOBITS`，**不计入 FLASH 镜像** |

复核 ELF 节类型得到 `.data PROGBITS`、`.bss NOBITS`。GNU ld map 给 `.bss` 显示的 load address 不意味着它真的占用 FLASH，解析器已经按 `NOBITS` 事实处理。

## 解释与限制

这是真实交叉编译器链接出的 **固件镜像静态尺寸**，不是任务 CPU 占用、FreeRTOS 运行中剩余堆栈、运行时峰值或物理机器人实测。此时并未开启 Stage5 实际 UART / PID / Trajectory 任务链，也不能将数字称作整合版最终内存结果。

此条只有 **before 基线**：未完成相同编译参数下 before/after 实验，不能宣称内存优化降低了多少。后续启用 Stage5 完整任务时必须另建同配置基线，并同时报告任务占用、控制周期和抖动。
