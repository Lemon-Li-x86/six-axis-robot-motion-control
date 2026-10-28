# Stage5 / 5.3 FreeRTOS 逐任务 CPU 统计：首轮 QEMU 实测

## 被测版本、方法与原始证据

- 本轮实验代码源提交：`d348288abbe1916533585bfdd7f7f81c1a13dbde`（集成候选），GitHub Actions [#40737897100](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40737897100) 中 `stage5-rtos-task-cpu-trace` Job **success**。
- 工件：`stage5-freeRTOS-cpu-trace`，原始 `observed.log`（真实 QEMU GDB 输出）及 `observed.json`（独立 Python 校验与提取）；提交本仓库新分支后按相同参数再复验。
- 运行平台：QEMU MPS2-AN386 Cortex-M4；25 MHz Timer0；FreeRTOS 1 kHz Tick；`ROBOT_ENABLE_STAGE4_TASKS=ON`、`ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF`、`ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON`、`ROBOT_ENABLE_STAGE5_CPU_TRACE=ON`。
- 方法：FreeRTOS 任务切换回调在实际 Timer0 计数器上累积每任务时钟周期；每 1 ms Tick 对正在运行任务记账，避免 Timer0 在长时运行时回绕影响累计值。Monitor Task 每 100 ms 刷新 `g_stage5_cpu_trace_report`，通过 GDB 从固件内存读取。不引入另一套任务周期或时钟配置。
- 不启用时完全关闭任务切换 Trace 代码，固件正常构建与 Stage4 封版基线不受影响。

## 本次数据（观测窗 4.74801464 秒）

| FreeRTOS 任务 | 估计 CPU 运行占比 |
|---|---:|
| Control | 0.96% |
| Trajectory | 0.21% |
| Monitor | 0.06% |
| ProtocolRX | 0.01% |
| ProtocolTX | 0.03% |
| S4RTOSCheck（自测任务） | 0.14% |
| IDLE | 97.95% |

采样任务数 7，overflow_count=0。各任务占比未必恰好加至100%，因为任务切换/中断服务/采样开销等时间不完全属于某个任务。此结果是**短时 QEMU 自检任务负载**，不是用户交互式 PyBullet 联动负载，也不是实体 Cortex-M4 的 WCET 或任务利用率证明。

## 可复现构建

~~~bash
cmake -S . -B build/stage5-cpu-trace -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON -DROBOT_ENABLE_STAGE5_CPU_TRACE=ON
cmake --build build/stage5-cpu-trace --target freertos_demo
bash tools/ci_stage5_cpu_trace_qemu.sh build/stage5-cpu-trace/firmware/freertos_demo/freertos_demo.elf build/stage5-cpu-trace/observed
~~~

脚本通过 GDB 停在既有 Stage4 自测结束点，要求任务名、Timer0 频率、任务时间与占比自洽，拒绝容量溢出，并输出 JSON。单独 CI 验证该任务而不依赖 GUI、PyBullet、Windows。

## 未满足的最终性能验收

- 长时及高负载 30 分钟运行统计、拥塞/反馈缺失时负载及抖动；
- 相同工具链、任务/运行配置下的优化前后 CPU、FLASH/RAM、控制 P99/最大抖动比较；
- 中断优先级和轨迹调度调整的受控 A/B 测试。

因此这一页是带可验证结果的**性能分析基线**，不是全项性能优化已完成的声明。
