# Stage5 5.3 系统性能优化：重复测量方法与签收边界（2026-11-01）

> **范围**：对照原始实习大纲第5阶段 5.3「通过FreeRTOS任务状态统计功能分析任务CPU占用率、优化高耗时算法；分析 .map 优化内存占用、裁剪冗余代码和全局变量；优化中断优先级和任务调度逻辑，降低控制周期抖动、提升实时性」。仅报告有源代码和实测依据的结论，**不得因测试工作流成功而编造实时性改善**。本工程是 ARM Cortex-M4/QEMU/FreeRTOS 与 PyBullet 的软件仿真，没有实体硬件 WCET 认证。

## 1. 第五阶段 5.3 对照表

| 原始要求 | 实际实施/证据 | 2026-11-01 结论 |
|---|---|---|
| CPU任务统计、定位高耗时部分 | Timer0 + FreeRTOS context-switch/tick 跟踪；GDB 读取每任务 CPU；Monitor 堆栈高水位扫描由每100ms改为JSON配置每1000ms，Monitor 仍每100ms正常更新心跳 | 已实现；之前短时窗口 Monitor CPU 比例 0.07%→0.04%，仅是指定自测负载 |
| .map 内存和代码裁剪 | 对完整 UART/RTOS ARM 固件同编译配置开关 `ROBOT_ENABLE_STAGE5_LINK_GC`，函数/数据分段与 `--gc-sections` | FLASH 105256→87384 B（减少17872B，约17.0%），静态 RAM 33796→33796 B（不变）；不等于运行堆峰值 |
| 优先级与调度 | 固件 Control=4 / ProtocolRX=3 / Trajectory=2 / Monitor=1；ARM QEMU NVIC UART/Timer ISR 按 FreeRTOS FromISR 阈值约束审查；`motion_task_wait_next_period` 对超期激活丢弃追赶 | 有代码级调度策略与优先级合规性审查，不能虚构 IRQ 改数值产生的独立收益 |
| 降低控制周期 jitter | 早期 QEMU 单次短窗 A/B 结果方向不一致；新建同编译配置交错重复实验 | **尚未可无保留证明。** 看多轮原始数据，不以中位数单项改善冒充稳定降低 |

### 不混淆三类性能证据

1. **本机真实 PyBullet/UART 动态闭环**：验证实际执行与稳定到位，不直接产出可比较的 ARM 单任务 CPU P99。
2. **QEMU FreeRTOS GDB/Timer0 自测**：验证 CPU/任务定时/栈；使用**固定输入、静止反馈**的 Stage4 RTOS self-test，不能外推满载运动或实物。
3. **同编译配置 .map A/B**：静态 ROM/RAM 资源，不能从 linker map 推导动态堆峰值，也不能从 FLASH 减少宣称 RAM 减少。

## 2. 这轮针对原测量方法的修复

原工作流 `.github/workflows/stage5-cpu-trace.yml` 原先使用 `ROBOT_STAGE4_QEMU_HEALTHY_RUN_MS=1200` 默认值，`stage4_timing_capture` 在健康阶段末冻结 Timer0 统计，但 Monitor 每100ms刷新 CPU trace，最终 GDB 却在额外的反馈过期、故障处理及多项式测试后才读取最后一次 CPU 快照。由此，**CPU 占比与 Timer0 抖动并不来自同一结束时刻/同一工作阶段**。早期约4.7秒 CPU 与约1.2秒健康阶段 Timer0 抖动因此不能写成同窗统计。

当前改法：
- `stage4_rtos_selftest.c` 在健康阶段边界 `stage4_timing_capture()` 后**立即调用 `stage5_cpu_trace_snapshot()`**，仅测试编译宏开启时执行；不混入后续故障注入。
- `monitor_task.c` 在该 QEMU 自测模式不再每100ms执行开销较大的 CPU report 快照；平常 `ROBOT_ENABLE_STAGE5_CPU_TRACE` 诊断方式不受影响。
- 两套 QEMU A/B 二进制显式使用 **`-DROBOT_STAGE4_QEMU_HEALTHY_RUN_MS=5000`**；只在自测构建生效，不修改生产周期、PID、速度/加速度/安全限位。
- `stage4_timing_probe.c` 仍仅保留**最后128个间隔**，因此 5 秒 CPU 数据与 Timer0 虽同一**健康阶段结束边界**，但并不是完全相同的**时间跨度**。Control 最后最多约1.28秒、Trajectory 最后最多约2.56秒、Monitor 最后最多约50个周期。**不能称它们是同样长的全5秒窗口**。
- `tools/run_stage5_perf_repeats.sh` 串行进行 6 对交错 A/B（奇数对 baseline→optimized；偶数对 optimized→baseline），每一对使用**相同 commit、工具链、RTOS 自测和其他 CMake 配置**，仅切换 `ROBOT_ENABLE_STAGE5_MONITOR_STACK_OPT`。
- `tools/aggregate_stage5_perf_repeats.py` 对 GDB CPU 和 Timer0 两类原始 JSON 逐组要求健康时长、采样数、CPU/TIMER 频率、非零栈余量及零溢出，并输出每组前后差值、每任务中位数及出现退化的次数；**统计分析本身不裁定硬件 WCET 或普遍抖动优化成功**。
- `tools/tests/test_stage5_perf_repeats.py` 覆盖遗漏任务、样本不足、频率/时长不一致、NaN、CPU 溢出、P99 大于最大值、确实退化等负例。

## 3. 可复现实验入口与证据

性能工作流：`.github/workflows/stage5-cpu-trace.yml`，任务 `task-cpu-measurement`。

```bash
python3 -m unittest discover -s tools/tests -p 'test_stage5_perf_repeats.py' -v
# CI 自动构建 baseline 与 optimized ARM ELF 后运行：
bash tools/run_stage5_perf_repeats.sh \
  build/stage5-cpu-trace/firmware/freertos_demo/freertos_demo.elf \
  build/stage5-cpu-optimized/firmware/freertos_demo/freertos_demo.elf \
  build/stage5-perf-repeat 6
```

原始工件 `stage5-freeRTOS-cpu-trace` 应包含 `build/stage5-perf-repeat/pair-XX-baseline.json`、`pair-XX-optimized.json`、对应 `*-timing.json` 和 `repeated-ab.json`，以及同配置内存 `arm-link-memory-ab.json` 和两份 ARM map。任何一组失败必须保留其原始日志，不把缺失组排除后声称六对通过。

- **首次试验**：[GitHub Actions #41308263682](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308263682)。发现默认**健康运行只有1.2秒**，CPU 新快照约1.23秒，Trajectory 59次间隔、Monitor 11次间隔，不满足计划中的延长窗口要求；比较器正确拒绝。该试验不是固件运行失败，更不是性能改善数据。
- **延长5秒健康阶段的复验**：[GitHub Actions #41308538374](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308538374)，提交 `e5e6dbc92449a06522dec42aafa279223b2325f1`，**工作流 completed/success**，6对全部具有原始 GDB CPU/Timer0 数据，且同提交两种 ARM map 构建及差值检查通过。源数据：[工件 stage5-freeRTOS-cpu-trace](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308538374/artifacts/13084018842)，重点是 `stage5-perf-repeat/repeated-ab.json`、每组 `pair-XX-*.json`、`stage5-cpu-optimized/arm-link-memory-ab.json`。

## 4. 六组 A/B 的真实结果（不是一次性跑绿的空结论）

以下是 `e5e6dbc9` 对应同提交、相同 5 秒健康运行阶段、交错顺序的 **6 个 baseline/optimized 配对**；使用每组实测值。CPU 百分比按百分数显示，`pp` 为百分点。两个各自中位数之差**不一定等于**逐组配对差值的中位数，所以单独报告。

| FreeRTOS 任务与指标 | Baseline 中位数 | Optimized 中位数 | 配对差值中位数（优化－基线） | 改善 / 退化的组数 |
|---|---:|---:|---:|---|
| Monitor CPU 占比 | 0.05% | **0.02%** | **−0.03 pp** | **6 / 0** |
| Control CPU 占比 | 1.23% | 1.23% | +0.005 pp | 2 / 3，另 1 相等 |
| Trajectory CPU 占比 | 0.13% | 0.13% | 0 pp | 2 / 0，另 4 相等 |
| Control P99 周期绝对抖动 | 0.04448 ms | **0.01978 ms** | **−0.03032 ms** | **5 / 1** |
| Trajectory P99 周期绝对抖动 | 0.07128 ms | **0.02732 ms** | **−0.04646 ms** | **5 / 1** |
| Monitor P99 周期绝对抖动 | 0.03376 ms | 0.05538 ms | **+0.01854 ms** | **1 / 5（退化为主）** |

**解释**：
- 降低 Monitor 栈扫描频率带来的 CPU 下降有 **6/6** 次同方向实测支持，可在**此 QEMU 合成健康负载**范围内签收。
- Control 和 Trajectory **大多数配对**改善（5/6），但仍分别有一组退化；Monitor 的周期 P99 在 5/6 配对中反而变差。**不能宣称“所有 FreeRTOS 任务抖动都稳定降低”或“控制周期 jitter 必然改善”**。
- Monitor 只在5秒窗口约取得50个间隔，P99 近似最大值，对离群点特别敏感；Control 和 Trajectory 的旧测量缓冲只保留各自最近128个间隔。因此这不是对完整5秒所有时序样本的精确分布推断。
- 单组 A/B 样本（单独 `monitor-stack-cpu-ab.json`）Control P99 `0.03644→0.03100ms`、Trajectory `0.09628→0.10076ms`、Monitor `0.02716→0.03212ms`，表明**即使单轮 Control 改善，也不能证明另外两个任务一并改善**。
- CPU 跟踪本身使用 QEMU Timer0、任务切换与 tick hook；本结果不代表满负载 PyBullet 或实体 Cortex-M4 的无跟踪运行时延。

### 同提交全功能 ARM map

同一 `e5e6dbc9` 与相同 ARM UART/RTOS 编译配置，只有链接 GC 开关变化：

| 静态资源 | 未启用 `ROBOT_ENABLE_STAGE5_LINK_GC` | 启用该开关 | 差值 |
|---|---:|---:|---:|
| FLASH 分配 | 106,464 B | **88,600 B** | **−17,864 B（约16.8%）** |
| RAM `.data+.bss` | 33,936 B | 33,936 B | **0 B** |

这组数与较早报告的 `105,256→87,384 B` 不是同一个提交/源码配置；**不能跨版本直接相减**。两轮各自都是对应提交下的同配置开关 A/B，均显示约17% FLASH 裁剪、RAM 没减少。

**5.3 当前可签收子项：** CPU 降低的指定短时 QEMU 实验、FLASH 链接裁剪、优先级与调度规则的源码审查、重复性测量工具链。**仍有保留：** 全负载稳定降低控制抖动、满载长时运行及硬件 WCET。即使 CI PASS，这几项依然不能写成全部完成。

## 5. 签收前的技术判据

- C/Host/Stage5 运动集成原有用例不退化；测量优化**不能更改** `robot_defaults.json`、周期、PID 增益、碰撞与到位标准。
- 同 commit 六对都得到真实 GDB CPU + timing 输出、正确编译开关和可核查原始文件；样本不足或单项恶化需如实标注。
- 当前仅有低负载 QEMU 证据时，可以声明“已做固件 CPU/FLASH 优化并有量化 A/B”，**不能声明**“全工况控制周期抖动显著下降”、“30分钟稳态”、“硬件 WCET 已证明”。
- 对照原大纲逐字要求，5.3 **实时性降低抖动这一项是否可签收取决于重复结果和测试覆盖，不能凭修复测量器本身宣告完成**。若仍未稳定改善，就按有保留结项记录，后续以独立长时/高负载对照实验攻关。

历史报告：[Stage5 5.1–5.4 严格审计](19_stage5_original_requirements_evidence_review_2026-11-01.md) · [原 5.3 CPU/map 结果](15_stage5_exit_review_2026-10-30.md) · [参数登记簿](01_parameter_register.md)。
