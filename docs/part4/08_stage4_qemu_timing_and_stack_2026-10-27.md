# 第四阶段 FreeRTOS 定时与栈水位实测报告

记录日期：2026-10-27。软件源分支：`feat/stage5-system-integration-2026-10-27`。冻结备份 `backup/stage4-verified-2026-10-27` **未修改**。

## 1. 证据及测试条件

首次包含完整栈水位、计时值、原始GDB和内存映射文件的 [GitHub Actions Run #40444477809](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40444477809)，`stage4-arm-qemu-algorithm-and-rtos` Job运行真实 `qemu-system-arm -M mps2-an386`、ARM编译的 `freertos_demo.elf` 及 GDB。

- 控制任务周期 `10ms`，轨迹任务 `20ms`，监测任务 `100ms`，全部从 `config/robot_defaults.json` 唯一配置生成；栈分配分别 `768/1024/384 words`。
- 现有 `timer_driver_get_counter()` 读取MPS2 CMSDK Timer0，`timer_driver_get_frequency_hz()` 返回实测配置 `25,000,000 Hz`。没有复制硬件时钟常量或引入新计时设备。
- **QEMU专用** `stage4_timing_probe.[ch]` 仅当 `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON` 且命令输出 OFF 时参与编译；正常固件无诊断数组和额外计时开销。
- 已有 Stage4 RTOS 自检在1.2秒内持续注入模拟反馈，并提交一次关节运动请求；自检只在这一**健康运行时段**冻结时间统计，然后另外停止反馈模拟失联。失联故障注入依然通过29项总自检中的原13项。
- 每任务最多记录**最开始128次**处理样本和相邻唤醒间隔；本次统计样本数见下表。P99采用最近阶百分位 `ceil(0.99*N)`，不是拟合概率的工业最坏情形。

### 数值定义

- `period_jitter = abs((Timer0上次唤醒计数 - 本次唤醒计数) - 计划周期计数)`，单位Timer0计数。报表按 `计数×1e6/25,000,000` 换算微秒。
- `elapsed_processing` 为从本次 `vTaskDelayUntil()` 返回后到本次任务处理末尾的**虚拟经过时间**。包含被更高优先级抢占的时间；不是独占CPU执行时间，也不是WCET。
- `uxTaskGetStackHighWaterMark()` 提供自任务运行以来的最小空闲栈空间，单位 **words**，不是字节。 `peak_used_words = 分配words-最小剩余words`。
- 计时器数据均从QEMU虚拟外设读取，**不等于**GitHub runner墙钟时间，也不能作为真实电机/硬件Cortex-M4上的绝对执行时长保证。

## 2. 实际样本与指标

| 任务 | 周期 | 间隔样本数 | 处理样本数 | P99绝对周期偏差 | 最大绝对偏差 | P99处理经过时间 | 最大处理经过时间 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Control | 10ms | 119 | 120 | **54.88μs** | 180.64μs | **290.88μs** | 2081.32μs |
| Trajectory | 20ms | 59 | 60 | **230.44μs** | 230.44μs | **1931.24μs** | 1931.24μs |
| Monitor | 100ms | 11 | 12 | **13.08μs** | 13.08μs | **285.32μs** | 285.32μs |

| 任务 | 配置栈words | 最小剩余words | 峰值使用words | 剩余占分配比例 |
|---|---:|---:|---:|---:|
| Control | 768 | 375 | 393 | 48.8% |
| Trajectory | 1024 | 609 | 415 | 59.5% |
| Monitor | 384 | 349 | 35 | 90.9% |

上述数据仅为本次短时QEMU仿真中的实际观测。**例如Control的最大处理经过时间2.08132ms，不能据此宣称硬件WCET=2.08132ms。** 对应历史运行的ARM模拟QEMU自检 `checks=29, failed=0`；在新的CI运行完成之前，不能把此报告数据标为经过所有后续源码更改的最终量化指标。

## 3. 测试输出、可复现性与内存占用

- 定时源码：`application/tasks/stage4_timing_probe.[ch]`、任务中仅受测试编译门控控制的两处计时调用。
- 统计与校验：`tools/analyze_stage4_timing.py`，只从**GDB原始日志**解析，校验有效样本、P99不大于最大值与栈实际余量，写出 `build/stage4-spec-qemu/rtos.timing.json`。
- 自动化入口：`.github/workflows/stage4-spec-acceptance.yml`，保存原始GDB日志、JSON统计、ELF分段大小、测试专用linker `.map`。
- `arm-none-eabi-size -A`本次诊断ELF的 `.text=90368 bytes`，`.bss=40684 bytes`，`.data=8 bytes`，`.isr_vector=192 bytes`。其中BSS包含32 KiB FreeRTOS堆与本次约6 KiB的诊断统计数组；**这是带调试与自检的固件ELF**，不是发布固件内存占用，也不是可使用RAM的压力上界。
- 源码仅以当前测试代码和配置作为单一数据来源，文档数字只是复制本次运行证据，不能反向被当作新的硬编码控制参数。

## 4. 新增自动拒绝门限（仅QEMU短时试验政策）

本报告最初只校验P99不高于最大观测值，这不足以发现异常调度。后续将验收政策**仅**加在 `tools/analyze_stage4_timing.py`，且补了纯Python单元测试验证失败分支：

- `P99(abs(相邻唤醒间隔−计划周期)) <= 计划周期×10%`；
- `最大单次处理经过时间 < 计划周期`；
- 样本数、GDB完成状态、29项检查、Timer0频率、栈剩余范围不满足时同样失败。

这些是**短时QEMU回归工程门限**，绝非产品规格或实机硬实时保证。控制任务10ms时对应P99绝对偏差门限1ms，本次观测54.88μs；最大处理经过时间应小于10ms，本次观测2.08132ms。

**门限定义的唯一来源**是分析脚本中的 `MAX_P99_JITTER_FRACTION` 与 `MAX_PROCESSING_FRACTION`，不得另在固件控制策略或JSON任务默认值中复制。此次门限修改的再次CI成功状态须以最新运行结果为准，不复用较早试验来声称新版本通过。

## 5. 本报告尚未证明的事情

1. 没有独占CPU运行时间/整系统CPU占用率，故报告`elapsed_processing`不等同WCET或CPU使用率。
2. 仅1.2秒健康运行、短时失联注入，没有30分钟满载持续稳定性；尤其Cartesian逆解高负载、协议高吞吐干扰、频繁替换队列等情况下P99可能显著变化。
3. 没有真实Cortex-M4硬件、DMA/UART活动干扰、真实中断优先级/电机输出负载的测量。
4. 真实UART请求到PyBullet反馈闭环、异常帧与硬件安全停机仍属于第5阶段系统集成门禁。

**阶段结论**：已经为第四阶段的调度报告补充了可审计的短时QEMU周期偏差P99、最大处理时间、栈水位与ELF内存分段**实测**，但是满负载/长时间/硬件级实时性仍需独立验收。
