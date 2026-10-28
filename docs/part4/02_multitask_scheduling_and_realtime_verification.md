# 第四阶段：多任务调度设计与实时性验证报告

**项目**：六轴工业机器人嵌入式运动控制固件开发与仿真验证  
**整理日期**：2026-10-28  
**交付类别**：原项目大纲第四阶段第三项正式文档交付物  
**源码基线**：仿真候选 `48e08c32b6a816014ff9e1b67462150f35d0a313`，已通过 PR #5 合入 `main`。下列历史 QEMU 计时指标有各自被测 SHA，不能一概归到最新提交。  
**验证平台**：ARM Cortex-M4 指令集、QEMU `mps2-an386`、FreeRTOS 与仿真反馈；不涉及实体机器人控制器的时间确定性证明。

## 1. 任务划分与调度配置

应用层根据统一 JSON 编译配置启动 FreeRTOS 任务；算法层保持独立于 RTOS/UART/驱动。所有优先级、周期、栈深度的**生产唯一来源**为 `firmware/freertos_demo/config/robot_defaults.json`，经 `tools/generate_firmware_defaults.py` 生成并供 `application/task_config.h` 使用。下表是所核实配置的只读记录，而不是第二份配置源。

| 任务 | 优先级（越大越高） | 周期 | 任务栈 |
|---|---:|---:|---:|
| Control：反馈校验、PID、命令限速 | **4** | **10ms** | **768 words** |
| ProtocolRX：串口接收/解析 | 3 | 既有接收调度 | 256 words |
| Trajectory：规划与航点参考 | **2** | **20ms** | **1024 words** |
| ProtocolTX：反馈/目标帧发送 | 1 | 默认1000ms；实验输出配置可另行对齐 | 256 words |
| Monitor：状态诊断与任务栈监测 | **1** | **100ms** | **384 words** |

任务深度单位为 **words**，不能误认为 bytes。Control 比 Trajectory 优先级高，使控制周期中的反馈校验及安全门禁优先处理；规划任务按20ms生成参考，Monitor以低优先级记录健康信息。10ms / 20ms / 100ms 为任务计划周期，不能把它们当成已证明的硬件最坏响应时间。

### 软件数据流

~~~text
ProtocolRX / Application motion_pipeline_submit_*()
                         |
               请求队列（容量1，最新覆盖）
                         |
            Trajectory Task（20ms）
            joint / polynomial / Cartesian
            轨迹预检、IK、限位与故障判定
                         |
               参考队列（容量1，最新覆盖）
                         |
             Control Task（10ms）
              同步读取 Motor State
              反馈时效性判断、增量PID
              指令速度和加速度限幅
                         |
        motor_state_set_targets()（输出门控）
                         |
            既有 ProtocolTX / PyBullet

 Monitor Task（100ms）：故障计数、运行状态、任务栈水位
~~~

- `application/services/motion_pipeline.c` 维护单槽请求队列、参考队列和运行态诊断，`xQueueOverwrite` 采用**最新请求/最新参考覆盖**策略，避免待处理运动请求无限排队。
- `application/motor_state.c` 使用 FreeRTOS Mutex，同步复制一组六轴反馈及时间戳；算法模块不直接无锁访问驱动状态。
- `trajectory_task.c` 在轨迹开始前检查反馈、起点、全程多项式极值、笛卡尔航点 IK/FK；失败则停止发布有效参考。
- `control_task.c` 遇到失效参考、反馈超时或控制计算故障，重置 PID 状态并设置取消/故障标志，避免继续生成新目标。这**不是**硬件独立急停。
- 反馈超时 **3000ms**；参考有效期按 **3×20ms = 60ms** 的规划周期倍数派生，均由配置控制。
- `monitor_task.c` 使用 `uxTaskGetStackHighWaterMark()` 观察已用栈水位，统计运行计数和故障。

## 2. 构建开关与测试模式

`firmware/freertos_demo/CMakeLists.txt` 将轨迹/PID纯算法编入固件目标，但以下选项**默认均为 OFF**：

| 构建开关 | 用途 |
|---|---|
| `ROBOT_ENABLE_STAGE4_TASKS` | 显式创建 Trajectory、Control、Monitor 三任务 |
| `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` | 仿真试验允许控制目标写入 Motor State；需要 Stage4 TASKS 同时启用 |
| `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST` | 专用假反馈任务自检，强制命令输出 OFF |
| `ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION` | 独立的 QEMU↔UART↔PyBullet 集成试验；需单独显式启用 |

**本报告的定时与栈测量使用 Stage4 自检模式，命令输出关闭**；不把该模式的 RTOS 任务数据等同于真实 UART 物理闭环的性能。

## 3. 测量方法及统计口径

核心实测依据为 [GitHub Actions #40444477809](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40444477809)：测试固件在真实 ARM QEMU/GDB 下运行，使用 MPS2 CMSDK Timer0 的 **25,000,000 Hz** 计数器。采样数据写入固定长度诊断缓冲区，再由 `tools/analyze_stage4_timing.py` 解析 GDB 原始日志。

- **绝对周期偏差**：相邻任务唤醒时刻的差与计划周期之间差值的绝对值；按 Timer0 计数换算为 μs。
- **P99**：使用最近阶百分位 `ceil(0.99×N)`，只反映当前观测样本。
- **处理经过时间**：从一次周期任务唤醒到本轮处理末尾的 QEMU 虚拟经过时间，**包含抢占等待**，不是 CPU 独占执行时间或 WCET。
- **最小剩余栈**：FreeRTOS `uxTaskGetStackHighWaterMark()` 报告的最低剩余量，单位 words；峰值已用栈 = 分配栈 − 最小剩余栈。
- 诊断模式在 `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON` 时编入 `stage4_timing_probe.[ch]`；普通固件不引入这份计时数组与测试开销。

## 4. QEMU 短时定时、栈水位实测

首次约1.2秒健康模拟反馈阶段的观测如下（它不是后来60秒试验的同一数据窗口）：

| 任务 | 周期 | 间隔样本/处理样本 | P99绝对周期偏差 | 最大绝对偏差 | P99处理经过时间 | 最大处理经过时间 |
|---|---:|---:|---:|---:|---:|---:|
| Control | 10ms | 119 / 120 | **54.88μs** | 180.64μs | 290.88μs | **2081.32μs** |
| Trajectory | 20ms | 59 / 60 | **230.44μs** | 230.44μs | 1931.24μs | 1931.24μs |
| Monitor | 100ms | 11 / 12 | **13.08μs** | 13.08μs | 285.32μs | 285.32μs |

| 任务 | 分配栈 | 最小剩余 | 峰值使用 | 剩余比例 |
|---|---:|---:|---:|---:|
| Control | 768 words | **375 words** | 393 words | 48.8% |
| Trajectory | 1024 words | **609 words** | 415 words | 59.5% |
| Monitor | 384 words | **349 words** | 35 words | 90.9% |

本次测量的最大处理经过时间都低于各自任务周期；P99 绝对周期偏差也低于规定的**QEMU 试验政策门限（计划周期的10%）**。门限是分析工具的测试规则，而不是工业实时规格。

测试调试 ELF 的资源统计：`.text=90,368 bytes`、`.bss=40,684 bytes`、`.data=8 bytes`、`.isr_vector=192 bytes`。其中 BSS 包含32KiB FreeRTOS heap及约6KiB测试诊断数组；**不能拿这个测试 ELF 数字冒充最终发行固件内存占用**。

## 5. 60 秒稳定性与故障门禁

独立 [GitHub Actions #40446109227](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40446109227) 将 `ROBOT_STAGE4_QEMU_HEALTHY_RUN_MS=60000`，得到 **60秒 QEMU 虚拟时间**运行记录：

| 观察项 | 实测 |
|---|---:|
| FreeRTOS `finished/checks/failed` | **1 / 29 / 0** |
| 已接受请求 | 1 |
| Trajectory 采样数 | **3150** |
| Control 执行数 | **6298** |
| Monitor 执行数 | **632** |
| Control / Trajectory / Monitor最小剩余栈 | **375 / 609 / 349 words** |
| 故障/过期反馈计数 | **1 / 1**（停掉模拟反馈后的预期注入） |

60秒版本采用**循环缓存的最近128次计时样本**；该窗口中 Control / Trajectory / Monitor 的 P99 偏差分别为 **19.56 / 35.60 / 51.64 μs**，最大处理经过时间分别为 **128.12 / 9.24 / 64.40 μs**。这些是临近60秒的稳定运行观测，**不能覆盖或抹去短时试验中 Control 2.08132ms 的较大处理峰值**。

健康期先定期注入静态模拟反馈，停止反馈后按生产策略等待超时，再核对控制器不继续生成有效目标。该故障链纳入29项 FreeRTOS 自检并通过；多项式错误计划拒绝、随后正确计划恢复的独立 FreeRTOS **8/8** 检查也通过，相关 Trajectory 栈在该额外试验中观测最小剩余 **467 words**。

这项60秒试验**没有**持续收发 UART、连续高负载求 IK、PyBullet 动态关节闭环、频繁换队列或30分钟任务压力，不应当称为满载长稳测试。

## 6. 工程复验及结论

| 验证类别 | 已有记录 | 证据来源 |
|---|---|---|
| Host 控制与规划 | 6/6 CTest；C桥接1/1 | 用户本机2026-10-28运行报告 + [Stage4 CI #40639281389](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40639281389) |
| ARM/QEMU 任务与故障 | FreeRTOS 29/29；多项式8/8 | 同次本机非驱动验收；[早期QEMU定时CI #40444477809](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40444477809) |
| 60秒虚拟运行 | 29项自检成功，静态假反馈、断链故障检查 | [扩展QEMU #40446109227](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40446109227) |
| RC 构建与集成回归 | Linux PID、Windows Host、QEMU/FreeRTOS/PyBullet三 Job success；三段 UART 3/3 到位 | [RC #40639281389](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40639281389)，源码 `48e08c3` |
| 合并后的主线编译回归 | native regression / ARM firmware build 2/2 success | [main #40642030001](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40642030001) |

**复现入口**（已安装 ARM GCC、CMake、QEMU/GDB、Python 依赖的 Windows 环境）：

~~~powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
~~~

日志与实验记录位于测试工作区 `build/`，云端原始 GDB/JSON/CSV 数据见对应 Actions Artifacts。完整用户本机原始文件未统一提交到正式交付介质，不应把“日志有路径”误写为“原件已归档签收”。

**第四阶段判断**：FreeRTOS规划、控制、通信、监测任务及队列、互斥锁、故障停止生成新参考、周期偏差与栈水位的**软件仿真验证**已有证据，可交付调度设计与仿真实时性报告。尚未取得真实硬件 WCET、独占 CPU 占比、全调用链最坏栈深、高负载30分钟运行及实体电机安全停机证据。这些限制不得被本报告中的短时 QEMU 通过替代。

### 历史资料索引

- [原多任务实时性记录](../part4_archive/04_realtime_verification.md)
- [原QEMU定时与栈水位完整数据](../part4_archive/08_stage4_qemu_timing_and_stack_2026-10-27.md)
- [原参数登记簿](../part4_archive/00_parameter_register.md)
- [原全阶段实现说明](../part4_archive/02_stage4_full_implementation_guide.md)
- [原Stage4封版候选审查记录](../part4_archive/12_stage4_release_candidate_2026-10-28.md)

历史资料留存其当时的配置与测试状态；现行控制参数仍以生产 JSON 单一来源为准。