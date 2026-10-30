# 第五阶段 5.1–5.4 系统集成与功能扩展：结项审查及证据

> 审查日期：2026-10-30；对象：软件仿真工程 ARM Cortex-M4 / FreeRTOS / QEMU UART / PyBullet UR5。与项目任务书第五阶段逐项核对。**不得将云端无头闭环验收写成 Windows GUI、动态障碍或实体电机安全验收。**

## 一、范围与结项判断

**主线软件仿真核心：达到阶段5集成/静态避障闭环目标。**
**阶段5原生 GUI 演示：Linux 原生窗口与 Windows/Linux 无头云端回归已通过；Windows 本机窗口、鼠标操作和交互 HOLD 仍待验收。**
**性能优化：有真实 CPU、周期抖动与内存 A/B 实测及路径优化数据，但尚不具备可移植的实时性/WCET 证明。**

该项目没有实体机器人。原任务书第六阶段的“原点→抓取点→途经点→放置点→返回原点”、多场景任务成功率、长时间连续运行、交付视频和完整用户手册是下一阶段工作，不能既提前宣称已完成，也不能拿它们当作所有第五阶段 CI 的前置条件。

## 二、任务书逐项验收

| 项目 | 已验证的事实 | 剩余限制/处理 |
|---|---|---|
| **5.1 全模块集成** | C 运动学、约束与 PID 由真实 QEMU ARM FreeRTOS 固件执行；UART 向 PyBullet 发送关节设定并接收反馈；4 段/28 段都取得逐段 ACK/实际到位，记录诊断计数 | UART ACK 不是物理到位；物理闭环由 `MotionSequencer` 独立等待反馈到位 |
| **5.2 C APF 静态长方体避障** | 真实 PyBullet AABB → 生产 C APF 多段路径 → C IK/FK/关节约束 → PyBullet 全臂离散路径预检 → FreeRTOS QEMU UART 执行；有被直线阻挡的场景和至少 10 mm 全臂净距检查，4/4 与 28/28 均成功 | **重要架构限定**：相同 C APF 源码链接进固件，但本次演示由仿真主机的同源 C 动态库完成整条 APF/IK/整臂预检，再通过 UART 把关节航点交给固件；FreeRTOS 任务本身尚未在运行时接管带障碍物输入的 APF 任务编排。不应称“固件端运行时自主重规划”；静态离散采样也不是连续时间的绝对碰撞证明 |
| **5.3 CPU/内存/实时性** | 真实 QEMU/GDB 逐任务 CPU 计时、Timer0 周期处理时间/抖动及任务栈水位；ARM GNU ld map 同编译配置实测；APF 安全压缩经预检从 28 个航点减少为 4 个，减少独立 UART 运动请求 | CPU/Timer0 来自短时 QEMU 自检任务而非全部 PyBullet 负载；下一阶段要做长时、高负载 P99/WCET 和系统级性能与故障压力测试；没有依据就不能声称 ISR 优先级优化或者 FLASH 减少 |
| **5.4 QEMU/GDB/日志故障排查** | GDB 在真实 QEMU 断点获取任务运行状态；字节流坏校验/丢字节/分块、半帧超时恢复有 C Host 与端到端测试；`NOT_READY(-7)` 复现定位到反馈和引用时间戳 TOCTOU，修复后4段与28段物理闭环成功、故障计数为零 | 完整的异常矩阵、各种崩溃注入与长时间自动恢复属于继续加强项；GDB 测量证明读取寄存器/断点与观测，不意味着已对所有错误逐个完成断点定位 |

## 三、可复现结果与对照

### 1. 真实动态避障（同一具名被阻挡场景）

| 指标 | APF 原始路径 | 开启 C 几何验证压缩 |
|---|---:|---:|
| 生产 C APF 原始笛卡尔航点 | 28 | 28 |
| 最终执行航点 | 28 | 4 |
| 实际逐段到位 | **28/28** | **4/4** |
| 运行时整臂碰撞/净距检查 | 2145次，无违规 | 913次，无违规 |
| 最终每段最大误差 | 0.12° | 0.09° |
| PyBullet 累计仿真物理时间 | 21.45秒 | 9.13秒 |
| QEMU/CI 墙钟时间 | 43.85秒 | 18.99秒 |
| 规划/控制/反馈过期/最终故障码 | 0 | 0 |

**优化解释：** 减少了 24/28 ≈ 85.7% 的独立执行航点；两组是在不同 CI 实验中得到的真实 QEMU/PyBullet 实测，不能仅凭这两次墙钟时间宣称可重复、同机器控制变量的性能加速比。压缩在中间任务点后不跨任务段，并且每个候选都重新经过 C IK 和整臂碰撞门禁。

原始长路径：[Actions #40881864044](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40881864044)，PR #26，主线 `259b95a`。缩短路径：[Actions #40880987874](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40880987874)，PR #25，主线 `de851eea`。

### 2. FreeRTOS CPU 与时间基准（QEMU 低负载/自测）

初始基线：[Actions #40737897100](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40737897100)，原始 CPU Trace：4.748秒内 Control 0.96%、Trajectory 0.21%、Monitor 0.06%、ProtocolRX 0.01%、ProtocolTX 0.03%、IDLE 97.95%，overflow=0。与真实高负载不同，**不可称高负载剩余算力 97.95%**。含 Test 任务时的 GDB 栈高水位和周期 P99 数据见本 PR 的 `stage5-freeRTOS-cpu-trace` 附件 `observed-timing.json`，以最终成功的 Actions 版本为准。

最新完整 GDB 测量：[Actions #41015285084](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41015285084)，工件 `stage5-freeRTOS-cpu-trace`（`observed.log`、`observed.json`、`observed-timing.json`），CPU 观测窗 **4.735 秒**，Control **0.98%**、Trajectory **0.21%**、Monitor **0.06%**、IDLE **97.87%**，overflow=0。

| 任务 | 周期(ms) | 间隔采样数 | 周期抖动 P99 / 最大 (ms) | 处理耗时 P99 / 最大 (ms) | 观察到的最小栈余量 (FreeRTOS words) |
|---|---:|---:|---:|---:|---:|
| Control | 10 | 119 | **0.0606 / 0.21164** | 0.30828 / 2.44832 | 371 |
| Trajectory | 20 | 59 | **0.032 / 0.032** | 2.211 / 2.211 | 465 |
| Monitor | 100 | 11 | **0.04256 / 0.04256** | 0.71016 / 0.71016 | 330 |

该计量复用 `stage4_timing_probe.c` 的 Timer0 原始计数和 FreeRTOS `uxTaskGetStackHighWaterMark()`，未复制另一套时钟或在正式运行任务中加入排序/阻塞操作。周期绝对抖动按 nominal_period 的 Timer0 ticks 差值测得，P99 为最近最多 128 个样本的 nearest-rank 百分位；**Monitor 只有11个间隔样本，不能将其 P99 当成长期稳定性保证**。这些任务数据来自自测条件的短时 QEMU 运行，不能直接外推到交互式 GUI 或第六阶段满负载。

### 3. ARM 链接静态内存同配置 A/B

原始早期单版默认构建基线 `FLASH 85,424 B / RAM 33,512 B`：[Actions #40700431434](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40700431434)。它默认关掉 Stage4 任务，**不可**和实际执行版直接比较。

本次按**相同 ARM toolchain / linker / 启用 Stage4 任务 / 关闭实体命令输出 / 关闭额外自检 / 打开 map** 在同一次 GitHub Actions 内构建：
- before `d2ca58f`：FLASH **99,792 B**；RAM **33,796 B**；
- after 本阶段提交基准：FLASH **99,840 B**；RAM **33,796 B**；
- 同配置差值：FLASH **+48 B**，RAM **0 B**。

这组数据反映时序安全修复带来少量 FLASH 增加，并**没有**证明全局变量裁剪带来内存缩小。评估静态资源占用合理性时必须保留这个事实。原始 ELF/map 与 JSON 对比存在 `Stage5 comparable ARM memory baseline and post-integration audit` 云端工作流工件中。

## 四、排障台账归档

- **S5-009** 生产 C 协议在丢一字节后恢复失败：问题已通过解析器重同步与回归修复，包含坏校验和分包。
- **Issue #22** 规划器意外 `NOT_READY(-7)`：FreeRTOS 接受新反馈后旧 tick 与新 accepted_tick 相减溢出；已修复 `motor_state_get_fresh_feedback()` 和 Control post-reference tick，且两类长短路径云端实测通过。
- **PR #15 → PR #28**：历史 GUI 观察器在旧固件下第2段超时；旧 PR #15 已关闭。干净的 PR #28 基于当前修复后的固件，Linux Xvfb 实际 p.GUI 三段和 Linux/Windows 无头 QEMU UART 观察器六项 CI 全部通过，已合入主线提交 `970a887c`。[原始 CI #41015789848](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41015789848)。Windows 原生窗口与鼠标拖拽仍需用户机器现场确认，不等同于云端 Windows headless。
- 原先 Issue log 的 `S5-002～008` 是 2026-10-27 的历史状态，须按这份结项表及最新 CI 判断，不要把当时“未实现”误当当前结论。

## 五、不可提前宣称的内容

- 不使用真实工业机器人，电机输出默认关闭，禁止拿本项目作为实体工业安全认证。
- APF 可能局部极小；自动避开任意形状/动态障碍、奇异姿态绝对稳定和连续时间扫掠碰撞证明尚未实现。
- Linux Xvfb 原生 GUI 自动三段演示已成功，Windows headless 云端程序也已通过。**Windows 桌面 p.GUI 显示、鼠标拖拽与交互 HOLD 的正确性**仍需 Windows 本机验证（见 [一次性验收指南](16_stage5_native_gui_windows_acceptance.md)）。
- 第六阶段的抓取—放置任务编排、多场景成功率、连续长时运行、视频/技术交付包仍未完成。

## 六、结项原则

**第五阶段云端软件仿真核心闭环及 Linux 原生 GUI 三段演示已通过，但第五阶段全部事项尚不具备无保留签收条件**：在 GUI 的集成范围保留未验收状态；若项目选择按最初 PDF 的 5.1–5.4 严格结项，应书面保留“主机侧规划编排而非 RTOS 内自主 APF”这一架构偏离，并确认交付边界。所有新数据应以原始云端工件为准，不许从成功日志倒推不存在的 CPU/WCET、内存收益和硬件安全等级。
