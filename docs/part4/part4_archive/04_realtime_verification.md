# 第4阶段 FreeRTOS 实时性与联调验证模板

> **封版候选状态（2026-10-28）**：本页为历史实时性验证记录，早期“待测”条目保留原始过程；已完成的短时和60秒 QEMU 实测统一以 [`08_stage4_qemu_timing_and_stack_2026-10-27.md`](08_stage4_qemu_timing_and_stack_2026-10-27.md) 为依据，最终签收门槛见 [`12_stage4_release_candidate_2026-10-28.md`](12_stage4_release_candidate_2026-10-28.md)。任务 CPU 独占占比、实体 MCU WCET 与30分钟满载仍未测得，不应将 QEMU 数据扩展为硬件性能保证。

已测得周期偏差、任务处理经过时间、栈水位和测试ELF内存段；没有FreeRTOS任务CPU占比/独占执行时间数据，短时值也不是WCET。已完成60秒静止反馈QEMU健康运行，30分钟高负载和UART持续干扰长稳尚未测试。

## 当前证据

- 新增代码：`application/tasks/{trajectory_task,control_task,monitor_task}.c`、`application/services/motion_pipeline.c`。
- Control/Trajectory/Monitor Priority: 4/2/1，协议接收/发送任务依旧为 3/1。
- Control/Trajectory/Monitor Period: 10ms/20ms/100ms，配置来自 JSON 生成宏。
- 三任务当前栈深：768/1024/384 *words*，不是 bytes；增大只是给 IK 分支计算预留运行空间，**不是栈分析已通过**。
- Request Queue / Target Queue 都只保留最新一份；任务时间线有明确单一 writer。
- Control 通过 `motor_state_get_feedback()` 获取整组已加锁的六轴反馈，检查反馈有效性和超时。输出默认关断。
- Monitor 通过 `uxTaskGetStackHighWaterMark()` 读取剩余栈水位。

## 历史验收模板及当前结果

下表早期曾作为待填模板；以下状态已依据2026-10-27本机统一验收和QEMU/GDB定时报告更新。各行不代表实物硬件结项。

| 测试 | 判据 | 本次结果 |
|---|---|---|
| ARM 编译 | Stage4 OFF / TASKS ON / OUTPUT ON 三种配置均构建 | TASKS ON 且 OUTPUT OFF、默认配置及独立QEMU自检构建通过；OUTPUT ON未作为本次验收项 |
| QEMU 纯算法测试 | `g_stage4_test_finished=1`, `failed=0` | **20/20通过**，新增多项式整段预检 |
| Host 原生算法 | 当前6个 Stage3/4 CTest 全通过 | **6/6本机通过** |
| Stage2 Host 回归 | Unity及纯C Host回归 | **Unity 25/25；纯C CTest 5/5** |
| FreeRTOS 任务启动 | 任务启动且无启动失败 | **QEMU启动检查通过；原RTOS 29项+新增多项式8项均失败0** |
| Stack High Water Mark | 记录峰值剩余栈 | 历史健康窗口Control/Trajectory/Monitor 375/609/349 words见08；本轮多项式路径规划最小剩余**467 words**；均非硬件上界 |
| FreeRTOS 队列 | 新请求替换策略 | RTOS自检覆盖请求提交/控制路径；高频满载队列压力仍需单独扩展 |
| 反馈过期 | 停止生成新 Motor Target；fault 计数上升 | **已通过QEMU故障注入**，29项总检查失败0 |
| 控制周期抖动 | 采集执行时间、唤醒间隔、P99 和最大值 | **60秒QEMU健康运行已测**；指标定义和数值见08报告，不代表WCET/CPU占比 |
| IK路径故障 | 无解/近奇异/限位时报告拒绝且不继续指令 | 近奇异参考输入2/2通过；全部无解/限位路径及任务级安全行为仍需补测 |
| Native C→PyBullet闭环 | MAE、RMSE、超调、稳态误差 | **本机单轴和六轴场景通过**，指标见03；不是UART/FreeRTOS闭环 |
| 长时间 | 至少30分钟仿真无异常/泄漏/栈溢出 | 60秒QEMU健康基线通过；30分钟满载尚未执行 |

## 开关与安全条件

`ROBOT_ENABLE_STAGE4_TASKS` 默认为 OFF。
`ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` 默认为 OFF。
必须明确打开，且 Task 开启成功以后才允许闭环控制。当前代码还没有完整的独立急停总线、机械碰撞检测或实物硬件实时验证。不要在实体机器人上启用此实验选项。

## 不能伪称已通过

没有 ARM/QEMU 工具链运行输出、GDB 变量和 PyBullet 数据就不能填写准确 PID 调优数值、CPU WCET、任务调度 jitter 或电机位置跟踪精度。

## 本批新增验证边界（2026-10-26）

- `task_stack_depth_words.control` 从 512 调整至 768 **words**；仅为速度/加速度双限制计算新增余量，绝非最大栈使用证明。
- 新增 `joint_command_limiter` C 模块根据同一份 Stage4 默认配置限幅命令速度与加速度、内部累计0.01度以下小数位。原先仅限制速度，现在可以对加速度进行纯算法回归。
- 新增第6组 Host CTest `stage4_command_limiter_host`、独立模拟 DLL 的 `stage4_sim_bridge_smoke`、以及 PyBullet DIRECT 下的动态六轴状态闭环试验。
- 旧第4阶段 QEMU 14/14、旧第3阶段 QEMU 128/128、FreeRTOS Monitor 命中断点均有来自用户的原版日志。新修订版本尚未通过重新构建和实测，不能复用旧证据断言新版本已验收。
- Windows 总验收入口：`tools/test_stage4_once.ps1`；默认第4阶段电机输出编译门控为 OFF，不会触发真实机械运动。
- 动态 DLL ↔ PyBullet 测试**不包括**真实 FreeRTOS 周期抖动、UART 故障恢复、硬实时 WCET、实机碰撞检测。它可以测量控制算法与 URDF 动力学交互，但不能宣称整个嵌入式系统闭环联调完成。

因此第四阶段仍保留以下未通过结项门禁：真实QEMU/FreeRTOS运动请求贯通、UART闭环动态测试、丢包/旧反馈故障注入、周期P99与最坏值、完整长时负载和最终增益整定记录。拒绝用“能编译”或“Python GUI 能动”替代这些门禁。

## FreeRTOS 三任务真实数据流自检（新增；执行状态待验）

为了避免把“Monitor Task能命中断点”错误等同于“Controller实际执行”，新增 `tests/qemu_target/stage4_rtos_selftest.c`：

- 独立 CMake 门控 `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON`，强制 `ROBOT_ENABLE_STAGE4_TASKS=ON` 且 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF`，否则 CMake 配置阶段直接报错。
- **QEMU专用**任务以小幅六轴反馈样本（J2 +1°目标）验证 `motor_state_update_feedback` → `motion_pipeline_submit_joint` → FreeRTOS Trajectory Task → Control PID → Monitor 三任务实际调度与队列贯通。
- 这个测试明确采用**固定不动的假想反馈**，不会模拟真实机械运动；目的是检测 `requests_accepted=1`、`trajectory_samples>0`、`controller_steps>0`、`monitoring_ticks>0`、无fault及三任务栈的非零水位。
- 在 `stage4_rtos_selftest_done` 断点处检查 `g_stage4_rtos_test_report.checks=11`、`failed=0`、`g_stage4_rtos_test_finished=1`。
- 独立 `build/cmake-arm-stage4-selftest` 构建目录，**不会污染正常固件 `build/cmake-arm`**。默认开关 OFF，真正电机输出始终关闭。

该自检和 PyBullet Native C 闭环是互补试验：前者测 FreeRTOS 内部实时任务间数据流，后者测算法和物理引擎相互作用；两者都不宣称完成真实 UART 闭环或 FreeRTOS 硬实时延迟测量。

最新版统一验收脚本 `tools/test_stage4_once.ps1` 包含此测试以及 Native C/PyBullet，步骤变成8项；尚未收到新版本实际执行记录。

## QEMU FreeRTOS 三任务真实执行验收证据（2026-10-26）

运行来源：[GitHub Actions #40345768977](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40345768977)，`stage4-arm-qemu-rtos-runtime` Job 真实使用 `arm-none-eabi-gcc`、`qemu-system-arm -M mps2-an386`、`gdb-multiarch`，而不是只执行交叉编译。测试开关 `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON` 且 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF`。

原先 Windows 本机在 `stage4_rtos_selftest_done` 断点处无法 `printf` 变量，GDB 报 `g_stage4_rtos_test_finished has unknown type`，根因是 `freertos_demo` 没有独立 QEMU 目标的 `-g3` 类型信息。现仅在专用测试构建添加 `-O0 -g3`，普通固件编译不变；同时扩展了 GitHub Actions 云端 QEMU/GDB 运行验证。

**真实 GDB 输出**：

| 指标 | 本次云端实测 |
|---|---:|
| `g_stage4_rtos_test_finished` | 1 |
| `g_stage4_rtos_test_report.checks` | 11 |
| `g_stage4_rtos_test_report.failed` | 0 |
| `requests_accepted` | 1 |
| `trajectory_samples` | 60 |
| `controller_steps` | 118 |
| `monitoring_ticks` | 12 |

此结果真实证明软件仿真中的 **模拟反馈→运动请求→Trajectory Task→Control PID计算→Monitor Task** 流程执行，且自检检查项通过；并不表示 `motor_state_set_targets` 电机输出开启、UART 往返跑通、PID 驱动真实或仿真机械执行完一条运动。满载抖动和30分钟稳定性仍待测。

## Windows 本机统一验收报告（2026-10-26，用户确认）

用户运行 `tools/test_stage4_once.ps1` 后明确反馈：“ok全过了”。据此将**该脚本包含的全部本机回归步骤**登记为用户报告通过，包括：

1. 本机六组 Stage3/4 Host CTest 构建与测试。
2. ARM Cortex-M4 固件与 QEMU 测试构建。
3. QEMU 运动学和第4阶段算法回归及近奇异 IK 回归。
4. FreeRTOS Monitor 任务启动检查。
5. 非驱动 QEMU 模拟反馈→轨迹请求→Trajectory/Control/Monitor 三任务实际运行自检。
6. 原生 C 共享库构建、smoke 和 PyBullet DIRECT 单轴、六轴动态闭环测试。

**证据等级**：这是用户对最后一次运行结果的明确确认，尚未收到该次 `build/stage4-final-validation.log` 的逐行内容；本节不额外编造本机执行计数、仿真数值或持续运行时间。对应算法、任务、PyBullet 云端真实证据已在前文分别记录。

**通过范围**：第四阶段**本机统一非驱动回归测试通过**。不等同于完整工业机器人系统验收。后续仍需如实单列：真实 UART↔PyBullet 闭环、通信异常与反馈过期故障注入、调度周期/抖动及最坏栈量、长时稳定性、实际硬件/安全联锁等验证。保留源代码和参数登记簿审查，不因一次脚本通过而跳过集成安全测试。

## 全局审查后追加：失联故障注入13项实测

[GitHub Actions Run #40349948758](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40349948758) 真实QEMU/GDB在修订后的自检源码上得到：

| 字段 | 实测 |
|---|---:|
| `finished`、`checks`、`failed` | 1、13、0 |
| `accepted` | 1 |
| `trajectory_samples` | 210 |
| `controller_steps` | 418 |
| `monitoring_ticks` | 43 |
| `controller_faults` | 1 |
| `planner_faults` | 0 |
| `feedback_stale_count` | 1 |

测试先定期注入模拟反馈约1.2s，再停止反馈超过正式 Stage4 配置的 `feedback_timeout_ms`，最后间隔再次检查Control步数不继续增加。验收为软件任务的故障检测与停止生成新目标，**不代表实体驱动器能主动制动或掉电安全**。原来的11项成功记录是未注入失联的旧证据，不与此结果混用。
