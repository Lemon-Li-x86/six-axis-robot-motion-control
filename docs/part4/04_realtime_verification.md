# 第4阶段 FreeRTOS 实时性与联调验证模板

## 当前证据

- 新增代码：`application/tasks/{trajectory_task,control_task,monitor_task}.c`、`application/services/motion_pipeline.c`。
- Control/Trajectory/Monitor Priority: 4/2/1，协议接收/发送任务依旧为 3/1。
- Control/Trajectory/Monitor Period: 10ms/20ms/100ms，配置来自 JSON 生成宏。
- 三任务的栈深：512/1024/384 *words*，不是 bytes；增大只是给 IK 分支计算预留运行空间，**不是栈分析已通过**。
- Request Queue / Target Queue 都只保留最新一份；任务时间线有明确单一 writer。
- Control 通过 `motor_state_get_feedback()` 获取整组已加锁的六轴反馈，检查反馈有效性和超时。输出默认关断。
- Monitor 通过 `uxTaskGetStackHighWaterMark()` 读取剩余栈水位。

## 每次必须记录

| 测试 | 判据 | 本次结果 |
|---|---|---|
| ARM 编译 | Stage4 OFF / TASKS ON / OUTPUT ON 三种配置均构建 | 待测 |
| QEMU 纯算法测试 | `g_stage4_test_finished=1`, `failed=0` | 待测 |
| Host 原生算法 | 4个 Stage4 CTest 全通过 | 待测 |
| Stage2 Host 回归 | 4个既有 CTest 全通过 | 待测 |
| FreeRTOS 任务启动 | 5个任务均存在、无启动失败 | 待测 |
| Stack High Water Mark | 无溢出，记录每任务峰值余量 | 待测 |
| FreeRTOS 队列 | 新请求覆盖旧请求，Target 队列不会积压 | 待测 |
| 反馈过期 | 停止生成新 Motor Target；fault 计数上升 | 待测 |
| 控制周期抖动 | 采集执行时间、唤醒间隔、P99 和最大值 | 尚无采样器，待扩展 |
| IK路径故障 | 无解/近奇异/限位时报告拒绝且不继续指令 | 待测 |
| 闭环 PyBullet | 位置 MAE、RMSE、超调、稳态误差 | 待测 |
| 长时间 | 至少30分钟仿真无异常/泄漏/栈溢出 | 待测 |

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
