# 第四阶段仿真版本封版验收单（RC，2026-10-28）

> 状态：**封版候选已建立，尚未签署最终 PASS，也未打正式发布 Tag。**
> 范围：按原《六轴工业机器人嵌入式运动控制固件开发与仿真验证》大纲第4阶段，交付 **Cortex-M4/FreeRTOS/QEMU + UR5/PyBullet 的软件仿真成果**。项目无实体机器人，本记录不声称实物安全、实体伺服精度或硬件 WCET。

## 1. 基线与冻结规则

- 本封版工作分支：`release/stage4-simulation-rc-20261028`，**从候选源码提交 `9fb22d93685f464e455194ff66e816fc671296a8` 派生**。
- 原 Stage4 历史快照：`backup/stage4-verified-2026-10-27`，原 SHA `1ee9ce02d6954e8e9ad4e095611dfd78f0795ab9`；不覆盖、不移动。
- 当前六轴仿真候选 PID 的**唯一生产配置来源**：`firmware/freertos_demo/config/robot_defaults.json`，`Kp=0.25, Ki=0, Kd=0.025`（每轴相同）；不能另建一份 PID 硬编码。
- 本 RC 封版期**只允许**证据核对、文档索引、可复现性修复和确有测试证据的阻塞性修复。未经独立复核不改 PID 算法、控制任务、URDF 几何、协议、限位或生产默认参数。
- Stage5 的 Windows 原生鼠标拖拽 / GUI HOLD / UART 时间同步修复位于另一分支及 [PR #3](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/3)；**不混入此 RC**。除非有单独决策及证据，不把第5阶段集成工作反向当作第4阶段结项门槛。

## 2. 原大纲四项核心任务逐项核验

| 大纲第4阶段核心任务 | 权威实现与验收依据 | 本 RC 判断 |
|---|---|---|
| 关节空间规划：三次/五次多项式、梯形速度与配置化速度/加速度约束 | `algorithms/trajectory/polynomial_trajectory.c`、`trapezoidal_profile.c`；整段极值预检；`06_stage4_requirements_verification_2026-10-27.md`、`09_polynomial_preflight_and_json_config_2026-10-27.md` | **核心实现完成、已有 Host/ARM 回归** |
| 笛卡尔规划：直线/圆弧、IK 选解、插补周期 | `algorithms/trajectory/cartesian_line.c`、`cartesian_arc.c`、`cartesian_waypoint.c`；`trajectory_task.c`；Host/QEMU IK 与周期回归 | **算法实现完成、已覆盖指定样本**；全空间可达性、整机连续避障不在本次声明内 |
| 增量式 PID：独立 P/I/D、抗饱和、限幅、死区及仿真调参 | `algorithms/control/pid.c`、`joint_controller.c`、`joint_command_limiter.c`；`03_pid_tuning_record.md`、`10_stage4_pid_sensitivity_study.md`、`11_stage4_pid_candidate_verification_2026-10-27.md` | **实现及受控仿真调参完成**；保留多工况性能边界 |
| FreeRTOS 多任务：规划、控制、通信、监测，优先级、周期、堆栈、同步 | `application/tasks/`、`application/services/motion_pipeline.c`、`config/robot_defaults.json`；`04_realtime_verification.md`、`08_stage4_qemu_timing_and_stack_2026-10-27.md` | **仿真任务调度及短时计时证据完成**；未取得实体硬实时证明 |

三份阶段产出对应：
1. **路径规划与运动控制模块完整源码**：`firmware/freertos_demo/algorithms/{trajectory,control}/`、`application/{tasks,services}/`，以及模型、运动约束和公开接口。第4阶段生产代码已存在，RC 不另复制一套。
2. **PID 参数调优记录与轨迹性能分析文档**：`03_pid_tuning_record.md`（历史基线）、`10_stage4_pid_sensitivity_study.md`（实验数据、定义及异常）、`11_stage4_pid_candidate_verification_2026-10-27.md`（候选配置和交叉复核）。
3. **多任务调度设计与实时性验证报告**：`04_realtime_verification.md`（任务与判据）、`08_stage4_qemu_timing_and_stack_2026-10-27.md`（实际 QEMU 观测及限制）。

上述是**文档与源码清单**，不意味着所有附件已在用户机器复验。

## 3. 已核实的云端证据（对应候选源码的真实运行）

候选提交 `9fb22d9` 的 [GitHub Actions #40498695392](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40498695392) 三项 Job **均 success**：

| 验证 | 实际结果与边界 |
|---|---|
| Linux Stage4 Host CTest / C 桥接 Smoke | Host **6/6 PASS**，C bridge **1/1 PASS**，生产 C 模块参与测试 |
| 18 组 C/PyBullet DIRECT 灵敏度矩阵 | **18/18 无异常执行**；原小位移门限的探索性跨工况复用仅 **16/18 满足**。剩余 2 组不是崩溃，不能写成“18/18 性能门限通过” |
| 独立标定/保留样本搜索 | 60 组 calibration、20 组 held-out validation 试验；候选 `Kp=0.25, Ki=0, Kd=0.025` 的六项被评估条件符合本次软件试验判据，最坏门限比约 **0.76733** |
| Linux Cortex-M4 QEMU UART ↔ PyBullet | 单次请求最大最终误差 **0.02°**；三段真实双向 UART **3/3 到位**，最大分段最终误差 **0.08°**，该轮 `passed=true`；此项是额外集成回归证据，不改变大纲对 Stage4/Stage5 的分界 |
| Windows CI Host | 6/6 Host CTest、1/1 桥接 Smoke 通过；该 Job 使用 `-HostBuildOnly`，**不证明 Windows 本地 PyBullet/QEMU 全套完成** |

另有独立的 ARM/QEMU FreeRTOS [60 秒仿真健康基线 #40446109227](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40446109227)：`finished/checks/failed=1/29/0`，Monitor/Control/Trajectory 均在仿真虚拟时间内运行。详见 `08_stage4_qemu_timing_and_stack_2026-10-27.md`，不是本候选 SHA 的重新跑测，不应混写成同一次运行。

### 实时性数据如何表述

`08_stage4_qemu_timing_and_stack_2026-10-27.md` 记录短时 QEMU 样本：Control 10ms、Trajectory 20ms、Monitor 100ms；对应 P99 绝对周期偏差分别为 **54.88 μs / 230.44 μs / 13.08 μs**，历史 Control 最长测得处理经过时间 **2081.32 μs**；Control / Trajectory / Monitor 观测最小剩余栈 **375 / 609 / 349 words**。这些是**被测样本的 QEMU 经过时间与栈统计**，不是 CPU 独占占比，更不是物理 MCU 的 WCET。60 秒虚拟时间基线是周期性静态反馈，不代表30分钟高负载稳定性。

## 4. 本机最终签收门槛（G2 核心回归已有用户实测证据）

在 Windows 仓库根目录使用明确的 RC 版本，不强行切换存在未提交改动的工作树：

```powershell
git status --short
git fetch origin
git switch release/stage4-simulation-rc-20261028
git pull --ff-only
git rev-parse HEAD
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
```

- [x] **G1：源码版本确认。** 用户本机在成功执行测试后报告 `git status --short` **无输出（工作树干净）**，`git rev-parse HEAD` = **`6cedeff99fac67b6aa4a25ccaa18909722164ccc`**；与对应首次 RC CI 源码 SHA 一致。GitHub compare 确认此 SHA 为候选基线 `9fb22d93685f464e455194ff66e816fc671296a8` 的后代（ahead 5、behind 0），两者差异只有 CI 工作流、README 和 Stage4 文档，没有生产代码修改。**本机测试归属 6cedeff，不得把后续仅文档修订的 HEAD 写成同一实际被测 SHA。**
- [x] **G2：Windows 本机 Stage4 核心回归。** 2026-10-28 用户实际在 RC 工作树运行封版脚本，返回 PID 两份 JSON 摘要和 `build/stage4-final-validation.log` 末40行，确认 Host 6/6、Bridge Smoke 1/1、18组全部执行、60/20 扫参、FreeRTOS 29/29 和多项式 8/8、Stage4 DIRECT 双场景及 `[PASS] Automated Stage3/4 non-actuating regression complete`。**G1 已补全本地 HEAD 与工作树状态**；报告引用的是用户本机复制的运行结果，尚未将完整本机 log 原件归档。
- [x] **G3：性能异常已审阅并记录为探索性边界。** 本机 18/18 执行成功、16/18 通过跨工况沿用的小幅测试门限；未达的是 six_axis_medium、60 N·m 条件下 *PI experimental*（J2 超调 0.55°）和 *PID experimental*（J2 超调 0.38°），均高于小幅测试的 0.15° 超调门限。二者不是生产默认 PID 候选的独立六工况验收失败。用户提供扫参摘要：最佳候选六项符合本次仿真试验门限，最坏比 0.7673293597209491。保留 CSV/JSON 为证据，本次不改生产控制增益。
- [ ] **G4：交付物与原始证据归档。** 三项阶段产出已有明确文件索引；本 RC 已重写项目根 `README.md`（Stage1–Stage4 逐阶段进展、结构、运行、性能边界及 Stage5 当前范围），同步更新 `docs/architecture.md`，纠正旧版“只有通信任务、Control Task 尚未实现”的过时声明；旧 PID 测试数据仍可追溯。**尚未完成**：保存用户本机 `build/` 下完整 JSON/CSV/GDB 日志到正式交付介质，及最终文档审阅签收。不能因 README 更新而直接将 G4 标为完成。
- [ ] **G5：最终签收与封存。** G1–G4 完成后才将此单状态改为“正式封版”，保存签收日期、**最终源码 SHA**、验收记录位置及例外；再考虑创建不可移动 Tag。此 RC **没有打 Tag、没有合并 Stage5 PR**。

可选额外集成观测：`-IncludeGuiReplay` 会重新构建 QEMU UART 固件并执行**独立轨迹录制和 GUI 回放**；它不是 PyBullet 原生鼠标拾取，不属于本次 Stage4 基础封版的硬性门槛。


### 2026-10-28 Windows 本机 G1 / G2 / G3 签收证据（用户现场输出）

- RC checkout 到 `release/stage4-simulation-rc-20261028`，本机 Visual Studio 18 2026 工具链；本地 `git status --short` 空输出，`git rev-parse HEAD=6cedeff99fac67b6aa4a25ccaa18909722164ccc`。这是本机实际执行时的版本，非更新后的文档 HEAD。
- Stage4 Host CTest：`6/6 Passed`；生产 C 桥接 Smoke：`1/1 Passed`；Python PID 矩阵/搜索定义各 `4/4 ok`。
- 用户提供 `study_summary.json`：`completed_without_exception=18`、`nominal_reference_passed=true`、`exploratory_baseline_threshold_passes=16`、`failures=[]`。**2 个跨工况不达实验门限不可表述成程序执行失败。**
- 用户提供 `sweep_summary.json`：`calibration_trials=60`、`held_out_validation_trials=20`、最佳 `kp_1_ki_0_kd_0.025`、`max_violation_ratio=0.7673293597209491`、`conditions_within_policy=6/6`、`all_six_conditions_within_test_policy=true`。
- 用户本机 `build/stage4-final-validation.log` 尾部显示 `g_stage4_rtos_test_report.checks=29/failed=0`、`g_stage4_polynomial_rtos_report.checks=8/failed=0`、`rejected_before_publish=1`、`valid_execution_started=1`、`trajectory_stack_free_words=467`、`[9/9] PyBullet DIRECT closed-loop dynamic verification` 中 J2 与 six_axis 两场景，最终 `[PASS] Automated Stage3/4 non-actuating regression complete`，终止日期 2026-10-28。
- GDB 同时打印 `could not convert ... CP1252` 与 `Source file is more recent than executable` 警告，未使此次测试失败；需继续遵循 G1 的版本 SHA 与实际构建关联检查，不据警告直接认定用错产物。
- 当前证据源是用户在对话粘贴的文本摘要，而非已经入库的完整本机测试附件；本地完整 JSON/CSV/GDB 日志留在 `build/`，后续交付归档应保存这些原件。

### RC 分支 CI 单独阻塞/例外跟踪

[Actions #40633371737](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40633371737) 针对封版分支 HEAD `6cedeff`：Linux Stage4 Native PID/C Bridge Job **success**，Windows Host Job **success**；Linux QEMU UART Job **failure**。该 Job 中**单次 QEMU UART 运动通过**（最大末端误差0.02°），**额外的 Stage5 双向 UART 物理三段运动在首段超时 210s：0/3**。这是仍需追踪的具体集成回归，不得将整条 workflow 标为成功。

三段实时 QEMU UART 物理闭环在项目原任务书中属于第五阶段系统集成，并非第四阶段的关节/笛卡尔规划、PID 与 FreeRTOS 非驱动验收本体。封版评审应明确决定**把这一集成失败作为 Stage5 独立阻塞项**，或修复其回归后再批准；不能通过修改 CI 测试超时或忽略失败制造绿色。**因此当前维持 RC / draft / 无 Tag**。

## 5. 已知限制与下一阶段分界

- 第4阶段不声明：全工作空间奇异性均已覆盖、不同真实负载的最优 PID、实体电机/安全急停、独占 CPU 占比、硬件级最坏执行时间（WCET）、30 分钟满载连续运行。
- 真实 UART 异常流、高负载系统优化、整臂碰撞与静态避障、完整搬运任务属于大纲第5～6阶段。Stage5 GUI 拖拽不稳定或 CI 虚拟桌面超时的工作项继续在 Stage5 分支追踪，不能覆盖此 RC 的功能验收结论。
- **RC 不等于成品发布**：在 G1–G5 签收前，对外只写“Stage4 仿真核心实现及候选 CI 通过，正式封版待本机验收”。

## 6. 证据导航

- [Stage4 大纲逐项审查](06_stage4_requirements_verification_2026-10-27.md)
- [Stage4 总体回归状态](../phase1-4_acceptance_status_2026-10-27.md)
- [PID 动态基线记录](03_pid_tuning_record.md)
- [18组 PID 灵敏度矩阵](10_stage4_pid_sensitivity_study.md)
- [新 PID 候选及跨工况复验](11_stage4_pid_candidate_verification_2026-10-27.md)
- [FreeRTOS 实时性实测](08_stage4_qemu_timing_and_stack_2026-10-27.md)
- [唯一参数来源登记簿](00_parameter_register.md)
- [CI：候选版本](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40498695392)

记录维护原则：新增事实以**提交 SHA + 测试日志 + 具体计量口径**为准；不以“演示能动”、测试脚本存在或阶段报告的历史措辞代替验收证据。
