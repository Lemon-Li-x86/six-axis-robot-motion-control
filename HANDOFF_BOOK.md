# HANDOFF_BOOK｜六轴工业机器人项目持续开发交接手册

> **先读本文件，再操作代码。** 本文件专为跨对话、跨开发者和跨工作树交接准备。它是**截至 2026-11-01 的已核实快照和操作规则**，不是自动更新的实时状态。新会话必须重新检查 Git HEAD、PR、CI 和本机测试证据，不能机械复述本页的历史结论。
>
> 最新操作入口：[README](README.md) · [Stage5 Windows 操作员验收](docs/part5/18_stage5_windows_final_two_step_acceptance.md) · [Stage5 原始要求审计](docs/part5/19_stage5_original_requirements_evidence_review_2026-11-01.md) · [参数登记簿](docs/part5/01_parameter_register.md) · [Stage5 Issue Log](docs/part5/02_integration_issue_log.md)。

> **场景变更（2026-11-01，注意历史数值已过期）**：当前 `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json` 的 `blocked` 障碍物从 2cm 边长放大到 **4cm**（`half_extents_m=[0.02,0.02,0.02]`）。历史 2cm 场景的 **28/28** 是旧提交证据，不是新的固定段数。[首次4cm ARM C APF 完整路线测试](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41311516679) 实际 **30段**、`route_complete=true` 且全连杆门禁通过；该旧CI有测试曾写死2cm而失败，相关测试已按场景 JSON 调整。短程 C APF 诊断应从配置的第一段 `via` 开始，而非绕过场景定义直接朝最后终点规划；若未通过安全门禁必须拒绝动作。后续同提交跨平台完整 CI 状态需重新核查。保留原来的真实物理日志，不修改 PID、APF安全净距或固件限位。

## 0. 给接手者的十条硬约束

1. **先检查仓库状态，后提修改方案。** 读取本书与 README，获取当前目标分支 `git rev-parse HEAD`、`git status --short`、PR 变更与该提交的 CI；不从聊天记忆猜代码状态。
2. **历史归档不等于当前生产代码。** Stage1–4 的历史文档和 Stage4 独立备份分支保留原样；Stage5 工作树复用并继续修复了 `pid.c` 等共有模块，不能把两套 SHA 的实测相互挪用。
3. **用户已有 Windows 仓库、Python 包和测试脚本。** 优先使用 `tools/run_stage5_embedded_apf_windows.ps1` 与 `tools/run_stage5_live_windows.ps1`；禁止无必要地重新拼几十行启动脚本。
4. **不要擅自更改用户环境。** 不自行 `pip install`、重建 `.venv`、升级 NumPy/PyBullet、清理 `build/`、删除 ZIP 或旧日志。
5. **参数一处定义。** 不在 GUI、测试入口或功能文件另造 PID/周期/速度/关节限位等默认值；先查各自的唯一权威来源（第 6 节）。
6. **绝不为了 CI 变绿放宽安全门禁。** 不能调宽关节到位误差、碰撞净距、超时或关闭真实反馈；禁止在未经分析时盲改 PID 增益。
7. **严格区分几个“成功”。** UART 请求 ACK ≠ 轨迹完成；初始 APF 提案 ≠ 物理运动许可；PyBullet 仿真 PASS ≠ 真机安全认证；GitHub CI PASS ≠ Windows 本机人工签收。
8. **仅凭有效运行工件下结论。** 优先 `result.json`、`worker.log`、`joint_trace.csv`、`uart_live.jsonl`、ELF/map/GDB 原始输出。外层 `Physics worker exited 1` 只表示工作进程退出，不自动等于 PyBullet 崩溃。
9. **每次改动明确模块所有权并加回归。** 算法不调用 UART；通信不运行复杂 IK；Trajectory 不定义新 PID；Control 不复制模型数据；应用层只负责编排。
10. **失败保留证据，绝不“伪通过”。** Git 无法安全切换、依赖缺失、首航点卡死、GUI 断开、任意碰撞或计时不匹配时停止并留存现有日志。

## 1. 当前权威状态与代码谱系

### 1.1 项目范围

- 工程：`Lemon-Li-x86/six-axis-robot-motion-control`；默认分支为 `main`。
- 目标：ARM Cortex-M4 QEMU + FreeRTOS 固件执行 C 运动学、规划、控制、UART；PyBullet UR5 为受控软件仿真的唯一物理关节反馈源。
- **没有实体电机/机械臂的硬件安全认证。** 仿真时 QEMU 使用 TCP 作为虚拟 UART 的主机承载，不等于真实硬件网络控制架构。

### 1.2 本次交接核实结果（2026-11-01）

| 对象 | 核实结果 | 注意 |
|---|---|---|
| 正在修改的分支 | `fix/stage5-verdict-docs-20261101` | 不要操作 `main` |
| 交接文档创建**前**的 HEAD | `5dcec157565823cde6b65802b9e558588787dc02` | 创建/编辑本书以后 HEAD 会变；必须重新查询 |
| 代码/控制修复基线 | `23d1faa9561df07eb2dbaec472ddbf85852896d4` | 包含 PID 偏置修复、诊断时间同步和测试 |
| [PR #37](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/37) | **open + Draft，未合并**；base `feat/stage5-rtos-apf-closure-20261101` | 原有 PR #35、#36 也仍 Draft；不可声称已进入主线 |
| [Stage5 CI：5dcec15](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41305051218) | **completed / success** | 这是 README 更新后、创建本书前的 CI 记录 |
| [Native firmware CI：5dcec15](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41305055339) | **completed / success** | 与同 SHA 对齐 |
| [Stage5 控制修复 CI：23d1faa](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41299105613) | **13/13 Jobs PASS** | 已含 Linux/Windows 集成、UART/ARM APF、GUI 回归 |
| Windows 本机最终验收 | **尚未在当前控制修复提交取得新的完整 PASS 记录** | 需要用户本机按当前 JSON 动态规划完整路线（现4cm CI为30段）+ 人工 GUI 五次连续回位 |
| [Issue #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34) | 2026-11-01 检索时仍 **open** | 旧 `six_axis` 首段不收敛历史 Issue；现有 CI 修复成功≠Issue 管理状态已关闭 |
| Stage6 | 尚未实现/验收最终搬运作业 | 不要把 Stage5 全绿等同整个项目完工 |

**CI 结果只属于列出的源代码 SHA**。新建本文件会触发新的工作流；即使只是 Markdown 变动，也应记录“本次提交 CI 状态”，不偷换为历史绿色状态。

### 1.3 之前已经归档的是什么

- **Stage1–2**：设计、驱动、协议、Ring Buffer、UART、MotorState、QEMU/Host 测试；文档在 `docs/part1/`、`docs/part2/`。
- **Stage3**：UR5 FK/IK、解析解、选解与角度限位；测试和对照报告在 `docs/part3/`。
- **Stage4**：多项式/梯形轨迹、Cartesian 轨迹、PID、FreeRTOS 控制任务及基线回归；`docs/part4/part4_archive/` 保存历史设计、调参、RC 报告。
- Stage1–4 交付与保留边界见 [汇总验收记录](docs/phase1-4_acceptance_status_2026-10-27.md)。它们是相应日期/版本的验收，不覆盖后续修改。
- 保留的 **Stage4 验证备份分支**：`backup/stage4-verified-2026-10-27`，SHA `1ee9ce02d6954e8e9ad4e095611dfd78f0795ab9`。
- 保留的 **Stage4 RC 分支**：`release/stage4-simulation-rc-20261028`（本次查询 HEAD `48e08c32b6a816014ff9e1b67462150f35d0a313`），对应 [RC 文档](docs/part4/part4_archive/12_stage4_release_candidate_2026-10-28.md)。**RC 不等于正式发布 Tag 或无保留签收**。
- 归档方式是**独立 Git 分支 + 历史文档**，不是把整套生产 C 源文件挪走。当前分支的 `kinematics/`、`trajectory/`、`drivers/` 等仍是真正被链接、被后续 Stage5 复用的生产代码。

## 2. 仓库地图：已完成基础、Stage5 功能和当前修改

**标记定义：** `[B]` = 既有基础/历史阶段实现，本次 PR #37 未改；`[S5]` = 既有 Stage5 功能，本次 PR #37 未改；`[M]` = PR #37 相对 base 实际修改的文件；`[+]` = PR #37 相对 base 新增文件；`[H]` = 历史归档文档。**“未改”不是“不允许修改”，而是改动须另外说明理由和测试。**

```text
six-axis-robot-motion-control/
├── README.md                                  [M] 项目入口、状态与操作
├── HANDOFF_BOOK.md                            [交接书] 先读这里
├── CMakeLists.txt / CMakePresets.json          [B] 总构建及 Stage5 ARM preset
├── cmake/arm-none-eabi-gcc.cmake              [B] ARM 工具链
├── requirements.txt                           [B] 新环境依赖声明（非已有环境升级令）
│
├── firmware/
│   ├── hello/                                 [B] 早期 Cortex-M4 启动示例
│   └── freertos_demo/
│       ├── main.c, board.c, startup.s          [B] 固件启动
│       ├── linker.ld, FreeRTOSConfig.h         [B] ARM 映像和调度配置
│       ├── common/                             [B] Stage1–2 类型与单位
│       │   ├── robot_types.h, error_code.h
│       │   └── angle_utils.c/.h, math_constants.h
│       ├── drivers/                            [B] Stage2
│       │   ├── uart_driver.c/.h
│       │   ├── timer_driver.c/.h
│       │   ├── gpio_driver.c/.h
│       │   └── motor_driver.c/.h
│       ├── communications/                     [B/M] Stage2 协议
│       │   ├── ring_buffer.c/.h                [B]
│       │   ├── protocol.c                      [B]
│       │   └── protocol.h                      [M] Stage5 只读诊断选择器
│       ├── model/                              [B] Stage3 本体模型
│       │   └── robot_model.h, ur5_model.c
│       ├── algorithms/
│       │   ├── math/                           [B] matrix4.c/.h
│       │   ├── kinematics/                     [B] Stage3
│       │   │   ├── kinematics_forward.c
│       │   │   ├── kinematics_inverse.c
│       │   │   ├── ur5_analytic_ik.c
│       │   │   └── kinematics_common.c + 各种接口/内部头文件
│       │   ├── motion/                         [B] Stage3
│       │   │   ├── ik_solution_selector.c/.h
│       │   │   └── motion_limits.c/.h
│       │   ├── trajectory/                     [B] Stage4
│       │   │   ├── polynomial_trajectory.c
│       │   │   ├── trapezoidal_profile.c
│       │   │   ├── cartesian_line.c / cartesian_arc.c
│       │   │   └── cartesian_waypoint.c, cartesian_common.c, 接口头
│       │   ├── control/                        [B/M] Stage4 → Stage5
│       │   │   ├── pid.c                       [M] 零 Ki 增量 PID 稳态偏置修复
│       │   │   ├── pid.h                       [B]
│       │   │   ├── joint_controller.c/.h       [B]
│       │   │   └── joint_command_limiter.c/.h  [B] 速度/加速度与量化限幅
│       │   └── avoidance/                      [S5] Stage5 APF
│       │       ├── potential_field.c/.h
│       │       ├── potential_field_path.c/.h
│       │       └── stage5_apf_joint_path.c/.h
│       ├── application/
│       │   ├── motor_state.c/.h                 [B] 已同步的运动状态
│       │   ├── protocol_tasks.c/.h             [B] UART 任务
│       │   ├── uart_tx_manager.c/.h            [B] 发送管理
│       │   ├── runtime_config.c/.h             [B] 运行时配置
│       │   ├── services/
│       │   │   ├── motion_pipeline.c/.h        [M] 只读诊断/追踪发布
│       │   │   ├── protocol_commands.c         [M] 新诊断查询
│       │   │   ├── protocol_commands.h         [B] 协议服务接口
│       │   │   ├── stage5_apf_proposal.c/.h    [S5] ARM 内 C APF/IK 提案
│       │   │   └── trajectory_request_adapter.c/.h [B]
│       │   └── tasks/
│       │       ├── trajectory_task.c           [S5] ARM 实际规划任务
│       │       ├── control_task.c              [M] 状态快照追踪
│       │       ├── monitor_task.c              [B] FreeRTOS 监控
│       │       ├── motion_tasks_internal.h     [M] 只读追踪字段
│       │       └── stage4_timing_probe.c/.h    [B]
│       ├── diagnostics/
│       │   ├── performance_monitor.c/.h        [B]
│       │   └── stage5_cpu_trace.c/.h           [S5]
│       ├── config/
│       │   ├── robot_defaults.json            [B] 统一默认参数源
│       │   └── stage4_motion_config.c/.h       [B] 生成/读取
│       └── tests/
│           ├── unity/, host/, qemu_target/      [B] 协议/驱动/RTOS
│           ├── kinematics/, motion/, model/     [B] Stage3
│           ├── trajectory/                     [B/M] Stage4 + 本轮 PID 回归
│           │   └── cartesian_control_test_main.c [M]
│           ├── simulation/
│           │   └── stage4_sim_bridge_smoke.c   [M] 生产 C 桥接回归
│           ├── avoidance/                      [S5] APF/几何测试
│           └── feedback_clock/                 [B] 反馈时序测试
│
├── simulation/
│   ├── uart_bridge/                            [B] Stage1–2 编解码/通信回归
│   │   └── protocol_codec.py, uart_receiver.py, ur5_uart_bridge.py, ...
│   └── pybullet_ur5/
│       ├── models/ur5/urdf/ur5.urdf             [B] UR5 URDF
│       ├── models/ur5/meshes/                  [B] 模型可视化/碰撞资源
│       ├── scenarios/stage5_static_obstacles.json [S5] 静态测试场景
│       ├── src/
│       │   ├── stage4_dynamic_closed_loop.py   [B] Stage4 动力学基线
│       │   ├── qemu_clock_gate.py              [S5] QMP stop/cont 物理批次门控
│       │   ├── qemu_uart_receiver.py           [M] Stage5 诊断帧解析
│       │   ├── stage5_qemu_uart_closed_loop.py [S5] UART 端到端
│       │   ├── stage5_qemu_uart_visual_demo.py [M] 物理执行/诊断时间戳
│       │   ├── stage5_qemu_uart_live_gui.py    [S5] 原生 GUI 工作进程
│       │   ├── stage5_safe_gui_jog.py          [S5] 人工点动约束
│       │   ├── stage5_collision_validation.py [S5] 真实 URDF 全连杆门禁
│       │   ├── stage5_avoidance_preflight.py   [S5]
│       │   └── stage5_obstacle_hold_recovery.py [S5]
│       └── tests/                             [S5] 几何/规划边界单测
│
├── tools/
│   ├── generate_firmware_defaults.py          [B] JSON → C 默认配置
│   ├── test_stage4_once.ps1                   [B] 历史阶段回归入口
│   ├── test_stage4_closure.ps1                [B] Stage4 综合回归
│   ├── run_stage5_embedded_apf_windows.ps1    [M] ARM APF 整路本机验收（段数由场景决定）
│   ├── run_stage5_live_windows.ps1            [M] GUI 人工点动验收
│   ├── run_stage5_controlled_recovery_windows.ps1 [S5] 受控恢复
│   ├── resolve_stage5_python.ps1              [+] 既有解释器检查
│   ├── stage5_existing_python.py              [+] 旧包 CP311 显式挂载
│   ├── run_stage5_first_leg_native_bullet_windows.ps1 [+]
│   ├── check_stage5_safe_gui_acceptance.py    [S5] GUI FAIL-closed 审计
│   └── tests/
│       ├── run_stage5_embedded_apf_uart.py     [M] 真实 ARM/APF/UART 执行
│       ├── run_stage5_first_leg_native_bullet.py [+]
│       ├── test_stage5_embedded_apf_runtime.py [M]
│       ├── test_stage5_visual_demo.py         [M] 时钟一致性回归
│       ├── test_stage5_existing_python.py     [+]
│       └── test_readme_links.py               [+] README 链接检查
│
├── docs/
│   ├── part1/, part2/, part3/                 [H] Stage1–3 历史报告
│   ├── part4/part4_archive/                  [H] Stage4 参数、调优、RC、性能
│   ├── part5/
│   │   ├── 01_parameter_register.md          [S5] 参数所有权（有早期快照段）
│   │   ├── 02_integration_issue_log.md       [M] 问题台账（旧条目可能过时）
│   │   ├── 18_stage5_windows_final_two_step_acceptance.md [M]
│   │   └── 19_stage5_original_requirements_evidence_review_2026-11-01.md [S5]
│   ├── part6/00_stage6_interface_and_acceptance_contract.md [规划，未完成]
│   └── phase1-4_acceptance_status_2026-10-27.md [H]
│
└── .github/workflows/
    ├── native-motion-regression.yml          [B] 原生固件 CI
    ├── stage4-*.yml                           [B] Stage4 历史回归
    └── stage5-live-gui.yml                    [M] 跨平台 Stage5 集成 CI
```

以上是**真实路径的主要工作视图**，不是 `git ls-tree` 全部文件的逐字转储。要得到完整文件清单，运行 `git ls-files`。新增或修改状态在未来合并后可能变化，届时按 PR 当前 base 重新计算。

## 3. 核心调用链与“到底谁负责什么”

### 路径 A：GUI 原生操作（主机 C APF 安全预检）

```text
GUI/静态场景 → 宿主同源 C APF/IK 候选
             → PyBullet 全连杆离散扫掠检查（许可）
             → UART 独立运动请求/ACK
             → ARM FreeRTOS Trajectory + Control / PID / limiter
             → UART 指令 → PyBullet UR5 关节物理反馈
             → 连续满足到位判据后 READY/HOLD
             → 人工 SAFE JOG → 独立安全批准及真实 C/UART 回位
```

### 路径 B：固件内 C APF 完整路线（旧2cm 28段，新4cm 30段）

```text
PyBullet 当前真实六轴反馈 + 静态 AABB / 目标
         → UART 0x05 请求
         → ARM ProtocolRX 解析/入队
         → ARM Trajectory Task 中生产 C APF + FK/IK/选解
         → UART 0x88 返回提案；此时尚未运动
         → PyBullet 基于 URDF 的所有连杆扫掠检查
         → 安全通过后 UART 0x03 独立运动请求
         → ARM C 轨迹 / 增量 PID / 限幅
         → UART 目标角 → PyBullet 物理积分 / 实测反馈
         → 到位并安全才可请求下一段；直至整路完成
```

- 只有**路径 B**能够证明 APF/IK 的候选规划**真的在 ARM FreeRTOS 轨迹任务内执行**；GUI 路径 A 不能充当此证据。
- C APF 是局部路径候选，场景含高层 `via` 途经目标；完整碰撞许可在 PyBullet 仿真端。不要声称固件可独立处理任意动态障碍、连续碰撞和全部奇异姿态。
- `qemu_clock_gate.py` 的 QMP stop/cont 用于让 QEMU 虚拟时间在耗时物理批次期间停止；不能据此声称实物控制周期精度。
- `stage5_qemu_uart_visual_demo.py` 的 `0.15°` 连续到位条件和时间比例门禁属于验收逻辑；协议 `ACK` 本身不是到位证明。

## 4. PR #37 正在改什么：冻结范围、问题、证据

以 2026-11-01 创建本书前的 PR #37 与 base 分支 diff 为准：**26 个路径变更**，集中在下面几组，而非“重写整个机器人项目”。

| 工作流 | 受影响文件（PR #37） | 已确认目的 |
|---|---|---|
| 固件稳态 PID 修复 | `algorithms/control/pid.c`、`tests/trajectory/cartesian_control_test_main.c` | 在 `Ki=0`、误差连续三次相同情况下，将增量式输出重新对齐已有 P 项；避免饱和历史留下小幅或错向偏置；不改变增益/输出硬上限 |
| 内部状态只读追踪 | `application/tasks/control_task.c`、`motion_tasks_internal.h`、`application/services/motion_pipeline.c/.h`、`protocol_commands.c`、`communications/protocol.h` | 查询参考、反馈、PID 修正、限幅与命令等状态，帮助定位 Windows 0.2° 左右的历史停滞；不是第二套控制算法 |
| 仿真 UART 与时间校验 | `simulation/pybullet_ur5/src/qemu_uart_receiver.py`、`stage5_qemu_uart_visual_demo.py`、`tools/tests/test_stage5_visual_demo.py` | 每个 RTOS 诊断响应与**当时**的物理帧计数配对；不再用早读到的轨迹采样计数除以诊断队列结束后的新帧数 |
| Windows 自动路线与环境兼容 | `tools/run_stage5_embedded_apf_windows.ps1`、`run_stage5_live_windows.ps1`、`resolve_stage5_python.ps1`、`stage5_existing_python.py`、`tools/tests/run_stage5_embedded_apf_uart.py` | 复用用户现有 Python 包、构建 ARM ELF、运行 APF 整路、保留可审计证据 |
| Windows 诊断与测试 | `tools/run_stage5_first_leg_native_bullet_windows.ps1`、`tools/tests/run_stage5_first_leg_native_bullet.py`、`test_stage5_embedded_apf_runtime.py`、`test_stage5_existing_python.py`、`firmware/.../tests/simulation/stage4_sim_bridge_smoke.c`、`tools/tests/test_readme_links.py` | 首航点分层诊断、独立 C/仿真回归和文档链接校验 |
| 工作流/文档 | `.github/workflows/stage5-live-gui.yml`、`README.md`、`docs/part5/02_integration_issue_log.md`、`docs/part5/18_stage5_windows_final_two_step_acceptance.md` | Linux/Windows CI、问题登记和真实手动验收说明 |

### 4.1 两类历史故障不要再混淆

1. **时间同步的虚假 FAIL**：诊断 `trajectory_samples` 较早到达，却拿后续所有诊断完成时的 `target_frames` 进行比例比较，RTOS 约 6.90s / PyBullet 最终约 9.37–9.72s 被错算为 71–74%。修复后使用 UART 诊断响应的物理帧同一时刻数据。原先 `CLOCK_ALIGNMENT_MAX_RELATIVE_ERROR=0.25` 未放宽。该故障即使 3/3 段到位也会被错误判 FAIL。
2. **实际末端停滞**：另一些真实运行中，目标与实测差 0.61°/0.97°，而 PID 输出小到约 +0.003°，甚至与目标误差方向相反。为零积分的增量式 PID 增加稳态重对齐处理。不能把这项故障说成纯粹时间统计问题。

现有 CI 核实过该代码基线，**但尚未证明所有 Windows 本地随机时序问题永不复现**；本机验收和重复性测试仍有意义。`pid.c` 属于 Stage4 形成的共享生产代码，而它**在 Stage5 改了**，必须保留新旧 SHA 的实验边界。

### 4.2 保留项（不要涂绿）

- Windows 本机**人工**原生 GUI：先完成当前场景对应的真实 ARM 整路（4cm场景实测30段），然后原始主机 APF 4/4、J6 `+0.5°`、`+2.0°`、另外至少三次合法点动（总计至少五次），每次等待 `RETURN COMPLETE`，最终点击 `SAFE JOG FINISH/EXIT`；记录实际点击数、`safe_gui_jog_refused=0`、物理收敛、零故障。此项目**暂未在当前版本完成**。
- Stage5 原 5.3 的“稳定降低控制周期 jitter”：多轮 QEMU 短窗口 A/B 有改善也有退化，尚无同负载可重复的长期改善证据、实体 WCET 或硬件保证。FLASH 基线 `105256 → 87384 B`、RAM `33796 → 33796 B`，仅可如实说明这些指标。
- [Issue #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34) 仍 open；后续若关闭，应附同 SHA 可复现测试/证据，不要凭一次成功自动关闭。
- **Stage6** 尚未实现：虚拟抓取/放置、原点→抓取→避障→放置→返回、多场景故障、长时运行、交付归档。接口要求见 [Stage6 设计契约](docs/part6/00_stage6_interface_and_acceptance_contract.md)。

## 5. Windows 本机：正确路径、现成入口、证据

### 5.1 路径（用户已提供，不能擅自猜成另一个 worktree）

- 要更新和验收的 Git 工作树：`D:\1_ToGo\Robo\code\stage5-acceptance`。
- **既有 Python 包目录所在的另一工程**：`D:\1_ToGo\Robo\code\six-axis-robot-motion-control\.venv\Lib\site-packages`。
- 现有旧 `.venv\Scripts\python.exe` 可能是 Python 3.10，二进制 PyBullet/NumPy 扩展却是 `cp311`；**不能简单从包目录反推解释器能工作**。
- 仓库的 `tools/stage5_existing_python.py` 明确支持已有 **uv CPython 3.11** + 指定 `-ExistingSitePackages`，不安装/写入包，也不永久更改 `PYTHONPATH`。
- 工具链需已有 ARM GNU `arm-none-eabi-gcc`、CMake、Ninja、`qemu-system-arm`；GUI 需 Windows 桌面显示会话。脚本若发现缺项，停止报告，不做未经授权的系统安装。

### 5.2 最小可复用 PowerShell 流程

以下**使用仓库已有脚本**，在 PowerShell 中运行，不手工重写 CMake/ELF/UART/物理回路。路径来自用户的实际工作树；解释器路径按当前安装环境检查。

```powershell
# 先检查本地工作树，必须确认未保存修改
Set-Location 'D:\1_ToGo\Robo\code\stage5-acceptance'
git status --short
# 有未提交或未跟踪的重要修改时停止，禁止 reset --hard

git fetch origin fix/stage5-verdict-docs-20261101
git switch --detach origin/fix/stage5-verdict-docs-20261101
git log -1 --oneline

# 复用已存在的 CPython 3.11 和原项目 site-packages
$py311 = Join-Path $env:APPDATA 'uv\python\cpython-3.11.15-windows-x86_64-none\python.exe'
$packages = 'D:\1_ToGo\Robo\code\six-axis-robot-motion-control\.venv\Lib\site-packages'
& $py311 .\tools\stage5_existing_python.py --site-packages $packages --probe
if ($LASTEXITCODE -ne 0) { throw 'Existing PyBullet / NumPy import FAIL' }

# 第一关：真正 ARM Cortex-M4/FreeRTOS 中 C APF/IK 的完整场景路线
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_embedded_apf_windows.ps1 `
  -Python $py311 -ExistingSitePackages $packages `
  -Qemu qemu-system-arm -TimeoutSeconds 300 `
  -OutputDir build/stage5-local-full-route

# 第一关确实通过后，再做 GUI 人工签收
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 `
  -Python $py311 -ExistingSitePackages $packages `
  -Qemu qemu-system-arm -Stage5RouteCase blocked `
  -SafeJog -RequireTwoStepJog -SafeJogHoldSeconds 600 `
  -OutputDir build/stage5-local-gui-final
```

**重要：** 上面 `git switch --detach` 是对已核对、无未提交重要修改的 `stage5-acceptance` 工作树。若 Git 拒绝，不通过强制切换继续，先阅读实际差异。**用户不要求创建新 worktree**。每次重新验收建议使用新 `-OutputDir`，保留旧工件。

脚本完成构建和物理回路；这是软件仿真，不会向实体伺服输出。若已有工作树上需要更保守的 Git 同步策略，先 `git status` 并判定是否快进，不假设用户当前分支或 stash 状态。

### 5.3 成功判据与故障证据

- `run_stage5_embedded_apf_windows.ps1`：`result.json` 必须 `passed=true`、`route_complete=true`、已批准航段均真实 ACK + 物理到位、`full_link_safe=true`、`joint_motion_submitted_only_after_gate=true`、`uart_checksum_errors=0`、`runtime_full_link_queries >= 7*joint_motion_acks`。旧2cm场景历史验收为28段；最新4cm场景已在Linux CI完成30段，实际段数随规划结果变化。
- `run_stage5_live_windows.ps1`：必须人工看见原始 4/4 航点完成、按钮 READY、每次独立回位与最终 FINISH；拒绝/未完成的任何一次点动不能获得 PASS。已配置的默认连续稳定收敛判据为 0.15°、15 个反馈帧。
- 失败优先收集：`build/<output>/result.json`、`worker.log`、`worker_exit.json`（如有）、`joint_trace.csv`、`uart_live.jsonl`、`qemu_stderr.log` 和程序终端输出。不能只报告“exit 1”。
- GUI 有 **真实操作** 和 CI 下 **自动脚本点击** 两种不同验证，不得互相冒充。既有脚本可能在 PASS 后生成 ZIP，失败时不能把旧 ZIP 当作新结果。
- 若 Windows 出现“残差约 0.23° 卡住”或新残差，检查最后数十条关节目标/反馈、trace 参考角/PID 修正/限幅输出、diagnostics、RTOS tick、物理帧时间、QMP pause 次数与脚本 SHA；**先定位，不再直接盲改 PID**。

## 6. 模块边界与参数唯一来源

| 数据类别 | 权威来源 | 不得做的事 |
|---|---|---|
| FreeRTOS 任务周期、优先级、栈、启动值、六轴 PID、速度/加速度、反馈超时 | `firmware/freertos_demo/config/robot_defaults.json` + `tools/generate_firmware_defaults.py` | GUI/测试/任务 C 文件再定义一份相同默认值 |
| UR5 模型、DH/关节范围、位姿和关节计算约束 | `firmware/freertos_demo/model/`、生产 C 运动学及加载的 `ur5.urdf`（需保持一致） | Python 另造一份放宽的关节限位 |
| C APF 算法与请求数据结构 | `firmware/freertos_demo/algorithms/avoidance/`、`application/services/stage5_apf_proposal.c` | Python 复制另一种 APF 数值算法并称“同源” |
| 场景障碍、途经点、外部全臂净距/离散扫描预算 | `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json`、`stage5_collision_validation.py` | 把场景写进 FreeRTOS Control Task；将末端点 APF 净距当全臂许可 |
| UART 协议、帧结构、选择器 | `communications/protocol.[ch]`、`simulation/pybullet_ur5/src/qemu_uart_receiver.py` 及协议一致性测试 | 添加同义异名、无一致性验证的 magic number |
| GUI 到位/安全点动和时间一致性判据 | `stage5_qemu_uart_visual_demo.py`、`stage5_safe_gui_jog.py` | 只为使验收 PASS 静默放宽阈值 |
| 实时关节角、反馈新鲜度、故障、PID/限幅诊断 | `motor_state`、`motion_pipeline` 与诊断服务的运行时快照 | 用静态配置变量冒充动态反馈 |

规则：**功能文件不拥有默认参数。** 每次新增值先判断它是机器人固有事实、算法临时变量、运行时测量、场景输入还是验收策略，再确定唯一所有者。详情见 [Stage5 参数登记](docs/part5/01_parameter_register.md)、[架构设计](docs/architecture.md)。

## 7. 下一轮工作计划和停机条件

**当前最近目标不是重新开发 Stage1–4，也不是立即启动 Stage6。**

1. 等当前文档提交对应 GitHub CI 完成，核对是否同提交全绿；记录具体 SHA/Actions URL。
2. 让 Windows `stage5-acceptance` 安全同步到目标工作分支，**复用已存在的 Python 3.11 与旧 site-packages**；不移动用户虚拟环境。
3. 使用 `run_stage5_embedded_apf_windows.ps1` 完整验证 ARM 内真实 C APF 当前场景全部路段，检查 `result.json` 和全连杆门禁。
4. 仅在上述通过后，用 `run_stage5_live_windows.ps1` 进行一次原生 GUI **至少五次合法点动**，依次 J6 +0.5°、+2°、再三次，逐次回位后 FINISH；若 FAIL，保存工件，停止无依据的下一步。
5. 本机 PASS 后更新 Stage5 Issue Log、验收文档、README、本书的 SHA 与签收状态；才考虑 PR #37 是否满足解除 Draft 的条件。需另审控制抖动目标和性能保留项。
6. Stage6 后续应从 [接口约束](docs/part6/00_stage6_interface_and_acceptance_contract.md) 构建搬运任务状态机；绝不可新写 PID/IK/APF 复制件或把 GUI 当硬件控制层。

## 8. 新会话怎么恢复上下文

用户新开 ChatGPT 对话时，提供 GitHub 仓库或本书链接，交给下一轮助手的开场指令可以是：

> 请先完整阅读仓库根目录 `HANDOFF_BOOK.md` 和 `README.md`，并核对 `fix/stage5-verdict-docs-20261101` 当前 HEAD、PR #37、当前 Actions 以及 `docs/part5/18_stage5_windows_final_two_step_acceptance.md`。这是一个已完成 Stage1–4 基础、Stage5 处于 Windows 本机签收阶段的真实 ARM/QEMU/FreeRTOS/PyBullet 项目。必须保留 Stage4 历史备份，严格使用已有 Windows 验收脚本和现有 Python 环境，先比较源文件与文档是否漂移，再继续。不要依赖你对过去聊天的记忆来猜版本。

**交接手册不是自我更新的记忆系统。** 新助手应独立核对 GitHub 和用户本机输出；若本书与实际代码、实测冲突，以**当前 SHA 的代码及可核查运行工件**为事实来源，先纠正文档再决定下一步。

---

### 手册修订记录

- **2026-11-01 初版**：从当前真实 Git 树、PR #37（文档前 HEAD `5dcec15`）、Stage4 归档分支、Stage5 两个 CI 工件、现有 Windows 脚本与验收文档核对写成。包含真实目录与“本轮 PR 修改”标记、本机 Python 扩展兼容、当前未完成事项以及进入下一轮会话的操作规则。
- **下次更新需填写**：新 HEAD SHA、对应 CI URL/结论、Windows `result.json` 路径及结果、GUI 五次完整操作事实、仍 OPEN 的问题、是否允许升级 Stage6。


## 9. 5.3 最新续作（2026-11-01；请以本节的新证据替换上面旧性能快照）

**最新实验事实不改变旧 Stage4 封版，也不能假设本机已经运行。** 细节报告：[5.3 重复性能 A/B 和边界](docs/part5/20_stage5_5_3_repeated_performance_evidence_2026-11-01.md)。

- 修改文件：`application/tasks/monitor_task.c`、`tests/qemu_target/stage4_rtos_selftest.c`；新增 `tools/run_stage5_perf_repeats.sh`、`tools/aggregate_stage5_perf_repeats.py`、`tools/tests/test_stage5_perf_repeats.py`；修改 `.github/workflows/stage5-cpu-trace.yml`。这些是 **Stage5 QEMU 性能测量和自测**，并未更改生产 PID、速度/加速度、UART 格式和关节安全限位。
- 关键测量根因：原 `1200ms` 健康测试结束就冻结 Timer0 抖动，但 CPU 快照继续涵盖后续故障测试。已改为同一健康阶段终点冻结 CPU/Timer0，并在性能 CI 两种配置使用 `ROBOT_STAGE4_QEMU_HEALTHY_RUN_MS=5000`。**计时缓冲仍只保留每任务最近128次**，不能宣称 CPU 和 P99 完全同长度的5秒统计。
- [性能测试成功 Run #41308538374](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308538374)（测量代码 SHA `e5e6dbc92449a06522dec42aafa279223b2325f1`），6 个完整交错 A/B 对；[原始工件](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308538374/artifacts/13084018842)。
- **可签收**：Monitor CPU 6/6 组较低，中位数 `0.05%→0.02%`；当前同提交 FLASH `106464→88600 B`、RAM `33936→33936 B`。之前 `105256→87384 B` 是另一提交的同配置 A/B，不能跨提交对比静态占用。
- **必须保留差距并使用更新的两轮数据**：[第二轮严格验证器 CI #41309323797](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41309323797) 也 PASS，但 Control/Trajectory 抖动在其6对中只有各3对改善。**合计12对：Monitor CPU 12/12下降（0.05%→0.02%）；Control/Trajectory P99 抖动各8/12改善、4/12退化；Monitor P99 10/12退化。** 综合原始配对绝对抖动中位数 Control `0.02232→0.02162ms`、Trajectory `0.03260→0.02732ms`、Monitor `0.03322→0.04994ms`。不能只引用第一轮5/6，不能写“所有任务抖动持续下降”“已证明硬件 WCET”或“5.3 无保留全部完成”。详细对照在第20号5.3报告。
- **接下来如果继续做 5.3**：先以新工件验证每组实际控制负载和 p99 对照，再决定是否实施独立的调度/ISR 级改动；不能仅凭某一组数据直接改变优先级、核心控制算法、QEMU Tick 频率或宽松化通过条件。新的修改要先立可复现的因果假设并独立跑同提交负载 A/B。
- 新会话应查看当前 PR #37 HEAD/CI。此节中的 `e5e6dbc9` 是**实验提交标识**，不是保证永远有效的分支 HEAD。
