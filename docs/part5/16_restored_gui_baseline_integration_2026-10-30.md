# 第五阶段 GUI 恢复基线与增量迁移登记（2026-10-30）

## 1. 不可修改的历史基线

- **Stage4 RC 备份（禁止修改）**：`backup/stage4-rc-passed-20261028-48e08c3`，SHA `48e08c32b6a816014ff9e1b67462150f35d0a313`。
- **用户 2026-10-28 Windows 手工测试后初步认可的 Stage5 GUI 回位版本（禁止修改）**：`backup/stage5-gui-recovery-20261030-7307984`，SHA `7307984ac282d7901817478c167003dd0171658a`。
- 二者属于**不同阶段/验收范围**：Stage4 RC 证明非交互闭环与控制算法基线；7307984 是原生 PyBullet 鼠标拖拽、松手后 HOLD 恢复的用户初步验收版本，并非完整跨平台长期验证。
- 本文档和以下集成工作**只修改** `feat/stage5-gui-preserved-integration-20261030`；不修改备份分支，也不合入 `main`。

## 2. 迁移方式与保护范围

新实验集成提交 `2e7a1a35452cf6773ea217d7862a8e5d6a769195` 使用两个已有祖先：

- 第一父提交：`7307984ac282d7901817478c167003dd0171658a`（可用 GUI 基线）
- 第二父提交：`970a887c1319c080e99cc5b9b5931754698dfbb6`（后续第五阶段主线）

以第二父提交的整套 Stage5 APF/C IK/FreeRTOS/测试源码为基础，**逐个恢复**第一个父提交中的原始 GUI 文件与测试，且其 Git blob SHA 与 7307984 完全相同：

| 被保护文件 | 7307984 原始 blob SHA |
|---|---|
| `simulation/pybullet_ur5/src/stage5_qemu_uart_visual_demo.py` | `a6d55cc10f4d754ab2c365c9b6e78437f59abb2b` |
| `simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py` | `7bcb6305600907d20b9a591faf845ed655eab647` |
| `tools/tests/test_stage5_visual_demo.py` | `9841a7a8a8cdffc3870611dac5399c326626ccdf` |
| `tools/tests/test_stage5_native_joint_stops.py` | `d5d11e21890d6f0c0e8687e3421729b051527d87` |

同时保留历史 `docs/part5/06_live_uart_gui.md` 与原 GUI CI 细节（仅在**集成分支**上给 CI 增加 PR 触发条件）。

## 3. 新增第五阶段功能

新分支继承主线已有模块，而非重写 GUI HOLD 状态机：
- Cortex-M4 FreeRTOS UART 固件中 C APF、路线生成、UR5 C IK、关节轨迹及已验证避障几何；
- PyBullet 场景配置、C APF preflight、全臂碰撞验证与路线压缩；
- 固件反馈时间戳 / tick 次序修复，以及 Task CPU trace、内存统计和相应回归；
- C APF UART 执行和带障碍路线的独立测试工作流。

**明确集成边界**：新增避障模块和 C APF UART 闭环独立可用，不意味着障碍避让路线已经注入受保护的 Windows 原生 GUI/HOLD 交互演示。完成该接口前，禁止声称鼠标 HOLD + 物理避障同时验收。优先回归确认被恢复 GUI 对新固件仍可正常到位和松手回位，再通过独立、可测试接口连接避障。

## 4. 验收门禁（不得仅凭静态集成声称通过）

1. 版本审计：Stage4 与 GUI 备份指针不变；四个受保护 GUI/测试源码 blob SHA 不变。
2. Windows/Linux GUI 模块测试与真实 QEMU/FreeRTOS UART observer。
3. 真实 PyBullet URDF 限位、固件轨迹、三段到位、Linux native no-drag GUI。
4. C APF/IK 阻挡场景、全臂安全距离、长路线与反馈时间顺序。
5. Windows 本机原生 GUI：三段自动运动 → 鼠标拖拽 J1/J2/J3 → 松开 → **C 固件输出继续且最终实际关节回到该次 HOLD 合法连续目标**；记录 worker.log、joint_trace.csv、result.json 中的目标和实测、规划错误计数。不得用单纯 ACK、单元测试、回放或 `passed=true` 代替物理松手验收。

在完成并复核上述门禁前保留 **Draft PR**，不合并 `main`；任何失败都应定位实测证据，不降低原阈值、不改变 Stage4 参数或在 GUI 外添加假回位。

## 5. 2026-10-30 兼容修复与人工测试入口

本分支最初的 `2e7a1a3` 合并提交包含 7307984 GUI 的字节级原版。**请不要把该事实误写成此集成分支后续文件始终完全相同**：为了补上第五阶段功能，本分支随后通过增量补丁修改了 `stage5_qemu_uart_visual_demo.py`、`stage5_qemu_uart_live_gui.py` 和 Windows 启动脚本；历史 7307984 原版仍可在备份分支直接按 blob SHA 校验，未动。原来已验证的连续角度解绕、关节限位、HOLD 触发与重规划函数未改。

追加的变更及验收：
- 为原 GUI worker 添加**可选** `--stage5-c-apf-library`、`--stage5-route-case`、`--stage5-raw-route`，采用原主线经历史 CI 验证的 C APF + C IK 路线预检与物理整臂逐帧碰撞门禁，缺省不改原六轴三段工作流；
- APF 短路径用 `monitoring_ticks` 检查全程时钟对应，普通三段运动仍用 `trajectory_samples`；
- 新增 Windows DLL 导出保护（`WINDOWS_EXPORT_ALL_SYMBOLS`），无第二份 C 避障算法。Windows 启动脚本 `-Stage5RouteCase blocked` 会生成 DLL、运行 CTest，然后启动真实原生 GUI；
- PyBullet 窗口被按 X 断开时仍返回 NOT PASS，且不再双重 disconnect。人工验收时**按 Q 正常退出**，不要在测量结束前关闭窗口。
- Linux/Windows 新增 CLI 单元测试；完整跨平台 CI 和 Windows 人工拖拽验收仍是门禁。

在 Windows 仓库根目录、切到当前 PR 最新 HEAD 后执行：

```powershell
# 原已初步验证的六轴 3 段运动 + 原生 GUI HOLD
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -DiagnosticsTimeoutSeconds 30 -OutputDir build/stage5-gui-hold-check

# 真实 C APF/IK 带阻挡长方体的静态避障 + UART/FreeRTOS/GUI HOLD
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -DiagnosticsTimeoutSeconds 30 -Stage5RouteCase blocked -OutputDir build/stage5-gui-blocked-check
```

人工验收按：自动运动完成 → 进入 HOLD → 使用原生鼠标拾取慢慢拉开且松开 → 等固件回到合法连续目标 → 按 Q 退出 → 查看各自目录 `result.json`、`worker.log`、`uart_live.jsonl`、`joint_trace.csv`。如关节逼近 URDF 极限，缓慢松开，不以故障停机假冒成功。

这是**Windows 本机 PyBullet/QEMU 仿真实验**，不是实体工业机器人安全认证。

## 6. GUI 六关节单圈试验行程（2026-10-30，待 Windows 鼠标验收）

观察到 J1 底座鼠标拖拽时偶尔可旋转两整圈。经检查，这与 URDF/C authoritative model 的关节 travel 一致：J1/J2/J4/J5/J6 均允许 [-360°, +360°]（合计 720°）；J3 为 [-180°, +180°]。此前 GUI 的关节 guard 只有超出 URDF 边界才复位，并非专门的人机交互单圈限制。

为使 Windows PyBullet 交互测试直观，新增仅用于 **native GUI mouse picking** 的行程模式，且不更改 URDF、C Model、固件规划、PID、UART canonical 协议、DIRECT 验收：
- 默认 `-GuiJointTravel single-turn`：逐个关节使用 `URDF joint limits ∩ [-π,π]`，也就是六轴 [-180°, +180°]；这是 GUI **实验交互窗**，不是改变机械模型的物理行程或硬件安全边界；
- `-GuiJointTravel urdf`：沿用原版 URDF 上的宽行程（J1/J2/J4/J5/J6 各 ±360°，J3 ±180°），用于历史行为对照；
- PyBullet 原生限位和每帧守卫、HOLD 连续角 servo 解绕使用**同一份本次加载后生效的六轴限制**，避免界面允许角与控制目标限制的重复硬编码；读到无穷、非法或没有可行等效角度时拒绝执行，而非静默假限幅。
- 运行开始时 `worker.log` 会打印 `[GUI JOINT LIMITS]` 及 6 个实际加载的范围；`run_metadata.json` 标记行程模式；新回归 `test_stage5_gui_joint_travel.py` 覆盖全部六轴和真实 Bullet 对象的越界纠正。
- **每次 Windows 原生 GUI 人手拖拽与松手回位仍需人工确认**；已有历史闭环 CI 或测试函数成功不能替代真实鼠标验证，也不宣称无限强度鼠标约束下的物理硬停止。

```powershell
# 六轴运动与 HOLD（新默认单圈交互限位）
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -DiagnosticsTimeoutSeconds 30 -GuiJointTravel single-turn -OutputDir build/stage5-gui-limits-check

# 对照原始 URDF 机械模型的宽行程（仅对照实验）
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -DiagnosticsTimeoutSeconds 30 -GuiJointTravel urdf -OutputDir build/stage5-gui-urdf-check
```

如果操作者要验证避障与单圈限位，可以在第一个命令加上 `-Stage5RouteCase blocked`；该选项会按启动脚本原有行为构建并先测试生产 C APF/IK 主机 DLL。

## 7. 2026-10-30 避障 GUI 现场排障：4/4 到位后进入 HOLD 触发安全间隙

Windows 手测 `blocked` + `single-turn` 的日志显示：`[SETTLED] Segment 1..4/4` 全部到位，然后进入 `[HOLD]`；发生关节 J1/J4 的原生拖拽限位修正；随后 `getClosestPoints` 对 link=5 报告 `distance_m=0.009787`，低于**不变**的 `minimum_full_link_clearance_mm=10`，违反量为 0.213 mm。这个**正距离**说明安全余量不足，不能直接声称物理接触，更不能将它伪装成 APF 规划器故障。因为事件发生在 HOLD，不排除人为拾取拖拽导致；仍须按实际安全违规处理。

观察到的 GUI 障碍方块“无法到达”还有独立的显示原因：历史 blocked 场景创建了真实碰撞箱（2cm x 2cm x 2cm），却没有与之相同形状的 `createVisualShape`；预检的 `direct_cartesian_segment_blocked=True` 是按测量到的 EE 初始世界坐标，沿世界 X 平移 -180mm 的末端直线与障碍箱相交算出来的，并不自动意味着用户能看见它。现在**同一位置/尺寸**创建红色可视物体，实际 GUI 覆盖一条红色原始直线及绿色由 C APF 产生的笛卡尔路径，实际物理 EE 动作仍绘制既有淡蓝色运动尾迹。引导用户区分规划路径与实测轨迹，不伪造避障。

新增严格区分 `planned_motion` 和 `interactive_hold` 的 `Stage5RuntimeClearanceError`，将 link、box、间距、10mm 要求、阶段存入 `result.json` 和 `uart_live.jsonl`。无论任何阶段，违规**立即停止物理和 QEMU 控制、返回 FAIL**，但在 native GUI 上保留冻结场景直到用户按 Q（而非窗口突然消失）。`[APF PHYSICAL EE]` 在最先到达 HOLD 时记录实际末端世界坐标与规划目标坐标、毫米级差值，即使随后人工拖拽触发安全停机也保留这一测量。未降低 10mm 碰撞间隙、未改原场景、未改机械模型、未关闭整臂检查。

回归新增 `test_stage5_gui_avoidance_visibility.py`，覆盖真实 PyBullet 可视障碍与几何位置、HOLD 正间距违规、负间距真实穿透、严格阈值、原直线和 C 规划线渲染；这些仍不足以取代 Windows 人工 GUI 手动拖拽和远程 CI 验收。

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -DiagnosticsTimeoutSeconds 30 -Stage5RouteCase blocked -GuiJointTravel single-turn -OutputDir build/stage5-blocked-visible-check
```

先**不拖拽**，观察红色物理箱、红色直线路径、绿色规划路径与蓝色实测轨迹；等 4/4 到位、日志 `[APF PHYSICAL EE]` 显示实际末端误差，再缓慢移动关节。任何 `[SAFE STOP]` 都是故障证据而非通过。安全停机时按 Q 退出后提供 `result.json`、`avoidance_preflight.json` 和 `worker.log`。

## 8. 2026-10-31 交互式 HOLD 安全回退（未完成碰撞感知恢复规划）

本地 Windows 避障后拖动测试确认：自动 C APF/IK 全部 4/4 段到位，用户拖动后 `uart_live.jsonl` 有 `hold_deviation`（首次跨越0.5°）及后续 `hold_replan`（序列45）。`hold_replan` 发送前真实六轴测量 centideg 为 `[3245,-4250,5290,-6229,4002,6833]`，目标为 `[4065,-6701,9383,-1569,1502,1415]`，实际多关节差值高达数十度。最终 `result.json` 为 `passed=false`, `fault_type=stage5_runtime_full_arm_clearance_violation`。目前尚缺失这一次完整故障距离日志，不能判断是拖拽本身还是直返 rearm 期间发生首次违规，也不能将事后 10ms 间距扫描视为预防碰撞的安全控制。

这是架构缺口，不是把 10mm 改小可修复的数值问题。C APF 预检只证明了**原起点**沿当前规划航点的离散全连杆几何。机械臂被拖出轨迹后的**新起点到原终点**需要重新做避障路径规划、C IK 路径转换与 full-link 验证；旧 `hold_replan` 直接请求最终关节目标，不具备上述证明。且鼠标物理约束可能同时拉动多个关节，不能以单关节小拖动认为整臂安全。

本补丁的**明确可见安全回退**：
- 仅当加载 APF 避障规划时，PyBullet native GUI 用 `COV_ENABLE_MOUSE_PICKING=0`，禁止人为将机械臂拖向障碍物；普通无障碍 GUI 的 picking 保留。
- 仅当存在 APF 安全障碍物时，若 HOLD 超过原偏移阈值0.5°，立即用 `Stage5UnvalidatedHoldRecoveryError` 拒绝继续未经核实的自动回位，冻结 GUI/终止 QEMU，并将 `fault_type=stage5_unvalidated_hold_recovery_blocked` 和 `unsafe_return_rearm_sent=false` 写入 result.json。发 `CMD_REQUEST_JOINT_MOVE` 的路径前还放置第二道相同的 fail-closed guard。该机制防止**未经验证的重新运动命令**，不宣称能够预防任何外力造成的运动或零延迟停止。
- `result.json` 对失败运行也明确保存 `hold_recoveries/hold_rearms/hold_rearm_acks/hold_final_error_deg`，避免日志字段空白被误解释为成功。
- `run_metadata.json` 和 result 记录 `native_gui_picking` 与 `avoidance_hold_recovery_policy`，worker 输出 `[APF HOLD SAFETY]`。避障模式 **4/4 自动闭环及无扰动 HOLD** 仍允许执行，但**不得报告避障场景拖拽回位已通过**。
- 旧 GUI/HOLD 备份、main、Stage4、C APF 与 C PID 参数均不改变，10mm 物理安全限距保持不变。

重新允许障碍环境拖拽必须实现从**实测连续六轴姿态**出发的碰撞感知路径重规划，并考虑交互期间的约束/失效保护、每个实际控制步的安全检查与动态执行偏差。不能只调用已有直返原关节目标 API。

## 9. 2026-10-31 受控仿真扰动后的 C APF/IK 回位（实验性）

新模块 `stage5_obstacle_hold_recovery.py` 从当前真实 PyBullet 六轴关节测量和原障碍物对象出发，使用**生产 C APF + C IK**计算新的笛卡尔/关节路径。由于 C IK 以当前姿态朝向为参考，附加到原始六轴最终目标的**关节返回段**也必须完整通过原有 `validate_joint_path` 的全连杆离散采样，避免把仅“末端位置回到了”的路线伪装成恢复成功。预检完成后必须把原物理状态复原，不允许预检自动执行。

集成中的**受控 DIRECT/QEMU 实验**由 `DemoOptions.test_obstacle_hold_recovery=True` 显式启用，且只能在 DIRECT、真实 QEMU FreeRTOS UART、`interactive_hold=True`、提供 `on_physics` 扰动和 C APF/IK 库时使用。进入 HOLD 后用可重现的 +2° J1 关节位置扰动模拟安全测试起点，任何预检失败均直接 FAIL，**绝不退回旧的直返最后六轴目标命令**。只有整条新路径的预检通过后才逐段发 `CMD_REQUEST_JOINT_MOVE`，每段真实 UART ACK + 15帧实测关节到位后发送下一段；最后必须返回原六轴目标并保留严格 10 mm 的物理检测和故障停机。新旧序列号分开，确保不覆盖原始待处理请求。

**重要限制：** 这个实验性功能没有对外打开 `PyBullet COV_ENABLE_MOUSE_PICKING`；原生 GUI 任意鼠标拖动的约束不是预规划关节运动，而是物理系统实时施力，即使返回路线安全，也无法证明手指拖动时不会早已碰到障碍。受控扰动使用模拟位置重置，不能代替真实力矩或工业硬件动态响应，更不能称作实体机器人部署。直到主动限速/接触预测/完整人机交互约束完成前，APF 场景鼠标拖拽**仍然关闭**。CI 要验证端到端真 QEMU/UART 结果，未通过前不得把实验标记 PASS。

```bash
# CI 上先构建原 C APF/IK bridge 与 QEMU 固件，再运行下列受控实验
python tools/tests/run_stage5_controlled_apf_hold_recovery.py \
  --elf build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf \
  --library build/stage5-avoidance/libstage5_apf_bridge.so \
  --output-dir build/stage5-controlled-hold-recovery
```

## 10. Windows 本机一键受控扰动回归（2026-10-31）

已通过的 Linux QEMU/UART 验证：2026-10-31 工作流 `41159505600`（历史提交 `e1ffad12`）记录真正生产 C APF+IK 的 4/4 段自动避障，人工输入**未使用鼠标**；受控 +2° J1 模拟位置扰动后重新规划 2 段，分别得到 `[HOLD C APF ACCEPTED] Recovery segment 1/2` 和 `2/2`、最终 `All validated recovery segments settled`，`CONTROLLED C APF HOLD RECOVERY ACTUAL QEMU UART PASS`。上一批较短 HOLD 时长的端到端回归曾因没完成第二段而失败，后续延长有限 HOLD 时长后通过；不得把早期失败排除记录。

使用相同验证内核的 Windows 一键入口：`tools/run_stage5_controlled_recovery_windows.ps1`。它会构建真实 FreeRTOS QEMU ELF 和 Windows C APF/IK DLL，执行 CTest，执行 `tools/tests/run_stage5_controlled_apf_hold_recovery.py`。新的模拟扰动测试在把关节重置到 +2° 前，通过 `validate_joint_path` 在**同一物理场景、同一障碍物、同一场景要求的 10mm 阈值**上检查起点到位移终点。失败即拒绝扰动。该受控实验仅能说明离散几何预检的有效性，不证明连续物理力矩人机交互安全。

```powershell
cd "D:\1_ToGo\Robo\code\six-axis-robot-motion-control"
git status --porcelain  # 非空时停止切换，保留本地改动
git fetch origin feat/stage5-gui-preserved-integration-20261030
git switch --detach FETCH_HEAD
git rev-parse --short HEAD
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_controlled_recovery_windows.ps1 -OutputDir build/stage5-controlled-recovery-local-1
Get-Content build/stage5-controlled-recovery-local-1/controlled_recovery_acceptance.json
```

可以覆盖传入 `-Python python` 与 `-Qemu <path>`（要与目标 Python 的 PyBullet、C DLL 架构兼容），默认使用已有 .venv；始终在项目仓库根目录运行。脚本会新建`controlled_recovery_acceptance.json`，每个关键检查都有 true/false，并保留 `result.json`、`uart_live.jsonl`、`joint_trace.csv`、`run_metadata.json`；只有 `passed=true` 才有本轮受控闭环 PASS 的依据。

在当前恢复规划算法的开发阶段，**原生 PyBullet GUI 任意鼠标拖拽保持禁用**。人机拖拽的运动约束、碰撞预测/安全距离以及真实机器人硬件安全均没有因此自动实现。单独的 Windows GitHub Actions job `windows-controlled-obstacle-recovery` 使用用户同一脚本，必须有明确的 Windows 完整回归通过记录，才能宣告跨 Windows 环境已验收；旧稳定备份与 main 不动。

## 11. 原生 GUI 受限关节点动（2026-10-31，Windows 手工验收前）

先前 Windows 本机 DIRECT/QEMU 的 C APF/IK 从当前六轴姿态返回原目标已经通过：`controlled_recovery_acceptance.json` 全部 true，初始 4/4 段 + C 回位 2/2 段，`hold_recoveries=1`，`hold_final_error_deg=0.02`，回位规划路径离散最小整臂间隙26.23mm，高于10mm要求（**不代表连续扫掠或物理电机安全证明**）。

现在实验性 GUI 入口 `-SafeJog` 或 Python `--safe-gui-jog`：只允许载入 blocked C APF/IK、原生 GUI、非被动 `HOLD` 时启用。PyBullet 原生鼠标任意拖拽 **始终关闭**。控制面板新增两个真正的交互按钮：

- `SAFE APF JOG: J1 +2 deg (once in HOLD)`
- `SAFE APF JOG: J1 -2 deg (once in HOLD)`

程序在收到按钮边沿后，拒绝早于 HOLD 的命令；每次测试只允许**一项成功点动**。在施加偏移之前，严格验证实际 GUI 单圈 J1行程、从当前六轴实测姿态到提出的新姿态的全部关节段、同一 PyBullet 场景内全连杆与真实障碍物的距离。交互预检在原场景最低10mm之外另保留2mm裕量，即要求 **至少12mm**；接触检测最大间隔不超过0.25°；预检结束必须还原原来物理状态。任何失败都记录 `safe_gui_jog_rejected` 并且**不改动关节姿态、也不发 UART 重规划**。按键路径只做模拟 `resetJointState` 注入小幅姿态扰动，不是安全的人力牵引或实时力矩控制。

成功授权后在该同一 GUI 机械臂上形成2°受控位移，随后使用已有“当前实测六轴 → 生产 C APF/IK → 原六轴姿态补段 → 全连杆整条路径验证”的回位规划，以及真实 QEMU FreeRTOS UART 多段 ACK 和物理反馈结算。任何无 GUI 按钮授权的其他>0.5°偏差仍以 `Stage5UnvalidatedHoldRecoveryError` 拒绝直返。运行时10mm整臂安全检查、界面故障冻结和 QMP 时钟锁均保留。

```powershell
# 项目根目录，拉取 Stage5 PR 最新提交后运行
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -Stage5RouteCase blocked -SafeJog -GuiJointTravel single-turn -DiagnosticsTimeoutSeconds 30 -OutputDir build/stage5-safe-gui-jog-local-1
# 等到界面和控制台出现 [HOLD]，只点击面板一个 SAFE APF JOG 按钮；
# 两段 C 回位都显示 [HOLD C APF ACCEPTED] 与 SETTLED，再按 Q 退出
Get-Content .\build\stage5-safe-gui-jog-local-1\result.json
Get-Content .\build\stage5-safe-gui-jog-local-1\worker.log -Tail 80
```

验收必须同时满足：`passed=true`、`safe_gui_jog_applied=1`、`native_gui_picking=false`、`obstacle_recovery_preflight.passed=true`、`obstacle_recovery_segments_settled>=1`、`hold_recoveries>=1`、最终误差<=0.15°，无 runtime clearance breach 和固件 fault。**没有按按钮就正常退出不得算点动通过**；按钮撞到拒绝区域不会擅自调整障碍物或者降低阈值。失败结果同样保留完整日志；由于这一版本尚需 Windows GUI 鼠标点击验收，不能将 Python 单元测试等同为 GUI 人工验收。

留意模式区别：`--safe-gui-jog` 是受控 GUI 按钮操作，不是 `--test-obstacle-hold-recovery` 的 DIRECT 无窗口注入，也不是以前曾经导致接触的原生 drag picking。真实硬件不在本项目目前验证范围内。

## 12. 六轴可选小步点动、回程预检和有限多轮恢复（2026-10-31）

Stage5 手动 GUI 安全点动按键的旧 J1+2° 单轮模式已在 Windows 本机验收成功，原报告: `safe_gui_jog_applied=1`, `obstacle_recovery_segments_settled=2`, `hold_recoveries=1`, `hold_final_error_deg=0.02`, `runtime_full_link_collision_free=true`。本节是**后续新版本**，不同于已经通过的旧实验，需重新手工验收，不得移用旧 PASS。

保留 `-SafeJog` GUI 入口（不重新开放无约束 mouse picking），将按钮改为左侧 GUI 调试面板的两个滑块和正反按钮：

1. `SAFE JOG: selected joint J1..J6`，将滑块调到关节编号 1 至 6（内部向最近的整数取整；超界或非有限值拒绝）。
2. `SAFE JOG: step deg (0.5,1,1.5,2)`，将步长调为0.5、1、1.5或2度（最大2度）。
3. `SAFE APF JOG: selected J + step` 或 `... - step`。只在自动避障及上一轮恢复全部真实 ACK / 物理结算完成之后允许操作，按键不能提前排队；**每次运行最多三次成功点动回位**，其它按键拒绝并在 result.json记录原因。

每个按钮均先读取同一 PyBullet URDF/GUI 实际六轴关节位置、速度；按所选轴的**当前**连续角计算候选新状态，不使用固定 J1 零点。检查 GUI 关节限位；用真实全连杆 Bullet 碰撞检测以**最多0.25°采样间隔**检查当前姿态→候选姿态，在原10mm安全距离之外预留交互额外2mm裕量（即 12mm）；**额外预检完整的 C APF/IK 回程到原六轴目标**，也要求12mm。回程预检期间只在同一仿真世界暂时设为候选姿态，且必须还原每轴原始位置及速度，不对 UART 下发动作。任一检查失败即拒绝点动，不改变实际模拟关节位置。通过后才在当前模拟机器人上施加受限关节状态偏移，交由已经验证过的真实 FreeRTOS/UART C APF/IK 回程流程执行，再次以12mm检查返回路线、运行时10mm检测不变。

**多轮回位状态约束：** 仅当所有 C APF 回程段均收到固件 ACK、并分别经过15帧实测到位，才将按钮解锁；回程 UART 序列号从前一轮最后一个序列号的下一值继续分配，避免每轮从45重新开始；`obstacle_recovery_routes` 记录各轮 `first_sequence`、规划段数和 `completed`。最终验收要求 `safe_gui_jog_applied == safe_gui_jog_completed == hold_recoveries` 且至少1，未完成任何一轮即 `passed=false`；`safe_gui_jog_refused` 允许非零，但拒绝期间不允许关节姿态突变或触发新 UART 请求。

PowerShell 命令沿用：
```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -Stage5RouteCase blocked -SafeJog -GuiJointTravel single-turn -DiagnosticsTimeoutSeconds 30 -OutputDir build/stage5-gui-sixaxis-jog-1
```
第一次手工验收建议只选择 J1、+2°，验证旧路径没有回归；第二轮在上一轮全部回位后选择另一轴（可先 J6 ±0.5°），确认新按钮状态、动作是否经 C APF/IK 重新规划且结果安全。**并非所有轴/方向/位移都保证可达，预检失败时拒绝点动才是正确行为。** 交互选项只供仿真；无连续扫掠形式化证明，无真实接触/力矩/伺服硬件安全认证。

自动化新增 `tools/tests/run_stage5_controlled_apf_hold_recovery.py --cycles 2`，在 PyBullet DIRECT 上安排两次不同符号的有界 J1 偏移，验证各轮重新规划、UART 序列不重复、物理到位及严格碰撞检测。Windows 一键脚本 `tools/run_stage5_controlled_recovery_windows.ps1 -Cycles 2` 可离线再现。此功能与真实 GUI 人工按钮是两种验收，不应混为一谈。
