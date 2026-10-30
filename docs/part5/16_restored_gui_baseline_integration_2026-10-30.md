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
