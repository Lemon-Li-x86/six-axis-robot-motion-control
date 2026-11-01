# Stage 5 Windows 操作员验收：按钮可重复，独立 C/UART 回位

**版本**：`feat/stage5-rtos-apf-closure-20261101`（PR #36，在 PR #35 的 GUI 修复之上新增固件内 APF/IK；请以本页所在 Git HEAD 核对）。本项目是 **QEMU ARM/FreeRTOS + 生产 C APF/IK + UART + PyBullet 虚拟 UR5**，不是有实体伺服驱动的真实工业机器人。

## 原始大纲对照

| Stage5 原 PDF 要求 | 已证实范围 | 仍须独立留证 |
|---|---|---|
| 5.1 驱动→协议→算法→规划→PID→反馈 | QEMU Cortex-M4/FreeRTOS 实现 C 控制/轨迹并由 UART、PyBullet 物理反馈闭环，独立 ACK/实测稳定到位、错误协议回归 | 本机操作记录 |
| 5.2 静态障碍 + C APF | PR #36 的 ARM FreeRTOS Trajectory Task 真正执行生产 C APF/FK/IK，并以 UART 提案交给 PyBullet 逐连杆安全门禁；被阻挡路线已有 28/28 实际物理到位记录 | 本机同版本独立复验；高层途经点和物理净距授权仍由仿真主机管理，不是任意环境自主避障 |
| 5.3 CPU/内存/调度/中断优化 | 真实 ARM .map 同配置 FLASH 105256→87384B，Monitor CPU 0.07%→0.04%，RAM 不变；任务/ISR 优先级已审查 | QEMU 短窗口 P99 周期抖动改善不稳定且有退化实验；无 WCET 和长期高负载改善证明 |
| 5.4 QEMU/GDB/UART 排障 | 字节帧恢复、计时竞态、QEMU 监控计数与物理帧不共起点 bug、GUI 末端纠偏等都有记录、测试 | 本机 J6 第二次 +2° 实测、FINISH 和最终归档 |

详情见 [原大纲 24 项审计](../project_plan_progress_audit_2026-10-31.md)、[Stage5 原始 5.1–5.4 严格审计](19_stage5_original_requirements_evidence_review_2026-11-01.md)、[技术测试和性能量测](15_stage5_exit_review_2026-10-30.md)、[参数登记簿](01_parameter_register.md)。

## 提交与环境预检

使用已安装的 arm-none-eabi-gcc、CMake、QEMU、Python+PyBullet、Windows PowerShell。先确认本地代码没有待保存改动。**不要**在有未提交修改时执行覆盖、`reset --hard` 或清理整个 build 目录。

**多 worktree 的 Python 选择（2026-11-01 修订）**：两个 Windows 验收脚本均自动检查**现有**解释器的 NumPy（项目要求 `numpy<2`）及 PyBullet，优先尝试当前工作树的 `.venv`，再尝试 Windows `py` 已安装解释器，最后检查 `python` 命令。**不安装任何东西，不新建环境**。不要传 `-Python python` 来强制选用另一个工作树已激活的旧 `.venv`。如需坚持使用已验收过的解释器，显式传 `-Python 'C:\path\to\working\python.exe'`；指定解释器若不能导入依赖则立即终止，不会默默改用另一个版本。

```powershell
git status --short
git fetch origin
git worktree add ..\stage5-acceptance origin/feat/stage5-rtos-apf-closure-20261101
cd ..\stage5-acceptance
git rev-parse --short HEAD
```

确认 HEAD 与 PR #36 当前 head 一致。**先运行自动的 ARM 内 C APF 整路安全闭环**（它与 GUI 主机侧 APF 是两项独立证据）：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_embedded_apf_windows.ps1 `
  -Qemu qemu-system-arm `
  -TimeoutSeconds 300 `
  -OutputDir build/stage5-windows-embedded-apf
```

此命令必须 `passed=true`、`route_complete=true`、28/28（按该场景已知成功基线）、所有 `approved_legs` 具有 ACK 和稳定物理到位、`uart_checksum_errors=0`，且 `runtime_full_link_queries` 非零并覆盖每个连杆。新增运行时逐连杆修复在最新 HEAD 的 CI 通过前不得提前标为已复验。

随后运行**一次**原始避障 + 连续多次点动的完整 GUI 仿真（不是分别启动多个演示）。`-RequireTwoStepJog` 仅要求**前两次** J6 +0.5°、+2.0°，不限制之后正常按钮使用的次数：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 `
  -Qemu qemu-system-arm `
  -Stage5RouteCase blocked `
  -SafeJog -RequireTwoStepJog `
  -SafeJogHoldSeconds 600 `
  -OutputDir build/stage5-windows-final-2step
```

1. 等待 **原始生产 C APF 4/4 航点全部按真实物理反馈收敛**，状态显示 READY/HOLD。
2. J6 +0.5°，等待状态出现 **RETURN COMPLETE**；不点击第二次，直到确认恢复。
3. 选择 J6 +2.0°，点击同一安全 `+`；等待第二次真实 C/UART 回位、再次显示 RETURN COMPLETE。不能将第一轮 0.5° 成功代替 2°。
4. 继续按至少三次，例如 J6 +2.0° ×3，**每次都必须等待真实回位并重新进入 READY**；可选其他关节/方向/0.5° 递增步长，但始终遵守限位与全连杆碰撞检查。第五次也不能因次数限制被拒绝。
5. 点击 `SAFE JOG FINISH/EXIT`，**不要**用窗口的 X 直接关闭。最后保留 `result.json` 和 `worker.log`。
6. 预期最终验收打印 `[STAGE5 GUI] PASS`。若完成五次，应有 `safe_gui_jog_applied=5`、`safe_gui_jog_completed=5`、`hold_recoveries=5`、`safe_gui_jog_refused=0`；如果多于五次，这三项也必须全部等于实际成功次数。原始航点 `segments_settled=4/4`，UART/固件故障为零，物理回位不超 0.15°，全连杆与时钟一致性门禁均通过。

**本次修复最关键的验收行为**：原始 Windows ZIP 显示 5 次点击仅成功 3 次，另外 2 次因会话硬上限被拒绝，旧程序却误写 `passed: true`。现在只要有被拒绝的点动、未完成的 C/UART 回位或未按顺序完成前两步，就不能得到 PASS。**正常按钮没有次数上限**；仅 CI 有有限的自动测试动作序列。

打包已产生的日志（输出目录不包含真实个人信息时再分享）：

```powershell
Compress-Archive -Path .\build\stage5-windows-final-2step\* `
  -DestinationPath .\build\stage5-windows-final-2step.zip -Force
```

## 自动化门禁与封版

- `Stage5 live QEMU PyBullet observer`：`native-gui-no-drag-xvfb`、`native-gui-scripted-j6-xvfb`、`native-gui-j6-two-step-xvfb` 及新增 `native-gui-j6-five-repeat-xvfb` 都必须 PASS。新增五次测试完整串行调用真实 C APF、QEMU/FreeRTOS UART、五次物理反馈回位、碰撞和时钟门禁；脚本化点击**不是** Windows 鼠标操作的替代。
- Windows/Linux 单测、ARM 固件回归、Windows C APF DLL、两平台 QEMU 实际 UART 和 Windows 控制回位均须 PASS，且必须对应**同一提交 HEAD**。
- 不得降低 0.15°、安全净距或修改 UR5/驱动/PID 参数来刷绿。仍保留 Stage5 原大纲 5.2/5.3 范围差距，不能把此 GUI 验收作为“完整自主固件 APF + CPU/内存优化全部完成”的证明。
- PR #36 在最新提交 CI 与 Windows 连续至少五次成功前保持 Draft；PR #35 是下层 GUI 修复基线，`main` 与历史 Stage4 快照不改动。

**签收条件**：同一代码版本的 ARM 固件内 APF 整路测试、逐连杆运行时监测、GUI 和固件回归门禁通过 + Windows 同一轮至少五次合法点动无拒绝、全部真实回位。5.3 的抖动改善仍须单独提供可重复高负载数据，否则阶段结论必须保留该性能限制，不能声称全面优化或实体工业机器人安全认证。
