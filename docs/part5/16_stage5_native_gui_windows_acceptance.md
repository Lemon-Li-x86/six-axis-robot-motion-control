# Stage 5 GUI / Windows: CI 准入结果与一次性本机验收

记录日期：2026-10-30。此文档仅对应 **软件仿真** Cortex-M4 FreeRTOS / QEMU UART / PyBullet UR5，不控制任何实体电机。

## 1. 云端已有证据

[PR #28](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/28)，合并提交 `970a887c`。
GitHub Actions：[Stage5 live QEMU PyBullet observer #41015789848](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41015789848)，**六个 CI Job 全绿**：

- Linux Xvfb 实际 p.GUI 窗口、**无鼠标干预**三段 QEMU FreeRTOS UART/PyBullet 运动：成功。
- Windows 与 Linux 的 `genuine-qemu-uart-live-observer`，包括真实 ARM 固件构建、UART 反馈闭环、延迟观察器与外力扰动恢复：成功。**Windows Job 是 `--headless-observer`，不是 Windows 桌面原生 PyBullet GUI。**
- Windows 与 Linux 的非阻塞传输/真实关节观察器单测：成功。
- 真实阻挡静态避障仍由主线独立的 C APF + IK / full-link PyBullet 回归验证，不把本 GUI 三段自由关节运动错误描述为 GUI 障碍物绕行。

这组证据足以在主线启用可复用的原生 GUI 观察器和 Windows 启动器；**不能替代用户电脑的 Windows OpenGL 显示、鼠标拖拽和交互 HOLD 体验验证**。

## 2. 用户 Windows 电脑只需一次集中验收（无需现在执行）

前置条件：Windows PowerShell，Git 仓库已更新到包含 `970a887c` 的最新 `main`，Python 虚拟环境 `.venv\Scripts\python.exe`、`requirements.txt` 依赖，CMake + Ninja、ARM GNU Toolchain 和 `qemu-system-arm` 均可在 PowerShell 里找到。没有安装这些工具时先报告缺失项，不要手动修改固件配置或复制代码。

在仓库根目录，只执行以下命令即可自动编译虚拟 ARM 固件并启动**三段后自动退出**的原生 GUI：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 -NoHold
```

`-NoHold` 很重要：它是可定界的三段 GUI 运动验收，不会把交互式 HOLD 无限运行误判为程序卡死。运行后核对：窗口确实出现、机械臂六轴都有可见/日志可测的运动、`segments_settled == segments_requested == 3`、`passed: true` 且 FreeRTOS 规划/控制故障为零。保留 `build/stage5-live-windows/` 内的 `result.json`、`worker.log`、真实关节 CSV 和一张窗口截图；若 Windows OpenGL2 兼容性有问题，记录屏幕错误和日志，不通过盲改 PID 处理。

三段测试通过后，再单独验证真正的 Windows 原生**鼠标操作**：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1
```

它默认进入 PyBullet 原生交互 HOLD，窗口提示左键拖动连杆并松开，按 `Q` 或 `Ctrl+C` 退出。记录窗口是否响应、是否发生固件故障与拖拽事件日志；**交互 HOLD 自动重规划/恢复的正确性不可从一次手工拖拽推断**，需与完整故障注入指标分开验收。此测试为可选的交互功能审核，不影响已经合并的非交互核心演示。

## 3. 失败处置与可维护性约束

- `PowerShell` 报找不到 Python/CMake/Ninja/ARM 编译器/QEMU：报告原始命令及错误，补环境依赖；不要更改 UR5 配置、串口协议或复用旧 PR #15 的控制代码。
- 图形窗口卡顿、白屏、OpenGL 错误：保存窗口截图、`worker.log`、`result.json`、`uart_live.jsonl` 和日志最后 60 行。先查显卡驱动和 PyBullet 原生渲染环境；不放宽诊断超时或控制误差阈值来掩盖。
- 串口反馈不连续、任何规划/控制/stale fault：**判本机 GUI 验收不通过**，保留完整证据；固件已实现非零故障即时停止，不能因三段将近结束而继续忽略。
- 不把 Windows CI 无头结果当作真实 Windows 窗口通过，也不把用户一次手工拖拽当作工业机器人安全认证。

第六阶段可复用此 GUI 作搬运任务可视化，但任务编排、抓取/放置状态、多场景负载与长时耐久仍需独立开发与测量。
