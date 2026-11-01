# Stage 5 Windows 操作员验收：按钮可重复，独立 C/UART 回位

**版本**：`fix/stage5-final-closeout-20261031`（PR #35，待实际 Windows 签收；请以本页 Git HEAD 核对）。本项目是 **QEMU ARM/FreeRTOS + 生产 C APF/IK + UART + PyBullet 虚拟 UR5**，不是有实体伺服驱动的真实工业机器人。

## 原始大纲对照

| Stage5 原 PDF 要求 | 已证实范围 | 仍须独立留证 |
|---|---|---|
| 5.1 驱动→协议→算法→规划→PID→反馈 | QEMU Cortex-M4/FreeRTOS 实现 C 控制/轨迹并由 UART、PyBullet 物理反馈闭环，独立 ACK/实测稳定到位、错误协议回归 | 本机操作记录 |
| 5.2 静态障碍 + C APF | AABB 障碍；主机生产 C APF/IK 计算静态路由，PyBullet 全连杆离散预检，固件 C/RTOS 经 UART 实际执行；另新增 ARM **非驱动** APF 单步诊断预览 | 原 PDF 写“APF 嵌入固件路径规划模块”；**固件运行时完整 APF 自主路径规划尚未实现**，不得凭 Host 生产 C 库写成完全符合 |
| 5.3 CPU/内存/调度/中断优化 | 现有 CPU、栈、P99/峰值周期、ARM map 同配置量测和路径段压缩证据 | 未有完整相同场景的 CPU/ISR/内存优化 A/B 证据，不能宣称全部优化实现；FLASH +48B（RAM 未变）不是内存优化 |
| 5.4 QEMU/GDB/UART 排障 | 字节帧恢复、计时竞态、QEMU 监控计数与物理帧不共起点 bug、GUI 末端纠偏等都有记录、测试 | 本机 J6 第二次 +2° 实测、FINISH 和最终归档 |

详情见 [原大纲 24 项审计](../project_plan_progress_audit_2026-10-31.md)、[Stage5 有保留结项](17_stage5_final_acceptance_2026-10-31.md)、[技术测试和性能量测](15_stage5_exit_review_2026-10-30.md)、[参数登记簿](01_parameter_register.md)。

## 提交与环境预检

使用已安装的 arm-none-eabi-gcc、CMake、QEMU、Python+PyBullet、Windows PowerShell。先确认本地代码没有待保存改动。**不要**在有未提交修改时执行覆盖、`reset --hard` 或清理整个 build 目录。

```powershell
git status --short
git fetch origin
git switch --detach origin/fix/stage5-final-closeout-20261031
git rev-parse --short HEAD
```

确认 HEAD 与 PR #35 当前 head 一致，运行**一次**原始避障 + 连续多次点动的完整仿真（不是分别启动多个演示）。`-RequireTwoStepJog` 仅要求**前两次** J6 +0.5°、+2.0°，不限制之后正常按钮使用的次数：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 `
  -Python python `
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
- PR #35 在 Windows 连续至少五次成功及全 CI 通过前保持 Draft；PR #33 不被覆盖，`main` 与历史 Stage4 快照不改动。

**签收条件**：所有自动化门禁为绿 + Windows 同一轮至少五次合法点动无拒绝、全部真实回位 + 补齐 5.2/5.3 的原 PDF 严格要求，才可称 Stage5 完整结项。否则保留技术结项状态，不假称无保留签收。
