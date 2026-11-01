# Stage5 5 cm 自动绕行实验（不替换已验收 3 cm 基线）

本实验始于已验收的 `ab2efaf`（PR #37）代码，独立分支 `experiment/stage5-5cm-auto-bypass-20261101`。原 `blocked` 仍为 3 cm；新增 `blocked_large` 立方体边长为 **5 cm**，起终点和沿程中心位置与基线一致；为了保持全连杆可行空间，障碍物向 EE 的 +Y 方向偏移 12.5 mm，仍实际截断起终点直线。

- 单一配置来源：`simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json`。
- 不设置 `via_offsets_ee_mm`。GUI 通过 C `robot_apf_generate_bypass_vias` 生成候选，再用真实 C APF/IK 与 PyBullet 全连杆预检逐条否决或许可。
- 运行时净距仍为 **10 mm**；自动规划预检为 **13 mm**（含额外 3 mm 余量），**不放宽**。
- 这是主机 GUI 实验场景，**不是**替换已有 ARM 固件 3 cm / 29 段基准。实验路径不可用时要拒绝执行，不保证 5 cm 一定有解。
- 本轮先验证较大障碍初始路线；未证明成功前 **不启用 SafeJog**，也不作为正式完成宣称。

## Windows 在原仓库目录运行

保留旧修改的 stash，不使用 `reset --hard` 或 `stash pop`。如果之前的 GUI 已关停，在 PowerShell 里：

```powershell
cd D:\1_ToGo\Robo\code\six-axis-robot-motion-control
git status --short
# 仅在没有未提交修改妨碍切换时继续
git fetch origin
git switch --detach origin/experiment/stage5-5cm-auto-bypass-20261101
git log -1 --oneline

$py311 = Join-Path $env:APPDATA 'uv\python\cpython-3.11.15-windows-x86_64-none\python.exe'
$packages = 'D:\1_ToGo\Robo\code\six-axis-robot-motion-control\.venv\Lib\site-packages'
& $py311 .\tools\stage5_existing_python.py --site-packages $packages --probe

powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 `
  -Python $py311 -ExistingSitePackages $packages `
  -Qemu qemu-system-arm -Stage5RouteCase blocked_large -NoHold `
  -OutputDir build/stage5-5cm-auto-bypass
```

**注意：** `-NoHold` 用于只做本轮初始路线验证，脚本会在执行结束后关闭 GUI。不要添加 `-SafeJog`、`-RequireTwoStepJog`，因为这两个交互模式仍仅在已验收的 3 cm `blocked` 场景允许。

检查 `build/stage5-5cm-auto-bypass/result.json`，至少核对 `passed`、`avoidance_preflight`、`segments_settled`、`runtime_full_link_queries` 和故障计数。成功后仍需确认执行日志证明运行时全连杆净距满足 10 mm。失败请保留 `worker.log`，不可改阈值来强行通过。

CI：独立 `Stage5 experimental 5cm obstacle` 工作流使用 Linux QEMU + 原生 PyBullet + C APF 桥接做相同初始路线实测。无结果前不能宣称 5 cm 已可绕行。

## 初次实测与第二轮候选扩展

首次 [Linux 5 cm 实机物理仿真](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41318424618) **FAIL**：原 C 只找到 8 条几何候选，全部由原有全连杆 13 mm 规划门禁拒绝。测试中存在 12.967 mm 的近门槛距离，**不得放松门槛或报告成功**。

第二轮只扩大 C 几何绕行搜索：每个侧向从 3 圈扩至 5 圈（查询 C 中单一容量声明），新增远距离候选仍必须逐条通过生产 C APF/IK、全连杆与真实 UART。独立 CI 结论决定 5 cm 是否可运行。

## 位置诊断结论（仅预检，尚非实际到位证明）

[无动作的 Bullet DIRECT 扫描](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41319131531) 对 15 个 5cm 障碍位置运行真实 C APF/IK 及全连杆 13mm 预检，**仅 `center_offset_ee_mm=[-90,12.5,0]` 通过**（选中第 3 个 C 几何候选，规划最小净距约 13.0006 mm）。原来的 [-90,0,0] 位置 16 条候选仍全部被拒。该位置变化仅编辑场景 JSON，保持尺寸、算法参数、10mm 运行时要求及 3mm 规划余量不变；必须继续进行完整 QEMU/ARM UART/PyBullet 物理执行验证。
