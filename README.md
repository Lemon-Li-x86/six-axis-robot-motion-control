# 六轴工业机器人嵌入式运动控制固件开发与仿真验证

基于 **ARM Cortex-M4、FreeRTOS、QEMU、C11 和 PyBullet** 的 UR5 六轴机器人软件仿真工程。ARM 固件负责运动学、轨迹规划、增量式 PID、限幅与通信任务；PyBullet 模拟关节执行器和物理反馈，通过 **真实 QEMU UART** 形成闭环。

> **项目边界：** 本仓库验证的是软件仿真中的受控运动链路，不连接实体工业机器人；PyBullet 的虚拟执行器、全连杆离散碰撞门禁与 QEMU 时间结果均不等于实体硬件安全认证或实时性保证。

## 当前状态（2026-11-01）

**Stage 1–4 核心功能已验收；Stage5 软件集成关键路径已通过跨平台 CI，但 Windows 本机人工 GUI 最终签收仍待完成；Stage6 尚未开发验收。**

| 阶段 | 范围 | 当前结论 |
|---|---|---|
| Stage 1 · 环境 | Cortex-M4 / FreeRTOS、QEMU、UR5、UART 双向通信 | 已完成 |
| Stage 2 · 驱动与协议 | UART / Timer / GPIO、Ring Buffer、协议解析、关节状态抽象 | 已完成 |
| Stage 3 · 运动学 | UR5 FK/IK、逆解筛选、连续角与关节限位 | 已完成 |
| Stage 4 · 运动控制 | 梯形速度、三/五次多项式、笛卡尔直线/圆弧、PID、FreeRTOS 任务 | 核心回归通过，历史验收记录独立保存 |
| Stage 5 · 集成与验证 | ARM FreeRTOS 内 C APF/IK、UART、PyBullet 28 段静态避障、GUI 安全点动、CPU/内存观测 | **指定场景的软件闭环 CI 通过**；本机连续 GUI 操作和部分性能目标仍有保留条件 |
| Stage 6 · 综合应用 | 搬运任务、虚拟抓取/放置、多场景与长时运行、最终交付 | 尚未完成 |

### 当前可信的软件验证基线

- **提交：`23d1faa`**，分支：`fix/stage5-verdict-docs-20261101`，[PR #37](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/37)（Draft，等待本机人工签收）。
- [Stage5 live QEMU PyBullet observer CI](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41299105613)：**13/13 jobs PASS**，包括 Linux/Windows UART 观察器、Windows ARM 首航点、Linux ARM C APF 28 段完整路线、原生 GUI 三段运动、J6 重复操作、Windows 障碍恢复、桥接与单元测试。
- [独立 Native firmware regression](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41299109366)：**PASS**。
- 这轮修复针对两类不同问题：① UART 诊断计数与最终物理帧不在同一采样时刻，导致**时间同步假失败**；② `Ki=0` 的增量式 PID 在稳态反馈下可能保留输出偏置，导致**末端收敛停滞**。修复有回归测试，并未修改 JSON PID 增益、关节安全限位或把时钟误差验收阈值放宽。
- **不能把 CI 全绿写成本机已通过。** 同提交的 Windows 原有环境下，仍须跑完整固件内 APF 路线，再完成操作员原生 GUI 连续至少五次合法安全点动，每次真正回位，最后点击 `SAFE JOG FINISH/EXIT`。

> 以上 CI 是 **`23d1faa` 代码基线** 的证据。后续仅文档更新的提交会有自己的 HEAD 和 CI，不能把旧提交的绿色状态自动算作新提交的测试结果。

### 仍需明确保留的技术边界

- 静态 `blocked` 场景已有 **28/28** 个 ARM FreeRTOS C APF/IK 候选、逐段全连杆批准、UART ACK 与 PyBullet 实际到位；但场景包含指定的高层途经点，且**全连杆碰撞许可仍由 PyBullet 仿真主机负责**。这不是“固件独立对任意障碍自主导航”。
- 同配置 ARM `.map` 对比显示 FLASH **105,256 B → 87,384 B（减少 17,872 B）**，RAM `.data+.bss` **33,796 B 不变**；Monitor 任务短窗口 CPU 占比有所降低。但多次 QEMU 的控制周期 P99 抖动结果方向不一致，**不能声称已证明持续降低抖动、WCET 或实体硬件实时性**。
- Stage6 的完整搬运、抓取/放置语义与长时稳定性测试尚未完成。

详细依据见 [Stage5 原始要求 5.1–5.4 逐条证据审查](docs/part5/19_stage5_original_requirements_evidence_review_2026-11-01.md)、[Stage5 Windows 本机最终验收说明](docs/part5/18_stage5_windows_final_two_step_acceptance.md)及 [阶段 1–6 项目大纲审计](docs/project_plan_progress_audit_2026-10-31.md)。注意历史审计文档按其写作日期保存快照，不自动代表最新提交状态。

## 架构与数据边界

```text
路径 A：既有主机 C APF/IK 预检（Windows GUI）
场景参数 → Host C APF/IK → PyBullet 全连杆门禁
         → ARM UART 关节请求 → FreeRTOS 轨迹 / PID / 限幅
         → PyBullet 关节物理反馈 → UART → ARM

路径 B：真实 ARM FreeRTOS C APF/IK（完整 28 段 CI 与本机无头验收）
PyBullet 当前六轴反馈 + 静态 AABB / 目标
         → UART 0x05 → ARM ProtocolRX 请求队列
         → ARM Trajectory Task：生产 C APF + FK / IK
         → UART 0x88 提案（尚未授权执行）
         → PyBullet 全连杆扫掠安全门禁
         → UART 0x03 单独授权运动
         → ARM Trajectory / Control / PID / limiter
         → UART 目标角 → PyBullet 实测反馈 → 下一段
```

**提案 ≠ 运动许可；ACK ≠ 物理到位。** 运行验收必须检查真实反馈、持续稳定到位、碰撞安全、错误计数和时间一致性。路径 A 的 GUI 结果不能替代路径 B 的“固件内规划”证据，反之亦然。

主要代码：

```text
firmware/freertos_demo/
  common/           类型、单位与错误码
  model/            UR5 模型、关节约束
  drivers/          UART、Timer、GPIO、MotorState
  communications/   协议与 Ring Buffer
  algorithms/       FK/IK、APF、轨迹、PID、命令限幅
  application/      FreeRTOS ProtocolRX、Trajectory、Control、Monitor
  config/           默认参数及生成配置

simulation/          PyBullet UR5、静态障碍场景、UART 虚拟闭环
tools/               Windows 验收入口、回归、诊断与构建脚本
docs/                架构、参数登记、阶段记录、性能与验收证据
```

**单一参数来源：** 固件 PID 增益、任务周期、速度/加速度、超时等默认值由 [`robot_defaults.json`](firmware/freertos_demo/config/robot_defaults.json) 管理；UR5 物理模型/限位属于模型层；场景障碍与安全门禁策略由 [仿真场景](simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json) 管理。不要在运行脚本里复制“另一套 PID / 关节限制”。

## Windows 本机验收：复用仓库已有脚本

**不需要手工拼接 CMake、QEMU 与 Python 启动命令。** 本仓库已有两个 PowerShell 入口：

| 入口 | 做什么 |
|---|---|
| [`tools/run_stage5_embedded_apf_windows.ps1`](tools/run_stage5_embedded_apf_windows.ps1) | 自动构建 ARM 固件；无头运行 **固件内 C APF/IK 的完整 28 段路线**，审查全连杆安全和实际回位，并归档结果 |
| [`tools/run_stage5_live_windows.ps1`](tools/run_stage5_live_windows.ps1) | 构建所需固件/桥接，打开 PyBullet GUI，完成 **人工 SAFE JOG 连续回位** 与最终审计 |

### 1. 同步已验证的功能分支

在 Windows PowerShell 中执行，使用现有验收工作树，不新建第二份仓库：

```powershell
cd D:\1_ToGo\Robo\code\stage5-acceptance
git status --short
# 如果有未提交代码修改，先停止，不要覆盖它们。
git fetch origin fix/stage5-verdict-docs-20261101
git switch --detach origin/fix/stage5-verdict-docs-20261101
git log -1 --oneline
```

核对当前 HEAD 与要验收的代码及对应 CI。**不要用 `reset --hard` 覆盖工作区，也不要清理历史 `build` 日志。**

### 2. 使用已有 Windows Python/NumPy/PyBullet（不重装）

旧项目虚拟环境位于：

```text
D:\1_ToGo\Robo\code\six-axis-robot-motion-control\.venv
```

**重要兼容情况：** 现存虚拟环境的 `python.exe` 可能是 Python 3.10，但其 NumPy/PyBullet 二进制扩展是 CPython 3.11 版本。遇到这种情况，**不要直接强行运行旧 `.venv\Scripts\python.exe`**，也不必创建新的环境。仓库脚本支持用已有 uv CPython 3.11 解释器加上原环境 `site-packages`，只对当前进程添加导入路径，不改环境或安装包。

```powershell
$py311 = Join-Path $env:APPDATA 'uv\python\cpython-3.11.15-windows-x86_64-none\python.exe'
$packages = 'D:\1_ToGo\Robo\code\six-axis-robot-motion-control\.venv\Lib\site-packages'

& $py311 .\tools\stage5_existing_python.py --site-packages $packages --probe
if ($LASTEXITCODE -ne 0) { throw 'Python / PyBullet / NumPy 导入失败；停止验收' }
```

如果 uv 解释器不在上述目录，先按实际安装位置调整 `$py311`；不要盲目改 Python 依赖。

### 3. 自动验收完整 ARM C APF 路线

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_embedded_apf_windows.ps1 `
  -Python $py311 -ExistingSitePackages $packages `
  -Qemu qemu-system-arm -TimeoutSeconds 300 `
  -OutputDir build/stage5-local-apf-acceptance
```

必须实际满足 `passed=true`、`route_complete=true`、完整路线逐段真实到位、`uart_checksum_errors=0`，且运行时每个已批准的关节运动完成全连杆检查。脚本通过后自动打包证据；失败时保留 `build/stage5-local-apf-acceptance/` 原始日志。

### 4. 人工 GUI 安全点动最终签收

**只有完整 ARM 路线 PASS 后**，再执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 `
  -Python $py311 -ExistingSitePackages $packages `
  -Qemu qemu-system-arm -Stage5RouteCase blocked `
  -SafeJog -RequireTwoStepJog -SafeJogHoldSeconds 600 `
  -OutputDir build/stage5-local-gui-acceptance
```

等待原始 4/4 航点到位进入 READY/HOLD，然后 **J6 +0.5° → 等待 RETURN COMPLETE → J6 +2° → 等待 RETURN COMPLETE → 再至少三次合法点动且每次独立回位**，最后点击 `SAFE JOG FINISH/EXIT`，不要直接点窗口 X。要求拒绝次数为 0、回位数与实际操作数相等、误差不超过 0.15°，并保存 `result.json`、`worker.log` 和审计记录。

请为重复验收使用**不同输出目录**，避免脚本用已有文件误认为本轮结果。完整流程、按钮语义和日志解释见 [Windows 人工验收指南](docs/part5/18_stage5_windows_final_two_step_acceptance.md)。

## 其他构建与回归入口

新建**全新开发环境**才参考 `requirements.txt`；上述 Windows 已有环境验收**不要**新建 `.venv` 或执行 `pip install`。需要 ARM 工具链、CMake、Ninja、QEMU、Python/PyBullet，GUI 还需可显示窗口的桌面会话。

```powershell
# 普通 ARM 固件构建（不同于启用 Stage5 UART 集成的验收 preset）
cmake --preset arm-qemu
cmake --build --preset firmware

# 已有 Stage4 自动回归
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1

# Stage4 PID 多工况及 FreeRTOS/QEMU 回归（更长的实验）
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
```

完整 Stage5 ARM 构建与 PyBullet 启动由上方现成 Windows PowerShell 验收脚本负责，不必再按 CI YAML 手动拼装几十条命令。

## 文档、结果和交接

- [参数登记簿](docs/part5/01_parameter_register.md)、[架构说明](docs/architecture.md)、[Stage5 问题跟踪](docs/part5/02_integration_issue_log.md)
- [Stage1–4 验收状态](docs/phase1-4_acceptance_status_2026-10-27.md)、[Stage4 历史封版](docs/part4/part4_archive/12_stage4_release_candidate_2026-10-28.md)
- [Stage5 集成与 GDB 性能报告](docs/part5/15_stage5_exit_review_2026-10-30.md)、[Stage5 原始要求逐条审计](docs/part5/19_stage5_original_requirements_evidence_review_2026-11-01.md)
- [Stage5 Windows 人工最终验收](docs/part5/18_stage5_windows_final_two_step_acceptance.md)、[Stage6 接口与任务约束](docs/part6/00_stage6_interface_and_acceptance_contract.md)

验收结论以**同提交实际 GitHub Actions 及本机保存的 `result.json`/`worker.log`/物理轨迹证据**为准。Stage5 的性能指标与 GUI 真人操作保留项须在最终报告明确写出，不能用 README 或一次绿色 CI 取代未完成的测试。
