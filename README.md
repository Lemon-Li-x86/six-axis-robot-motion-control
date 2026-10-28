# 六轴工业机器人嵌入式运动控制固件与仿真验证

基于 **ARM Cortex-M4 / FreeRTOS / CMake / QEMU / UART / Python / PyBullet** 的六轴 UR5 运动控制项目。核心运动学、轨迹和控制算法以 **C** 实现，运行于宿主机算法测试或 Cortex-M4 固件；PyBullet 用于动力学反馈与仿真验证，而不是另一套 Python 运动规划器。

> **状态（2026-10-28）**：第1至第4阶段的**软件仿真核心功能已实现，本机 Stage4 RC 验收通过**；第四阶段正式封版仍待交付归档、CI 例外评审与 Tag 签署。第5阶段系统集成正在独立开发；**第6阶段尚未验收**。本项目没有实体机器人，所有定位精度和实时性数据仅适用于明确列出的仿真/测试工况。
>
> 当前 README 描述 `release/stage4-simulation-rc-20261028` 的实际范围。Windows GUI 原生拖拽、HOLD 回位、额外 UART 时钟同步等 **Stage5 试验代码不在本 RC 中**，见 [Stage5 PR #3](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/3)；第四阶段封版见 [PR #4](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/4)。

## 一、阶段进度

| 阶段 | 已实现的工作 | 验收状态与边界 |
|---|---|---|
| **Stage 1：方案与环境** | 软件分层；Cortex-M4/FreeRTOS/CMake/QEMU 工程；UR5/PyBullet；QEMU UART ↔ Python 基础双向通讯 | **基础仿真完成**，不代表实物板级移植 |
| **Stage 2：驱动与通信** | UART 收发中断、TX/RX 环形缓冲、二进制协议解析及错帧恢复；Timer/GPIO、六轴电机状态抽象、运行参数与错误码 | **Host 与 ARM/QEMU 指定回归通过**；GPIO 为 Shadow 实现，电机为仿真状态抽象 |
| **Stage 3：运动学与选解** | UR5 FK、解析 IK、候选解筛选、连续角度展开、关节行程约束与近奇异检查 | **128 组 ARM/QEMU FK→IK→选解→FK 回代与参考条件化近奇异样本通过**；不等于全工作空间证明 |
| **Stage 4：轨迹、PID、调度** | 三次/五次多项式、梯形/三角速度、笛卡尔直线/圆弧插补；六轴增量式 PID、输出速度/加速度限幅；FreeRTOS Trajectory / Control / Monitor 任务 | **核心完成，Windows 本机封版回归通过，正式 Tag 未发布**；性能仅按所测仿真场景声明 |
| **Stage 5：系统集成与扩展** | 真实模拟 UART ↔ FreeRTOS ↔ PyBullet 运动请求/反馈联调、原生 GUI 扰动与恢复探索；静态障碍物 C 人工势场实验 | **进行中，独立分支**；整臂避障、异常长稳及部分集成回归尚未验收 |
| **Stage 6：最终应用与交付** | 完整搬运任务、长时间稳定性和最终交付 | **未验收** |

分阶段的**完成**专指已有代码、明确场景与对应测试通过，不表示每一个工况都满足工业级精度或实机安全要求。详细记录见 [Stage1–4 总体验收](docs/phase1-4_acceptance_status_2026-10-27.md)、[Stage4 封版验收单](docs/part4/12_stage4_release_candidate_2026-10-28.md) 与 [Stage5 范围及验收](docs/part5/00_stage5_scope_and_acceptance.md)。

## 二、系统架构

```text
                    主机 Host / 测试环境
   CMake + ARM GNU Toolchain ──> Cortex-M4 ELF
                                        │
                              QEMU mps2-an386
                                        │
          ┌──────────────────────── FreeRTOS ─────────────────────────┐
          │ ProtocolRX / ProtocolTX       Trajectory / Control / Monitor
          │         │                          │           ▲           │
          │    协议解析/诊断 ──> Motion Pipeline ──> C 规划/PID/限幅     │
          │         ▲                                      │           │
          │         └────── Motor State / 六轴反馈 <───────┘           │
          └────────────────────── UART ───────────────────────────────┘
                                        ⇅
                              QEMU 串口 TCP 后端
                                        ⇅
                              Python UART 桥接/观测
                                        ⇅
                           PyBullet UR5 关节动力学
```

上图是**模块关系与 Stage5 集成目标**，并不表示所有图示路径已在 Stage4 默认固件中打开：Stage4 非驱动回归强制关闭命令输出；完整 UART 动态联调使用显式隔离的仿真编译门控。通信仍是 MCU 侧 **UART 二进制帧**，TCP 只是 QEMU 将串口连接到主机的后端。

- **唯一来源**：UR5 机器人几何及关节范围由 `model/` 管理；角度单位、公共类型由 `common/` 定义；任务周期、栈、六轴 PID、速度/加速度、反馈超时等生产默认值由 `firmware/freertos_demo/config/robot_defaults.json` 生成。测试的姿态、力矩和门限是**测试夹具**，不能反过来成为生产参数。
- **数据含义**：协议规范角与连续关节角分开；关节原始角度通常为 `0.01°` 单位。IK 选解和运动轨迹必须在连续角域及限位条件下校验。
- **依赖边界**：`algorithms/` 为不依赖 FreeRTOS/UART 的 C 算法；`application/` 负责任务和运动状态；`drivers/` 不实现轨迹/PID；PyBullet 不替代生产 C 控制器。

更多说明：[现行架构与模块所有权](docs/architecture.md)、[Stage4 参数登记簿](docs/part4/00_parameter_register.md)。

## 三、代码与文档位置

| 目录 / 文件 | 用途 |
|---|---|
| `firmware/freertos_demo/common/`、`model/` | 公共数据类型、单位、UR5 模型、关节限位 |
| `firmware/freertos_demo/drivers/`、`communications/` | UART/Timer/GPIO/Motor 仿真驱动、Ring Buffer、协议 |
| `firmware/freertos_demo/algorithms/{math,kinematics,motion}/` | 矩阵计算、FK/IK、候选解选择与限位 |
| `firmware/freertos_demo/algorithms/{trajectory,control}/` | 关节/笛卡尔轨迹、增量式 PID、输出限制 |
| `firmware/freertos_demo/application/{services,tasks}/` | 运动请求管线、Trajectory/Control/Monitor 周期任务 |
| `firmware/freertos_demo/config/robot_defaults.json` | 任务、PID、速度/加速度、超时等生产配置唯一来源 |
| `simulation/pybullet_ur5/`、`simulation/uart_bridge/` | UR5 仿真、动力学验证与 UART 桥 |
| `tools/`、`tools/tests/`、`firmware/freertos_demo/tests/` | Windows 一键验收、Host/ARM/QEMU 回归与仿真试验 |
| `docs/part1/` ... `docs/part5/` | 按阶段归档的设计、测试、缺陷和验收文档 |

历史文档保留当时的版本语境。例如某份 Stage1 文档中“Control Task 尚未创建”是**历史状态**，不代表当前实现。

## 四、环境与快速运行（Windows）

测试记录使用 Windows、CMake / CTest、Visual Studio C++ 或其它本机 C 编译器、ARM GNU Toolchain、QEMU、GDB、Python 虚拟环境及 PyBullet；ARM 工程使用 Ninja，QEMU 目标为 `mps2-an386`。

先安装本机的 C/C++ 编译器、`cmake`、`arm-none-eabi-gcc`、`qemu-system-arm`、`arm-none-eabi-gdb`（或脚本可用的 GDB），创建并安装 Python 依赖：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

**只检验 Stage4 的原生 C 算法与 PyBullet 调参实验**：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1
```

**复现第1至第4阶段的完整非驱动本机回归**：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
```

**Stage4 封版组合入口**（在本仓库 RC 工作树运行）：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
```

`-IncludeQemuBaseline` 是额外执行既有 QEMU/FreeRTOS 回归，**不是**打开真机或完整 Stage5 GUI 鼠标交互。该入口会对不同 PID、运动工况重复运行较多次 PyBullet 测试，终端出现重复 `CALIBRATE n/30`、`VALIDATE` 属正常的有限参数搜索，不是循环卡死。脚本使用 `.venv\Scripts\python.exe`，完成后输出：

```text
[Stage4] Native closure study completed.
```

证据默认保存在 `build/stage4-closure-local/evidence/` 与 `build/stage4-final-validation.log`。`build/` 不入库，正式交付应另归档原始 JSON/CSV、测试日志和对应源码 SHA。

**仅需编译 ARM 固件**：

```powershell
cmake --preset arm-qemu
cmake --build --preset firmware
```

该命令构建 Cortex-M4 ELF；默认编译门控不等于启用完整运动执行。Stage1 的固定目标 UART 演示可从 [系统方案](docs/part1/01_system_design.md) 和 [环境/通信验证](docs/part1/04_env.md) 查阅，不应再把它作为当前全系统演示入口。Stage5 UART/GUI 运行命令、故障证据及对应分支以其独立 PR/文档为准。

## 五、已经取得的测试证据

| 范围 | 有据可查的结果 | 不能据此宣称 |
|---|---|---|
| Stage2 驱动/协议 | QEMU 协议 8/8、UART TX 7/7、Motor 11/11、GPIO Shadow 13/13 | STM32 真机 UART、实物 GPIO、电机硬件驱动 |
| Stage3 运动学 | 128 组 ARM/QEMU FK/IK 及选解回代通过；参考近奇异 2/2 | 任意工作空间/奇异附近的无条件连续解 |
| Stage4 算法/任务 | Host 6/6，ARM 轨迹 20/20，FreeRTOS 自检 29/29、多项式路径 8/8 | 真实硬件调度 WCET、30 分钟满载 |
| Stage4 C + PyBullet | Windows 本机 J2/六轴 DIRECT 测试通过，18/18 PID 研究工况运行成功；16/18 达到跨工况借用的小幅性能门限 | 18/18 工况的运动质量都符合同一个标准 |
| Stage4 候选增益 | 60 校准 + 20 独立验证，最佳候选测试条件 6/6，最坏超标比约 0.76733 | 全工作空间最优 PID 或真实电机标定 |

当前 RC 的 **Windows 本机通过版本**：`6cedeff99fac67b6aa4a25ccaa18909722164ccc`（2026-10-28，工作树干净）。此后纯文档提交不应冒充同一次本机测试的源码 SHA。候选唯一配置为**每轴 `Kp=0.25, Ki=0, Kd=0.025`**，历史初始方案 `0.2/0/0` 见旧调参记录。

[候选基线 CI #40498695392](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40498695392) 的三项 Job 已通过。**本 RC 首次 CI #40633371737 并非全绿**：Linux 与 Windows 的 Stage4 原生工作项通过，额外的 Stage5 模拟 UART 三段集成测试在 210s 内未完成首段（0/3）；单次 UART 测试通过。该集成失败应保留在 Stage5 问题追踪中，不能假称“整体 CI 成功”，也不抹掉 Stage4 已通过的非驱动测试。

实时性报告中 Control/Trajectory/Monitor 的 P99 采样和栈水位属于 **ARM/QEMU 仿真观测**，不是目标真实 MCU 独占 CPU 占比或 WCET：[Stage4 实时性数据](docs/part4/08_stage4_qemu_timing_and_stack_2026-10-27.md)。

## 六、交付、限制与后续开发

- **阶段4交付一：规划与运动控制源码**，见 `algorithms/trajectory/`、`algorithms/control/`、`application/tasks/`。
- **阶段4交付二：PID 调优和轨迹性能证据**，见 [历史调优](docs/part4/03_pid_tuning_record.md)、[18组实验](docs/part4/10_stage4_pid_sensitivity_study.md)、[候选验证](docs/part4/11_stage4_pid_candidate_verification_2026-10-27.md)。
- **阶段4交付三：多任务与实时性报告**，见 [任务验证](docs/part4/04_realtime_verification.md)、[QEMU 周期与栈实测](docs/part4/08_stage4_qemu_timing_and_stack_2026-10-27.md)。
- **封版状态及例外**：[Stage4 RC 验收单](docs/part4/12_stage4_release_candidate_2026-10-28.md)；目前仍为候选，尚无最终版本 Tag。
- **Stage5 工作**：[Stage5 范围及验收](docs/part5/00_stage5_scope_and_acceptance.md)、[集成问题记录](docs/part5/02_integration_issue_log.md)。GUI 拖拽 / HOLD 的独立实验见 [PR #3](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/3)。

本项目**没有实体机器人**。所有仿真控制与性能数据不构成实物安全认证；没有经过真实驱动、独立停机/联锁、完整碰撞/限位与硬件时序验证，**禁止直接将本项目实验固件用于工业实体机器人动作**。
