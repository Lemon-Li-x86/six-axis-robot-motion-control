# 六轴工业机器人嵌入式运动控制固件与仿真验证

基于 **ARM Cortex-M4、FreeRTOS、QEMU 和 PyBullet** 的 UR5 六轴机器人运动控制项目。使用 C 实现运动学、轨迹规划和 PID 控制，通过 QEMU 运行嵌入式固件，并在 PyBullet 中完成运动仿真与反馈验证。

**当前进度：Stage 1–4 已验收；Stage 5 的 QEMU/FreeRTOS/UART ↔ PyBullet 静态避障核心闭环已通过 CI，原生 Windows GUI 与最终范围签收尚待完成。Stage 6 综合搬运任务未开发完毕。** 项目以软件仿真为目标，不使用实体机器人。

## 项目进度

| 阶段 | 主要内容 | 进度 |
|---|---|---|
| Stage 1 · 基础环境 | Cortex-M4 / FreeRTOS 工程、QEMU、PyBullet UR5、UART 双向通信 | 已完成 |
| Stage 2 · 驱动与协议 | UART / Timer / GPIO 驱动、Ring Buffer、协议解析、六轴电机状态抽象 | 已完成 |
| Stage 3 · 运动学 | UR5 正逆运动学、IK 选解、连续角处理、关节限位 | 已完成 |
| Stage 4 · 运动控制 | 三次/五次多项式、梯形速度、笛卡尔直线/圆弧轨迹、六轴增量式 PID、FreeRTOS 控制任务 | 核心验收通过，封版候选 |
| Stage 5 · 系统集成 | 真实 ARM/FreeRTOS UART ↔ PyBullet 静态整臂避障、CPU/GDB/.map 审计；GUI 尚需独立验收 | 无头核心通过；阶段全项未签收 |
| Stage 6 · 综合应用 | 搬运任务仿真、长时间运行及最终交付 | 待开发 |

详见 [Stage 1–4 验收记录](docs/phase1-4_acceptance_status_2026-10-27.md)、[Stage 5 原始验收范围](docs/part5/00_stage5_scope_and_acceptance.md)与 [Stage 5 结项证据/尚存限制](docs/part5/15_stage5_exit_review_2026-10-30.md)。

## 系统架构

```text
      Cortex-M4 固件（QEMU + FreeRTOS）
                       │
         协议 / 运动请求 / 状态管理
                       │
      C 运动学 → C 轨迹规划 → C PID
                       │
               UART 双向通信
                       │
              Python 通信桥接
                       │
                PyBullet UR5
             关节执行 / 状态反馈
```

运动学、轨迹和控制算法使用同一套 C 实现，既可进行 Host 单元测试，也可由固件调用。PyBullet 负责仿真执行与实际关节反馈。

主要技术：**C11、FreeRTOS、CMake、ARM GNU Toolchain、QEMU、UART、Python、PyBullet、GDB**。

## 代码结构

```text
firmware/freertos_demo/
├── common/            公共类型、单位与错误码
├── model/             UR5 模型与关节约束
├── drivers/           UART、Timer、GPIO、Motor
├── communications/    二进制协议与 Ring Buffer
├── algorithms/
│   ├── kinematics/    FK / IK
│   ├── motion/        选解、连续角与限位
│   ├── trajectory/    关节与笛卡尔轨迹
│   └── control/       PID 与命令限幅
├── application/       FreeRTOS 任务与运动请求服务
└── config/            统一默认参数

simulation/             PyBullet 场景与 UART 仿真桥
tools/                  构建、回归与分析脚本
docs/                   阶段设计、测试和验收文档
```

任务周期、六轴 PID、速度/加速度与超时等默认参数统一由 [robot_defaults.json](firmware/freertos_demo/config/robot_defaults.json) 管理。完整模块关系见 [架构说明](docs/architecture.md)。

## 快速开始（Windows）

环境需要 Python、CMake、C/C++ 编译器、ARM GNU Toolchain、QEMU 和 GDB；固件构建使用 Ninja。

安装 Python 依赖：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

构建 Cortex-M4 固件：

```powershell
cmake --preset arm-qemu
cmake --build --preset firmware
```

运行 Stage 1–4 自动化回归：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
```

运行 Stage 4 的 PID 多工况分析及 FreeRTOS/QEMU 回归：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
```

后一命令会执行多轮参数搜索，输出及实验文件保存在 `build/stage4-closure-local/evidence/`，完整非驱动回归日志保存在 `build/stage4-final-validation.log`。

## 测试结果与文档

2026-10-28 的 Stage 4 Windows 本机验收记录包括：

- Host CTest **6/6**、原生 C 桥接测试 **1/1**；
- PID 仿真实验 **18/18** 正常执行，候选参数的保留测试条件 **6/6** 达标；
- FreeRTOS 自检 **29/29**、多项式测试 **8/8**；
- PyBullet DIRECT 的单关节与六轴动态回归通过。

当前六轴仿真 PID 候选配置为 **Kp = 0.25，Ki = 0，Kd = 0.025**。

- [Stage 1：系统设计与通信环境](docs/part1/01_system_design.md)
- [Stage 2：驱动与协议测试](docs/part2/03_unit_and_function_test_report.md)
- [Stage 3：运动学验证](docs/part3/02_algorithm_validation_report.md)
- [Stage 4：PID参数调优与轨迹性能分析](docs/part4/01_pid_tuning_and_trajectory_performance.md)
- [Stage 4：多任务调度与实时性验证](docs/part4/02_multitask_scheduling_and_realtime_verification.md)
- [Stage 4：历史封版记录](docs/part4_archive/12_stage4_release_candidate_2026-10-28.md)
- [Stage 5：集成问题记录](docs/part5/02_integration_issue_log.md)
- [Stage 5：完整闭环、性能和 GDB 验收报告](docs/part5/15_stage5_exit_review_2026-10-30.md)
- [Stage 6：未来任务编排和边界约束](docs/part6/00_stage6_interface_and_acceptance_contract.md)

Stage4 验收备份保持独立不变。Stage5 无头闭环和时序竞争修复分别见已合并的 [PR #25](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/25) 与 [PR #26](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/26)。原生 GUI 实验 [PR #15](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/15) 尚未通过完整验收，不应和主线成功的 PyBullet DIRECT 测试混淆。
