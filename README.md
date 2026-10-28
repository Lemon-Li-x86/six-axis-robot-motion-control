# 六轴工业机器人嵌入式运动控制固件与仿真验证

基于 **ARM Cortex-M4、FreeRTOS、QEMU 和 PyBullet** 的 UR5 六轴机器人运动控制项目。使用 C 实现运动学、轨迹规划和 PID 控制，通过 QEMU 运行嵌入式固件，并在 PyBullet 中完成运动仿真与反馈验证。

**当前进度：Stage 1–4 核心功能完成，Stage 4 Windows 本机验收通过；Stage 5 系统集成开发中。** 项目以软件仿真为目标，不使用实体机器人。

## 项目进度

| 阶段 | 主要内容 | 进度 |
|---|---|---|
| Stage 1 · 基础环境 | Cortex-M4 / FreeRTOS 工程、QEMU、PyBullet UR5、UART 双向通信 | 已完成 |
| Stage 2 · 驱动与协议 | UART / Timer / GPIO 驱动、Ring Buffer、协议解析、六轴电机状态抽象 | 已完成 |
| Stage 3 · 运动学 | UR5 正逆运动学、IK 选解、连续角处理、关节限位 | 已完成 |
| Stage 4 · 运动控制 | 三次/五次多项式、梯形速度、笛卡尔直线/圆弧轨迹、六轴增量式 PID、FreeRTOS 控制任务 | 核心验收通过，封版候选 |
| Stage 5 · 系统集成 | QEMU UART ↔ PyBullet 动态联调、GUI 交互、静态障碍物避障与性能优化 | 进行中 |
| Stage 6 · 综合应用 | 搬运任务仿真、长时间运行及最终交付 | 待开发 |

详见 [Stage 1–4 验收记录](docs/phase1-4_acceptance_status_2026-10-27.md)和 [Stage 5 任务与进度](docs/part5/00_stage5_scope_and_acceptance.md)。

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
- [Stage 4：封版记录、性能数据与待办](docs/part4/12_stage4_release_candidate_2026-10-28.md)
- [Stage 5：集成问题记录](docs/part5/02_integration_issue_log.md)

Stage 4 当前为 [封版候选 PR #4](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/4)；Stage 5 的 GUI 与 UART 联调改动在独立的 [PR #3](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/3) 中。
