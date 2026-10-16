# 六轴工业机器人运动控制项目：架构与扩展边界

> 本文描述当前实现及后续设计约束，不将计划中的功能标为已实现。
> 项目总体阶段目标以《六轴工业机器人嵌入式运动控制固件开发与仿真验证》为准。

## 1. 运行链路与现状

当前链路：

```text
Cortex-M4 / FreeRTOS (QEMU)
    ProtocolTX ──UART/TCP──> Python bridge ──> PyBullet UR5
    ProtocolRX <─UART/TCP─── Python bridge <── 关节状态
```

当前已实现的主要模块包括驱动、通信协议、运行参数、监测、UR5 模型、
FK / 解析 IK、关节角连续展开与限位判定。存在 Motion Limits 算法和
QEMU 测试目标，不等于已接入运动指令执行链。

当前实际创建的 FreeRTOS 业务任务是 ProtocolRX 和 ProtocolTX；
配置中预留 Control、Trajectory、Monitor 等参数不代表任务已实现。
运动学启动自测和 Parser Benchmark 已通过 CMake 选项控制。

## 2. 固件模块与所有权

| 当前目录 | 职责 | 约束 |
| --- | --- | --- |
| `common/` | 公共类型、单位、错误码 | 不引入 FreeRTOS 或业务状态 |
| `model/` | 机器人静态参数、关节约束 | 不保存实时目标和反馈 |
| `drivers/` | UART/GPIO/Timer/Motor 硬件与仿真驱动边界 | 不调用轨迹或 IK 策略 |
| `communications/` | 帧编解码、字节流解析、缓冲 | 不直接决定运动决策 |
| `algorithms/math/` | 通用数学工具 | 不依赖应用、驱动和 RTOS |
| `algorithms/kinematics/` | FK、IK 候选解 | 不负责轨迹时序和任务调度 |
| `algorithms/motion/` | 限位、角度展开、未来的逆解选择 | 不直接驱动电机或发送协议 |
| `application/protocol_tasks.c` | FreeRTOS RX/TX 任务、ISR 通知及解析循环 | 不新增命令业务分支 |
| `application/services/protocol_commands.c` | 完整帧业务分发、反馈 ACK、参数与诊断请求 | 不创建任务或实现字节流解析 |
| `application/runtime_config.c` | 运行时配置的校验与生效值 | 避免与 JSON 默认配置出现重复定义 |
| `application/uart_tx_manager.c` | 完整帧发送串行化 | 不实现具体消息业务 |
| `diagnostics/` | 运行性能度量 | 不控制机器人运动 |
| `tests/` | 自测及 QEMU 测试入口 | 测试不得反向成为核心算法依赖 |

当前 Motor Driver 仍持有目标与反馈（含连续位置、速度估计）。
这是**已知待设计边界**，不是立刻创建独立 Robot State 模块的理由。
第四阶段加入多个任务共享状态时，应一次性定义所有权、采样时序、
并发同步、反馈有效性与故障策略，再决定是否提取状态服务。

## 3. 未来开发按项目阶段引入

- **阶段 3 收尾**：IK 解的有效性、关节限位与就近连续解选择；算法接口采用明确的角度单位和约束，执行前另作集成验证。
- **阶段 4**：真正开始轨迹算法时才创建 `algorithms/trajectory/`；
  PID 实现时才创建 `algorithms/control/`；创建对应 FreeRTOS Task
  之前先约定周期、超时、数据快照及线程同步。
- **阶段 5**：实现规划/避障时再考虑 `algorithms/planning/`，
  不提前生成空目录；继续保持 QEMU、Python Bridge 和机器人模型的边界。
- **阶段 6**：把可复现的构建、回归测试、精度及实时性证据纳入交付。

轨迹规划的输出与驱动输入之间必须定义**连续角/规范角、单位、限位、
时间戳和异常处理**，不可直接把 IK 候选解视为可执行命令。

## 4. 模块依赖约束

```text
application (FreeRTOS 任务/服务)
   ├─> communications (协议)
   ├─> algorithms (纯算法)
   ├─> model (静态参数)
   ├─> drivers (硬件接口)
   └─> diagnostics (指标读取)

algorithms ──> common / model / math
drivers ─────> common / 平台相关实现
communications ──> common
```

原则是核心算法不反向依赖 Application、FreeRTOS、UART 或 Python Bridge。
当前 `protocol_commands.c` 属于**应用服务**，尽管它直接调用 Motor Driver、
Runtime Config 和 Diagnostics，但不应被误判为纯协议解析模块。

## 5. 不再为了行数拆文件

创建新模块至少满足下列条件之一：不同变化原因、独立测试需求、
独立生命周期/线程所有权、依赖方向明显需要隔离。
否则优先在原文件内部使用清晰的 `static` 函数。
不使用 `xxx_new.c`、`xxx_final.c` 等版本副本；历史版本交给 Git。

结构调整必须保留公共接口，或明确列出接口迁移和使用方；
不得把文档规划当作已通过 QEMU 或物理硬件验证的证据。
