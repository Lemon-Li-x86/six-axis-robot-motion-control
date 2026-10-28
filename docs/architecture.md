# 六轴机器人嵌入式控制系统架构（Stage4 仿真 RC）

> 更新：2026-10-28。本文描述 `release/stage4-simulation-rc-20261028` 的**当前实现**，取代先前“只创建 ProtocolRX/ProtocolTX、Control Task 尚未实现”的历史状态描述。原始设计及演进过程仍保存在 `docs/part1/` 和 `docs/part4/`。
>
> 本工程无实体机器人；“已运行”限定为 Host / Cortex-M4 QEMU / PyBullet 试验场景，不意味着具备实物驱动、硬件急停或工业机器人安全认证。

## 1. 分层与数据流

```text
┌─────────────── QEMU Cortex-M4 + FreeRTOS ────────────────┐
│ ProtocolRX  ──> Protocol parser ──> Application commands │
│                                        │                │
│                                        ▼                │
│                                  Motion Pipeline        │
│                                        │                │
│                           Request Queue / Generation   │
│                                        ▼                │
│ Trajectory Task (20 ms) <─ C 轨迹 / FK/IK / 约束         │
│                                        │                │
│                              Reference Queue            │
│                                        ▼                │
│ Control Task (10 ms) <──── Motor State + 新鲜反馈        │
│          │   C 增量式 PID + 速度/加速度命令限幅           │
│          ▼                                              │
│ Motor Target State ──> ProtocolTX ──> UART               │
│                                      ↑                   │
│ Monitor Task (100 ms) ─── 任务/栈/故障诊断                │
└──────────────────────────────────────┼───────────────────┘
                                       ⇅
                              QEMU UART TCP backend
                                       ⇅
                              Python 仿真通信桥
                                       ⇅
                                PyBullet UR5
                             实际关节状态/动力学
```

这是组件与 Stage5 完整联调所使用的逻辑结构。**Stage4 默认非驱动测试和 Stage5 集成是不同运行模式**：

- Stage4 `tools/test_stage4_once.ps1` 把物理电机命令输出编译门控保持 `OFF`，QEMU 测规划、PID计算及任务调度，真实 PyBullet DIRECT 则通过**生产同源 C 算法共享库**执行动力学实验，两者不能合并冒称同一次串口动态闭环。
- Stage5 显式仿真编译开关打开时才运行 QEMU UART ↔ PyBullet 反馈闭环；本 RC 中的历史候选集成测试并未涵盖最新原生 Windows GUI 拖拽补丁，详见 [Stage5 范围](part5/00_stage5_scope_and_acceptance.md)。
- TCP 是 QEMU 对主机暴露 UART 的承载方式，不是嵌入式自定义控制协议。

## 2. 源码职责和所有权

| 模块路径（相对 `firmware/freertos_demo/`） | 负责什么 | 不应负责什么 |
|---|---|---|
| `common/` | 角度/连续位置、统一错误码、单位与公共数学量 | 不拥有实时运动状态 |
| `model/` | UR5 静态模型、坐标变换与关节行程 | 不保存运行时电机目标 |
| `drivers/` | QEMU UART、Timer、GPIO Shadow、Motor Driver 抽象 | 不做 PID、逆解或路径规划 |
| `communications/` | 二进制协议、帧校验、字节流解析及缓存 | 不为运动任务选轨迹 |
| `algorithms/math/` | 坐标变换/通用数学 | 不依赖 FreeRTOS |
| `algorithms/kinematics/` | FK、解析 IK、候选解可用性检查 | 不接管串口或电机 |
| `algorithms/motion/` | IK 选解、连续角展开、关节限位 | 不规划障碍物整臂路径 |
| `algorithms/trajectory/` | 多项式/梯形关节运动、Cartesian 直线/圆弧离散插补 | 不拥有 FreeRTOS 队列 |
| `algorithms/control/` | 单关节增量 PID、六轴 Controller、命令速度/加速度限幅 | 不使用 Python 模拟另一套控制器 |
| `application/services/` | 协议动作分发、Motion Pipeline、请求合法性/接纳 | 不复制轨迹或 PID 数学 |
| `application/tasks/` | Trajectory/Control/Monitor 独立周期任务 | 不自行定义另一套静态机器人参数 |
| `application/motor_state.c` | 互斥访问六轴反馈及目标、采样序号和反馈新鲜度 | 不对外暴露未同步的裸状态供业务任务操作 |
| `diagnostics/` | 周期/负载样本、诊断数据读取 | 不改变运动规划 |

任务周期/优先级/栈、初始状态、增益、速度/加速度/反馈超时的唯一**默认配置**是 `config/robot_defaults.json`，生成器为 `tools/generate_firmware_defaults.py`。UR5 DH 与关节范围仍以 Model 为唯一来源；QEMU 性能判据和物理仿真实验的测试常量属于独立测试策略，不作为生产参数。

## 3. 运行路径与异常

1. **接收**：UART RX 中断将字节送入 Ring Buffer，ProtocolRX 组帧与校验；Application Service 根据命令语义解析并给出明确错误状态。
2. **请求**：`motion_pipeline_submit_joint/polynomial/cartesian` 预检请求，在请求队列中提交最新运动；UART `MOVE_ACK` 如存在仅证明请求被接受，不证明轨迹已走完。
3. **轨迹**：Trajectory Task 以规定周期进行梯形/多项式采样，或笛卡尔 IK 航点转换；一旦出现非法参数、规划失败、失联或控制器要求中止，取消参考，不能沿用旧目标假装继续规划。
4. **控制**：Control Task 读取反馈及创建时间，按有效参考运算同源 C PID 与输出限制，更新经 Motor State 保护的目标；反馈陈旧、参考过期及控制异常应停发新目标并记录故障。
5. **仿真**：在显式开通的虚拟联调中，Python 将 QEMU UART 目标映射至 PyBullet 六轴，实际关节角经反馈帧返回。规范角与连续角域必须区分，避免 ±360° 的错误距离。
6. **观察**：Monitor 与诊断指标输出任务计数、故障、栈水位及 QEMU 计时；计量只对指定运行条件有效。

**实机安全边界**：当前故障停止新目标的策略**不是**独立硬件制动、急停或驱动级联锁。在没有实体驱动、看门狗/制动器和安全验证之前，不得将实验输出直接用来驱动实体机器人。

## 4. 编译模式及验收含义

| 运行方式 | 验证内容 | 不覆盖 |
|---|---|---|
| Host CTest | FK/IK、路径、多项式极值/限幅、PID 公式和异常输入 | ARM调度、真实UART吞吐 |
| Stage4 ARM/QEMU 自检 | Cortex-M4 FreeRTOS 任务、静态反馈、失联拒绝、算法路径 | PyBullet运动动力学 |
| Stage4 Native C + PyBullet DIRECT | **同源 C** 轨迹/PID驱动仿真关节与误差分析 | FreeRTOS/GDB/UART动态链 |
| Stage5 双向 QEMU UART + PyBullet | 独立串口接收运动请求、虚拟反馈、系统级故障处理 | 实物 MCU、整臂碰撞安全与工业认证 |

封版候选的测试数据和准确版本以 [Stage4 RC 验收单](part4/12_stage4_release_candidate_2026-10-28.md) 为准；**RC 不是已经打了正式 Release Tag**。当前完整 Stage5 GUI 交互的试验与问题追踪以独立 PR 和 [Stage5 Issue Log](part5/02_integration_issue_log.md) 为准。

## 5. 维护约束

- Core C 算法不可反向依赖 FreeRTOS、UART、Python 或 PyBullet。
- 配置、常量、派生量、实时状态、测试输入严格分开；同一个参数只设一个权威来源。
- 不为了缩短源文件复制模块或另写 Python 控制器；保持现有 API、任务和 Queue 所有权。
- 任何“修复完成”的判断必须绑定提交 SHA、真实可复现的测试与覆盖边界。Stage4 已通过的证据不得直接覆盖 Stage5 仍失败的 UART/GUI 集成测试。
