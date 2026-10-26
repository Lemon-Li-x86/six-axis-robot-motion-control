# 第4阶段收尾：全局架构审查、技术债清理与阶段交接门禁

日期：2026-10-26  
范围：`firmware/freertos_demo`、`simulation/pybullet_ur5`、`tools/`、Stage3/4 Host、ARM/QEMU、PyBullet测试。  
代码分支：`feat/stage4-trajectory-2026-10-26`。**未合并 main；备份分支不改。**

## 1. 审查依据与验收等级

本次依据源码目录、`CMakeLists.txt`/CMakePresets、生产接口与对应测试，使用已发生的用户 Windows 一键验收反馈和 GitHub Actions 真实结果核对。证据按强度区分：

- **实现**：源码和构建目标存在，不等于编译或运行成功。
- **Host CI 通过**：原生算法测试实际完成，不能证明 ARM / FreeRTOS / 串口动作。
- **ARM/QEMU 通过**：Cortex-M4 + FreeRTOS 中的给定任务路径实际执行，不能证明真实硬件或 UART 外部闭环。
- **PyBullet 通过**：同源生产 C 轨迹/PID/限速算法与 URDF 仿真闭环执行，但绕开 UART/FreeRTOS。
- **系统验收**：须另测 UART 双向闭环、超时与异常、实时性、负载和硬件安全。**目前尚未满足该等级**。

历史证据：用户已反馈旧版 `tools/test_stage4_once.ps1` 本机全部通过；云端已完成 Host 6项与两种 PyBullet 场景，并有 `QEMU RTOS checks=11 failed=0; requests=1; trajectory=60; controller=118; monitor=12` 的旧版证据。**本次审查后的重构/新注入测试，必须由新CI独立证实，不复用旧日志充数**。

## 2. 最终目录职责（当前实际模块）

| 层级/目录 | 已实现职责 | 应有的依赖方向 | 审查 |
|---|---|---|---|
| `common/` | 公共角度/位置类型、单位、角度回绕、统一π/弧度换算 | 不依赖Robot Model/FreeRTOS | 符合；数学常量重复已清理 |
| `model/` | UR5 DH/基座工具转换/连续关节限位 | 只引用Common，不访问实时状态 | 符合；硬件动态极限仍未标定 |
| `algorithms/math/` | 4×4变换计算 | 纯算法 | 符合 |
| `algorithms/kinematics/` | FK、解析IK、参考条件化解、统一FK残差数学 | Model/Common/Math | 符合；解析奇异仍有限制 |
| `algorithms/motion/` | IK选解、连续关节限位/圈数展开 | Model/kinematics/Common | 符合；不是避障规划器 |
| `algorithms/trajectory/` | 关节多项式/梯形、笛卡尔线/圆弧与IK航点 | Motion/Kinematics/Common，无UART/RTOS | 符合；Cartesian路径尚无完整加速度重定时 |
| `algorithms/control/` | 增量式PID、六轴组合、命令速度+加速度限幅 | Model/Common/Trajectory类型，无硬件接口 | 符合；增益只做初始仿真整定 |
| `drivers/` | UART/定时器/GPIO/Motor Driver | Board/Common | 符合；实物硬件急停不存在 |
| `communications/` | 二进制协议和环形缓冲区 | Common/Driver界面 | 符合；串口指令尚未映射Stage4运动请求 |
| `application/services/` | 协议命令处理、运动管线、轨迹请求适配 | 算法/Driver/Tasks协调 | 符合；少量服务将来可继续拆 |
| `application/tasks/` | Trajectory/Control/Monitor周期任务与状态 | RTOS/Application/算法 | 符合；尚未做完整实时负载剖析 |
| `tests/` + `simulation/` | 原生、QEMU、物理仿真诊断 | 可依赖生产代码；生产不得依赖测试 | 符合；两类联调证据分离 |

## 3. 本轮已执行的去重和一致性修改

1. **统一FK误差数学**：`kinematics_pose_within_tolerance()` 新增在 `kinematics_common.c`、声明在 `kinematics.h`。条件化IK、Cartesian起点校验、轨迹航点回代共用同一个位置L2(mm)+旋转矩阵最大元素差计算；所有门限仍从原本所属算法/应用配置注入，拒绝非有限数与非法门限。修改了 Stage3 Host 回归以检查合法、越界、NaN、NULL 和非正阈值。
2. **统一角度数学常量**：将 UR5 Model、Kinematics 和 IK Selector 的 π/deg↔rad 字面量收敛到 `common/math_constants.h`。保留旧 Kinematics 宏作为兼容别名，数值只定义一次；不会为了整理常量改变 DH 模型/关节限位。
3. **参数登记簿重建**：`00_parameter_register.md` 不再把“未来接入PID”“采样周期尚无来源”等历史描述列为当前事实。现按 Model事实 / Common单位 / Algorithm门限 / Config策略 / Runtime状态 / Test Fixture 逐项登记唯一值源和消费者。
4. **跨语言协议防漂移**：新增 `tools/check_protocol_mirror.py`，逐项核对 Python UART 接收示例与 C 端帧头、目标角命令ID、6轴数量、raw角度缩放、12字节payload；在 GitHub Host CI 和 Windows 一键脚本开头执行。**这是兼容性回归，不是共享生成协议或物理串口闭环**。
5. **补充失联故障注入**：仅测试专用的 `stage4_rtos_selftest.c` 将在健康任务运行后停止注入反馈，延迟超过唯一 `feedback_timeout_ms`，检查错误计数和 Control Step 最终停止增长；扩充 `checks` 到13项，强制电机输出仍 OFF。须由 ARM/QEMU 云端验证此新版案例才能标绿。

## 4. 已核实、但本轮不盲改的风险

| 优先级 | 风险/缺口 | 源码定位与原因 | 必要后续处理 |
|---|---|---|---|
| P0：实体上电阻断 | Control故障时当前策略停止**继续发送**新目标，不具备实体驱动的独立紧急停机/失联制动机制 | `application/tasks/control_task.c`、`drivers/motor_driver.c`；当前 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF` | 设计驱动硬件停机/看门狗/故障复位联锁；此前**禁止实物上电执行** |
| P1 | UART指令没有 `motion_pipeline_submit_*()` 的正式命令映射 | `application/services/protocol_commands.c` 仅处理Joint State/Parameter/Diagnostics；`communications/protocol.h` 中现有 `CMD_SET_JOINT_TARGETS` 用于**设备发给外部的目标**，不是运动规划命令 | 另定可版本化请求/ACK/拒绝语义并端到端跑通，不复用反向目标帧误触发运动 |
| P1 | 目前“PyBullet通过”与“FreeRTOS QEMU通过”是两条互补的**独立**证据 | `tests/simulation/stage4_sim_bridge.c` 绕开串口；QEMU自检使用静止虚拟反馈 | 增加同一轮UART双向真实反馈→控制→目标验证，含拆包/丢包/超时 |
| P1 | 未测周期分位数、最坏执行时间、长时稳定性 | Monitor仅记录任务运行和栈水位；现有QEMU自检为短时冒烟 | 增加周期统计、P99/WCET定义；在系统负载下做长时试验 |
| P1 | 笛卡尔任务检查每次IK航点的位置/FK与速度跳变，但没有针对任意路径的完整加速度重定时/连续碰撞检验 | `trajectory_task.c` Cartesian分支；Output Limiter能限制命令加速度，但不代表几何轨迹参数化已满足关节动力学 | Stage5约束规划/静态避障时设计独立速度/加速度/碰撞门禁 |
| P2 | Python旧UART接收器与C协议存在镜像常量 | `qemu_uart_receiver.py` 与 `protocol.h` | 已加一致性回归；将来如需协议演进，迁移到单一schema+代码生成 |
| P2 | PID积分变量和字段名末尾 `_deg_s`，但按代码 `error_deg*dt_s` 积分后维度应是 **度·秒** | `algorithms/control/pid.[ch]` | 正式接口改名需全模块迁移及回归，暂保留旧ABI并在文档明确实际维度 |
| P2 | `docs/part4/02_stage4_full_implementation_guide.md` 与部分早期阶段文档保留历史测试/缺口描述 | 文档为追加式阶段演化记录 | 现以本报告和参数登记簿作为当前状态权威，下一阶段清理历史章节时保持可追溯 |

## 5. 全局结论与阶段门禁

**结构层面**：当前不需要大规模搬文件。主要模块已经处于目标架构中的正确位置，参数事实/算法/任务状态已基本隔离。本轮优先消除重复计算、重复数学字面量和文档漂移，以避免重构变成“重命名一大堆文件却没有解决实际问题”。

**软件质量层面**：截至审查前，Stage3/4 Host、QEMU RTOS任务、自有C算法PyBullet小幅运动均有通过证据。本次修订后的代码需新CI再验。通过后可以宣告**第4阶段核心固件功能及自动化非驱动回归收尾**。

**系统验收层面**：在完整UART双向闭环、失联故障注入、实时统计与长时稳定性门禁完成前，不能标注“第4阶段全系统验收通过”；更不能宣称具备实物工业机器人安全等级。第5阶段可以**进入规划/接口设计**，但不能用未验证的系统安全作为已具备前提。

**后续顺序（无需用户逐文件协助）**：最新CI Host→ARM/QEMU故障注入→PyBullet回归通过后，进入UART端到端协议及安全门禁独立开发；届时完成性能报告与再次全局审查，再合并/开启下一阶段。
