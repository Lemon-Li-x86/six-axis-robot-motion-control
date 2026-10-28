# 第4阶段参数登记簿（权威版本）

更新时间：2026-10-27。适用分支：`feat/stage5-system-integration-2026-10-27`。本登记簿替代前期逐段追加的“未来实现”“尚未接入”等过时条目；**实际定义位置以源代码为准**，本文不作为第二份可编辑配置。

## 所有权和单位规则

- **机器人事实 Model**：UR5 几何、基座/工具坐标变换及模型关节行程，唯一值源于 `model/ur5_model.c`，其他层只能通过 `robot_model_get_active()` 引用。
- **公共约定 Common**：关节数量、Canonical/Continuous 编码、单位、纯数学常量，唯一源于 `common/robot_types.h` 与 `common/math_constants.h`。
- **数值算法门限 Algorithm**：只服务具体算法的稳定性/选解，定义在各自算法模块；不能被默认为真实硬件安全能力。
- **策略配置 Config**：Stage4默认PID、速度、加速度、超时和Cartesian起点容差，以及任务静态默认值统一由`config/robot_defaults.json`生成；`stage4_motion_config.c`只装配生成配置并引用算法派生门限，再注入Application。
- **运行状态 Runtime**：当前反馈、活动轨迹、队列、PID/限速器状态不能保存在 Model、Const 或 Config 内；由 Task、Motor State、Driver 拥有。
- **测试策略/样本 Fixture**：PyBullet 测试构型、单次试验接受阈值、GDB 检查计数属于 `tests/` 或 `simulation/`，不能复制回生产控制策略。

## 1. 本体事实与单位（唯一来源）

| 参数 / 数据 | 语义与单位 | 定义 / 所有者 | 主要消费者 | 类别 |
|---|---|---|---|---|
| `ROBOT_JOINT_COUNT` | 6轴，个 | `common/robot_types.h` | 所有六关节数组和循环 | 公共不可变常量 |
| `ROBOT_JOINT_ANGLE_UNIT_DEG` | 1 raw = 0.01° | 同上 | Driver、UART、IK、轨迹、控制限速器 | 公共编码常量 |
| `ROBOT_JOINT_HALF_TURN_RAW`、`FULL_TURN_RAW` | 18000、36000 raw | 同上 | `angle_utils`、Motion、IK | 公共编码常量 |
| `robot_joint_angles_t` | `int16_t` 的 Canonical 六轴角，[-180°,180°) | 同上 | 协议、FK/IK、Motor State | 公共类型 |
| `robot_joint_positions_t` | `int32_t` 六轴连续位置，不在±180°回绕 | 同上 | 轨迹、PID、反馈 | 公共类型 |
| `robot_joint_velocities_t` | 关节速度 °/s | 同上 | 轨迹、限速器 | 公共类型 |
| `trajectory_joint_accelerations_t` | 关节加速度 °/s² | `algorithms/trajectory/trajectory.h` | 多项式、梯形、限速器 | 接口类型 |
| `robot_transform_t` | `base_link→ee_link`，平移 mm、旋转 3×3 | `common/robot_types.h` | 运动学、笛卡尔轨迹 | 公共几何类型 |
| `ROBOT_PI_F`、`ROBOT_DEG_TO_RAD_F`、`ROBOT_RAD_TO_DEG_F` | π、°↔rad | `common/math_constants.h` | UR5 Model、IK、IK Selector | **单一数学常量来源** |
| `dh_a_mm`、`dh_d_mm`、`dh_alpha_rad` | UR5 标准DH，mm/rad | `model/ur5_model.c` | FK、解析IK | **机器人本体事实** |
| `base_frame_conversion[(_inverse)]`、`tool_frame_conversion[(_inverse)]` | DH与公开Base/Tool坐标间4×4变换 | 同上 | FK、IK | 机器人本体事实 |
| `joint_min_positions`、`joint_max_positions` | 连续关节行程，raw；J3 ±180°、其余 ±360° | 同上 | `motion_limits`、IK选择、限速器 | **当前URDF仿真模型限位，不等于实物安全限位** |

## 2. 运动学数值稳定与选解策略

| 参数 | 语义/单位 | **唯一数值定义位置** | 引用 / 状态 |
|---|---|---|---|
| `KINEMATICS_TRIG_DOMAIN_EPSILON` | `acos`/余弦有效域浮点容差，无量纲 | `algorithms/kinematics/ur5_analytic_ik.c` | 几何有效域；**未因J6近奇异修复而放宽** |
| `KINEMATICS_SINGULARITY_EPSILON` | 单纯解析算法奇异判别 | 同上 | `q6` 求解；独立于 Selector 的运动安全拒绝门限 |
| `KINEMATICS_GEOMETRY_EPSILON_MM_SQUARED` | 几何退化检查，mm² | 同上 | IK 内部算法 |
| `KINEMATICS_ORIENTATION_EPSILON_SQUARED` | 姿态相关内部数值界 | 同上 | IK 内部算法 |
| `KINEMATICS_SEEDED_WRIST_SIN_THRESHOLD` | `abs(sin J5)<0.001`（近奇异参考入口），无量纲 | `algorithms/kinematics/kinematics_private.h` | 只用于显式参考IK的数值条件化 |
| `KINEMATICS_SEEDED_SHOULDER_MAX_ERROR_RAD` | J1参考分支匹配最大0.005 rad | 同上 | 不能把不同肩部分支强制配对 |
| `KINEMATICS_SEEDED_Q6_MAX_ERROR_RAD` | 原始J6与参考允许偏差0.05 rad | 同上 | 仅局部参考连续性 |
| `KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM` | 条件化候选的FK回代最大0.5 mm | `algorithms/kinematics/kinematics.h` | Stage4默认航点位置容差由此**派生** |
| `KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL` | FK回代旋转矩阵最大单元素差0.003 | 同上 | Stage4默认航点姿态容差由此**派生**，不是角度 |
| `IK_SELECTOR_HARD_SINGULAR_SIN` / `SOFT_SINGULAR_SIN` | 选解器硬奇异0.01 / 软惩罚0.25，无量纲 | `algorithms/motion/ik_solution_selector.c` | Motion策略，不是解析IK解存在性判据 |
| `IK_SELECTOR_SOFT_LIMIT_FRACTION` | 关节限位软边界比例0.10 | 同上 | IK分支选解策略 |
| `IK_SELECTOR_LIMIT_WEIGHT` / `SINGULAR_WEIGHT` | 选解评分权重0.35 / 0.50 | 同上 | IK分支选解策略 |
| `IK_SELECTOR_SCORE_TIE_EPS` | 候选评分数值相等容差1e-5 | 同上 | IK分支选解策略 |
| `kinematics_pose_within_tolerance()` | **唯一**FK姿态残差计算（位置L2 mm、旋转矩阵最大单元素差） | `kinematics_common.c`，声明于 `kinematics.h` | 近奇异IK、笛卡尔起点/航点，门限由各调用方提供 |

重要：`KINEMATICS_SINGULARITY_EPSILON` 和 `IK_SELECTOR_HARD_SINGULAR_SIN` **不是同义参数**。前者仅判断解析算法可否独立求解，后者代表当前应用拒绝的奇异风险，不能为了“八解齐全”合并或放宽。

## 3. 应用控制默认值（单点配置）

下列运行策略值定义在`firmware/freertos_demo/config/robot_defaults.json`的`stage4_motion`中，由构建生成C宏，经`stage4_motion_default_config()`、`motion_stage4_config_t`注入。不要直接编辑生成头，也不要复制到算法、任务或Python中。

| 配置项 | 当前仿真初始值 / 单位 | 使用与归属 |
|---|---|---|
| `pid[0..5].kp/ki/kd` | **0.2/0/0**，每轴可独立配置 | 六轴增量式PID默认配置，基于小幅PyBullet对照；不是实体硬件标定 |
| `pid[0..5].integral_limit_deg_s` | 10，°·s | `pid.c`积分限幅 |
| `pid[0..5].output_limit_deg` | 3° | PID角度修正输出限幅 |
| `pid[0..5].deadband_deg` | 0.02° | PID死区 |
| `max_velocity_deg_s[0..5]` | 15°/s | 梯形规划、多项式整段预检和输出限速器 |
| `max_acceleration_deg_s2[0..5]` | 30°/s² | 梯形规划、多项式整段预检和输出加减速限幅器 |
| `feedback_timeout_ms` | 3000ms | Trajectory、Control反馈新鲜度门禁 |
| `reference_timeout_trajectory_periods` | 3个规划周期；当前60ms | 与`TASK_PERIOD_TRAJECTORY_MS`相乘派生Control参考超时 |
| `cartesian_start_tolerance_mm` | 0.5mm | 请求起点vs实际反馈FK对比 |
| `cartesian_start_rotation_matrix_tolerance` | 0.01，最大矩阵元素差 | 请求起点姿态容差 |
| `STAGE4_CARTESIAN_WAYPOINT_TOLERANCE_MM` | 从 `KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM` 派生 | 每航点FK位置回代 |
| `STAGE4_CARTESIAN_WAYPOINT_ROTATION_TOLERANCE` | 从 `KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL` 派生 | 每航点FK姿态回代 |

控制指令中未量化的 `joint_command_limiter_t.command_raw`（0.01° raw）与 `velocity_deg_s`（°/s）是 **Control Task 私有运行态**，不能移进上述配置。加速度限幅在 `joint_command_limiter.c` 内按当前状态计算，不设第二份上限。

## 4. 启动配置、协议和运行态

| 参数 / 状态 | 唯一来源 | 主要消费者和约束 |
|---|---|---|
| `task_priority.*` | `config/robot_defaults.json` → `tools/generate_firmware_defaults.py` → `task_config.h` | 5个 FreeRTOS 任务优先级；0..4 |
| `task_period_ms.*` | 同一 JSON → generated header | Control 10ms、Trajectory 20ms、Monitor 100ms；单位毫秒 |
| `task_stack_depth_words.*` | 同一 JSON → generated header | Control 768、Trajectory 1024、Monitor 384（words），未经满负载上界证明 |
| `timer_frequency_hz`、`parser_benchmark_samples` | 同一 JSON | Timer/启动性能诊断 |
| `initial_joint_targets_centideg` | 同一 JSON | 开机初始角度，不自动触发 Stage4 运动 |
| `runtime_protocol_tx_period_ms` | `application/runtime_config.c` 的单一可变实例；初值来自生成头 | ProtocolRX SET_PARAMETER写；ProtocolTX读，运行态覆盖默认 |
| `CMD_*`、`PROTOCOL_HEADER_*`、`PROTOCOL_JOINT_PAYLOAD_LEN` | `communications/protocol.h` | 固件收发格式/命令种类；**Python UART旧示例有镜像常量，须加跨语言一致性测试** |
| `motor_state_observation_t`、`last_accepted_tick` | `application/motor_state.[ch]` | 持锁复制反馈与接收时间戳 |
| `g_motion_runtime`、`abort_requested`、队列 | `application/services/motion_pipeline.c` 运行态 | 任务启动、请求/参考队列各1项、故障取消 |
| `pid_controller_t`、`joint_command_limiter_t` 实例 | Control Task 栈上的私有状态 | 积分/导数、命令累积和上次速度 |
| `trajectory_trapezoid_plan_t`、`cartesian_index` | Trajectory Task 私有状态 | 每次请求计算和迭代，不应变成配置 |
| `ROBOT_ENABLE_STAGE4_TASKS` | `firmware/freertos_demo/CMakeLists.txt` | 默认 OFF，显式允许创建任务 |
| `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` | 同上 | 默认 OFF；允许电机目标写入需另外启用，不代表实体安全 |
| `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST` | 同上 | 默认 OFF，强制 TASKS=ON/OUTPUT=OFF，绝不允许量产固件误触发 |

## 5. 测试夹具和剩余重复风险

| 对象 | 归属 | 不能替代的东西 |
|---|---|---|
| 近奇异 `IK_REFERENCE_TEST_MATCH_WINDOW_RAW` | `tests/kinematics/ik_reference_test_limits.h` | 候选匹配窗口，不是关节控制容差 |
| QEMU 自检起始姿态与小幅J2目标 | `tests/qemu_target/stage4_rtos_selftest.c` | 非驱动固定反馈测试夹具，不是运动任务默认目标 |
| PyBullet `TRIAL_GOALS`、`START_RAW` | `simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py` | 动力学小幅试验样本，不是UR5模型事实 |
| PyBullet `max-final-error-deg` / `max-overshoot-j2-deg` / `max-tracking-rmse-j2-deg` | 同一测试脚本命令行 | 仿真验收指标，不是实物控制承诺 |
| GDB `checks/failed` | `tests/qemu_target/stage4_rtos_selftest.c` | 软件仿真断言计数，不是机械/系统安全认证 |

**尚存的跨语言镜像问题**：旧 `simulation/pybullet_ur5/src/qemu_uart_receiver.py` 单独写了帧头、命令ID、关节数量和角度缩放。它们必须与固件 `protocol.h` / `robot_types.h` 一致；未来协议扩展应通过单点schema/生成逻辑消除人工双维护。本轮全局审查将其作为待处理接口风险，而不是在没有端到端测试的情况下直接替换协议解析器。

**安全边界**：真实UART↔PyBullet反馈闭环、丢包/延迟/错误帧故障注入、Control周期P99/WCET、长时运行、实物急停/碰撞约束仍需独立验收；此前6组Host/ARM-QEMU软件自检/PyBullet小幅实验全绿不等于整机可以上电运行。

## 2026-10-27 第四阶段 QEMU 时间计量参数补录

新增的 `application/tasks/stage4_timing_probe.[ch]` **仅用于非驱动QEMU自检**，不属于正式运动控制参数或硬件性能承诺：

| 对象 | 单位/语义 | 唯一来源 | 使用范围 |
|---|---|---|---|
| Timer0原始频率 `timer_driver_get_frequency_hz()` | counter ticks/s（本次25 MHz） | `drivers/timer_driver.[ch]` 返回 `BOARD_SYSCLK_HZ` | QEMU采样和报告换算，**不能再硬编码25MHz** |
| 周期名义值 | ms，Control 10、Trajectory 20、Monitor 100 | `config/robot_defaults.json` → `task_config.h` | 时间采样分别按现有任务周期计算偏差 |
| `STAGE4_TIMING_CAPACITY=128` | 每任务采样容量（次） | `application/tasks/stage4_timing_probe.h` | **测试夹具固定内存上限**，不是固件控制策略 |
| `g_stage4_timing_summary[3]` | Timer0 ticks下的P99绝对周期偏差、最大值、P99/最大处理经过时间 | `stage4_timing_probe.c`；测试结束由QEMU专用任务冻结 | GDB诊断；正常固件不编译或链接 |
| `uxTaskGetStackHighWaterMark()` | words，任务运行以来的最小剩余栈空间 | FreeRTOS内核/Monitor Task | 与配置JSON的栈分配words作对照 |
| 报告P99算法 | 最近阶 `ceil(0.99*N)` | `stage4_timing_probe.c` 单一排序实现 | `tools/analyze_stage4_timing.py` 只做校验、单位换算与归档；**不重算另一份P99** |

该诊断默认不可用：只在 `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=ON`、同时保持 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF` 的隔离ARM测试镜像中编译。参数和诊断不得迁移到生产Model、PID或Stage5 APF逻辑。


## 收尾补测新增的实验参数（仅 Host/PyBullet）

| 参数与单位 | 单一来源 | 引用位置 / 配置类别 |
|---|---|---|
| 实验 PI 的 Ki（C PID 算法所定义单位） | tools/stage4_pid_study.py 的 TRIAL_KI | 每组试验传入 C 桥接库实例，非生产配置 |
| 实验 PID 的 Kd（C PID 算法所定义单位） | tools/stage4_pid_study.py 的 TRIAL_KD | 仅实验组合，非生产配置 |
| 各场景运动目标，raw=0.01° | simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py 的 TRIAL_GOALS | Stage4 物理仿真夹具 |
| 力矩夹具，N·m | tools/stage4_pid_study.py 的 ACTUATOR_FORCES | PyBullet POSITION_CONTROL 最大执行器力矩，仅试验策略 |
| 小幅仿真的默认最终误差门限与物理步长 | stage4_dynamic_closed_loop.py 的 DEFAULT_FINAL_ERROR_DEG、DEFAULT_PHYSICS_DT_S | 测试判据与数值积分设置 |
| 测试增益传输格式 [6][3] double | tests/simulation/stage4_sim_bridge.c 的 stage4_sim_create_with_pid_gains() | 桥接 ABI，六关节行×Kp/Ki/Kd 列；不入正式固件 |

注意：默认 Kp/Ki/Kd 仍仅来自 config/robot_defaults.json；测试矩阵在运行时从此文件读取，**不维护第二份默认值**。实验 C ABI 只改变本次创建的 host 模拟控制器，不能写入应用配置或真实驱动。
