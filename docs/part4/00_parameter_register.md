# 第4阶段参数登记簿（2026-10-26）

> 设计规则：机器人本体事实属于 Model；算法参数由调用方注入；运行时状态属于 Application/Motor State；轨迹只计算，不自行配置底层驱动。

| 参数或数据 | 语义 / 单位 | 唯一声明 / 来源 | 第4阶段引用 | 分类 / 处理 |
|---|---|---|---|---|
| `ROBOT_JOINT_COUNT` | 六轴数量 | `common/robot_types.h` | 全部轨迹数组与循环 | 固有公共常量，**不重定义** |
| `ROBOT_JOINT_ANGLE_UNIT_DEG` | 1 raw = 0.01° | `common/robot_types.h` | 插值/梯形算法输入输出转换 | 单位换算唯一来源，**不硬编码100或0.01** |
| `robot_joint_positions_t` | 连续位置，0.01°，可跨±180° | `common/robot_types.h` | 插值端点、当前反馈、选中的 IK 目标 | 公共类型，不归一化 |
| `robot_joint_velocities_t` | 速度，°/s | `common/robot_types.h` | 多项式端点速度、梯形最大速度、采样返回 | 公共类型，数值从调用方注入 |
| `trajectory_joint_accelerations_t` | 加速度，°/s² | `algorithms/trajectory/trajectory.h` | 五次多项式端点、梯形最大加速度、采样返回 | 新接口类型，**仅此一份定义**；公共 Robot Types 暂无对应结构 |
| `duration_s` | 多项式总时长，s | 初始化函数实参 | `polynomial_trajectory.c` | 每条轨迹的请求参数 |
| `max_velocity_deg_s` | 各关节最大速度，°/s | 调用方入参 | `trapezoidal_profile.c` 和适配器 | 可配置策略；暂不假定硬件数值 |
| `max_acceleration_deg_s2` | 各关节最大加速度，°/s² | 调用方入参 | 同上 | 可配置策略；暂不假定硬件数值 |
| `joint_min_positions`、`joint_max_positions` | UR5 仿真关节行程，0.01° | `model/ur5_model.c`，经 `motion_limits_check_positions` | `application/services/trajectory_request_adapter.c` | Robot Model 静态事实，**禁止复制进轨迹模块** |
| `coefficients_deg` | 归一化时间 u=t/T 的多项式系数，° | `trajectory_polynomial_plan_t` | `polynomial_trajectory.c` | 计算派生状态，不是全局配置 |
| `acceleration_fraction_s2`、`velocity_fraction_s` | 梯形公共路径进度导数，s⁻²、s⁻¹ | `trajectory_trapezoid_plan_t` | `trapezoidal_profile.c` | 每次规划派生结果 |
| `sample elapsed_s` | 轨迹采样时刻，s | Task/调用方注入 | 两个 sample() | 运行时输入；**不使用 FreeRTOS tick 作算法内部时间** |
| 轨迹采样周期 | 将来与 Control Task 同步的周期 | 未来应用配置 | 当前模块不保存 | 尚未接入，不得猜测硬编码 |
| PID Kp、Ki、Kd 等 | 控制策略配置 | 未来 Control 配置 | 当前不使用 | 第4阶段后续任务，不放进轨迹模块 |

注意：`trajectory_request_from_ik()` 只桥接数据并检查位置端点，最大速度与最大加速度需要未来配置来源；不应从 Python、Motor Driver、IK 或多个源文件各复制一套。笛卡尔轨迹、碰撞约束和 PID 不在本次实现范围。

## 后续补充：完整第4阶段（2026-10-26）

| 参数/状态 | 语义和单位 | 唯一来源 | 使用位置 |
|---|---|---|---|
| `task_priority.{control,trajectory,monitor}` | RTOS 优先级 | `config/robot_defaults.json` → generated `task_config.h` | 新任务创建 |
| `task_period_ms.{control,trajectory,monitor}` | 调度周期，ms | 同上 | 周期任务/笛卡尔离散约束 |
| `task_stack_depth_words` | StackType_t words | 同上 | 新任务栈分配 |
| Stage4 Kp/Ki/Kd，积分限幅/输出限幅/死区 | PID 反馈策略，见 `pid_config_t` | `config/stage4_motion_config.c` 的单一定义宏和6关节数组 | Control Task 通过 `motion_stage4_config_t` 注入 |
| 默认关节最高速度/加速度 | °/s、°/s²，仿真占位值 | 同一 `stage4_motion_config.c` | 轨迹算法接受配置，不内置数值 |
| 反馈最长年龄 | ms | 同一 `stage4_motion_config.c` | Control/Trajectory Task |
| `CARTESIAN_PATH_LINE/ARC` | 路径种类 | `cartesian_trajectory.h` | Cartesian 算法与队列 |
| SO3 矩阵合法性容差 | 无量纲算法输入验证门限 | `cartesian_common.c` | 私有数值保护，不是 Robot Model 参数 |
| `ROBOT_ENABLE_STAGE4_TASKS` | 是否启动任务 | `firmware/freertos_demo/CMakeLists.txt` | main/构建 |
| `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` | 是否向 Motor State 写入 PID 目标 | 同上 | Control Task 编译门控 |
| Continuous→Canonical wrap | raw 0.01° | `common/angle_utils.c` | Motor Driver、轨迹任务、Control Task |
| 轨迹请求/参考队列容量 | 单条最新请求/单条最新参考 | `application/services/motion_pipeline.c` | Task 服务，非可配运动学常量 |
| `g_motion_runtime`、Trajectory/PID instance | 运行时状态、监测统计 | Application Task/Services | **不得存入 Robot Model 或算法常量文件** |

注意：此前 Stage2 Driver 内的同名角度回绕数学逻辑已改为调用 Common 唯一实现；修订了其 QEMU / Host / Unity 构建依赖，避免重定义。

### 笛卡尔安全门限与中止策略（补充）

- `cartesian_start_tolerance_mm`、`cartesian_start_rotation_matrix_tolerance`：起点反馈姿态与请求起点一致性判据，统一由 `config/stage4_motion_config.c` 提供。
- `cartesian_waypoint_position_tolerance_mm`、`cartesian_waypoint_rotation_matrix_tolerance`：每一个 IK+Selector 选中解的生产 FK 回代误差判据，统一由同一配置文件提供；**矩阵元素差不是角度误差**。
- `reference_timeout_ms`：已生成 `TASK_PERIOD_TRAJECTORY_MS` 的倍数，声明于 Stage4 配置，Control Task 不自行复制常数。
- `abort_requested`：Control Task 发现故障、反馈过期时请求 Trajectory Task 停止，属于 Application 运行态而非算法参数。

## 跨阶段修复：J6近奇异参考条件化（2026-10-26）

| 参数/接口 | 语义、单位 | 唯一声明位置 | 分类与约束 |
|---|---|---|---|
| `KINEMATICS_SEEDED_WRIST_SIN_THRESHOLD` | `abs(sin(J5)) < 0.001` 的近奇异数值敏感门限，无量纲 | `algorithms/kinematics/kinematics_private.h` | 算法可靠性常量，仅用于**带参考**的 IK，不改变数学可达域 |
| `KINEMATICS_SEEDED_SHOULDER_MAX_ERROR_RAD` | 当前参考J1与候选J1可匹配的最大角差，rad | 同上 | 算法可靠性常量，拒绝不同肩部分支套用参考 |
| `KINEMATICS_SEEDED_Q6_MAX_ERROR_RAD` | 参考J6与姿态反算J6允许的最大一致性偏差，rad | 同上 | 算法可靠性常量，拒绝远离参考的替代 |
| `KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM` | 参考条件化后的IK候选必须通过的FK位置回代门限，mm | `algorithms/kinematics/kinematics.h` | 公开算法验收常量；Stage4默认运行配置由此派生 |
| `KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL` | 同一候选的FK旋转矩阵最大单元素误差，无量纲 | 同上 | 公开算法验收常量；不能解释为角度 |
| `KINEMATICS_TRIG_DOMAIN_EPSILON` | q3/q5 的反三角函数计算域容差 | **仍只在** `algorithms/kinematics/ur5_analytic_ik.c` 原位置定义 | **未放宽**，不被近奇异修复替代 |
| `kinematics_inverse_with_reference()` | 显式引入当前Canonical关节参考，0.01°单位 | `algorithms/kinematics/kinematics.h` | 纯函数输入；不引入运行时全局状态 |
| `current` → `reference` | 当前连续关节位置转换成Canonical参考 | `common/angle_utils.c` → `algorithms/trajectory/cartesian_waypoint.c` | 当前状态来自调用方，不在IK内部缓存 |

设计保证：旧 `kinematics_inverse` 不受影响；新的IK接口只在近腕部奇异、与当前肩/腕符号匹配、J6与参考相近的条件下替代J6，随后按实际请求位姿执行生产FK回代门限。第4阶段 Cartesian 之后仍使用既有 Selector 的近奇异拒绝、关节行程检查和任务端速度跳变保护；**不绕过硬奇异保护，不代表零J5奇异点的连续运动已解决**。

`config/stage4_motion_config.c` 的初始Waypoint FK门限现在从 Kinematics 两项常量派生，运行态允许用户注入更严格的门限；无相同语义硬编码副本。以上算法门限不是生产机器人经实测确认的安全参数。

## 一次性验收批次：Joint Command Limiter（2026-10-26）

| 参数/状态 | 语义 / 单位 | 唯一来源 | 消费模块 |
|---|---|---|---|
| `max_velocity_deg_s` | 各关节命令速度上限，°/s | 调用方 `motion_stage4_config_t` / 默认 `stage4_motion_config.c` | 既有轨迹、`joint_command_limiter.c` |
| `max_acceleration_deg_s2` | 各关节命令加速度上限，°/s² | 同一份运行配置/默认配置 | 既有轨迹、`joint_command_limiter.c` |
| `joint_command_limiter_t.command_raw` | 尚未取整的六轴命令累积值，0.01° raw | Control Task 私有实例 | `joint_command_limiter.c` |
| `joint_command_limiter_t.velocity_deg_s` | 逐轴上一次命令速度，°/s | 同一个实例 | `joint_command_limiter.c` |
| `task_stack_depth_words.control` | Control Task StackType_t 字数 | `config/robot_defaults.json` 的单点配置，目前768 | Config Generator → FreeRTOS xTaskCreate |

控制命令算法输入校正量来自既有 Incremental PID。命令限速器为纯算法，依次约束**加速度变化率、绝对速度、模型关节位置限位**，最终仅在驱动边界量化为0.01°；内部保留小数位，否则低加速度与10ms更新周期会出现量化锁死。该算法不发送命令、不增加 UR5 常量和静态全局运行状态。

Control Task 启动栈由512加到768 words以预留嵌套计算余量；该数值**不是最终满负载验证值**。运行初始/错误恢复后会重置速率累积状态，仍需用反馈真实执行、栈高水位与周期数据实测验证。

Stage4默认 `cartesian_waypoint_*` 容差由 `kinematics.h` 定义的安全门限派生；即使应用层手动注入更宽松配置，`motion_pipeline_start()` 现在也会拒绝大于 IK 验收门限的运行配置，避免算法放行而应用忽略。

以上改动全部保持 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF` 为默认；任何机械执行或 PyBullet 实际动作均需另外明确开启且检查仿真环境。
