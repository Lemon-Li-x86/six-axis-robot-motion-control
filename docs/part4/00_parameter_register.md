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
