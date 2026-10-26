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
