# Stage5: C APF 航点到六轴 IK / 梯形段规划的桥接

## 本次新增的实际功能

`stage5_apf_points_to_joint_route()` 是无任务、无 UART、无电机输出的纯 C 算法接口。输入是 `robot_apf_plan_path()` 已生成的末端空间航点（公共 `base_link` 坐标系，单位 mm）。它使用既有的 UR5 FK 获取初始工具方向，逐点生成同方向末端目标，再调用：

1. `kinematics_inverse_with_reference()`：现有解析 IK；
2. `ik_solution_selector_select()`：保留上一关节连续位置、跳变/限位/奇异点策略；
3. `kinematics_forward()` + `kinematics_pose_within_tolerance()`：选择后回代误差检查；
4. `trajectory_trapezoid_init()`：每段依既有速度/加速度约束计算完整停到停运动时间，不把 20mm 的 APF 步长错当 20ms 可执行指令。

任意点 IK 不可达、靠近被拒绝奇异点、限位越界、FK 残差超过项目已有唯一限值、无效速率配置都拒绝整条路径；先完整预检，再写入输出数组，不会将前半截成功航点误认为可执行动作。

当前代码编译进 ARM，但**没有向 FreeRTOS 任务或电机输出端接线**，也不自动使用真实或仿真机器人运动命令。整个 C 接口仍然只完成关节路线与**合法时间**的预规划，不提供整臂扫掠无碰撞证明。

## 测试

`firmware/freertos_demo/tests/avoidance/stage5_apf_joint_path_test_main.c` 使用真实生产 C APF 路线、UR5 IK、解选优、FK 与梯形规划，测试小幅可达航点，逐点检查 FK、关节限位、每段时长，以及任意后续不可达航点导致的全路线拒绝、输出缓存零污染、空路径、预算和容差非法输入。GitHub Actions `Stage 5 integration and APF safeguards` 运行这些 Host C 测试； ARM 常规编译则验证该模块可与固件一起链接。

## 仍需要补齐的全链路

`robot_apf_plan_path()` → **新增 C IK/轨迹段预规划** → **按关节路径运行 PyBullet 全连杆碰撞门禁** → **FreeRTOS 分段轨迹执行** → QEMU UART → PyBullet 物理反馈。

重点是不能把“末端点路线上障碍物外”当作“关节空间实际运动整臂不会碰撞”。最终只有整臂路线通过检测、且链路全过程有可查的故障/时序证据时才允许仿真执行；无法验收的路径必须拒绝运行。实际硬件命令输出仍默认关闭。
