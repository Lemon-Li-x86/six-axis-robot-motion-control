# 第五阶段 5.2：人工势场多航点与整臂几何验收门禁

本文件只报告本分支新增的功能，不代表第五阶段整体验收。

## C 算法路径

- 输入：现有 `robot_apf_point_t`、`robot_apf_aabb_t`、`robot_apf_config_t`。
- `robot_apf_plan_path()` 使用已经存在的 `robot_apf_step()` 计算受步长限制的连续末端航点。每个线段保持原 C 实现的 AABB 膨胀和穿越拒绝。
- API 调用者显式提供 waypoint 容量。步数耗尽、APF 停滞、数值异常、障碍穿越一律失败；失败后有效输出计数为零。执行两遍纯算法预检，避免把失败前的半条路径当成有效轨迹。
- 规划器不为增益、障碍位置或机器人模型添加默认值，不堆分配，不直接下发电机指令。
- 代码已编入 ARM 固件，但 **没有连接到现有 FreeRTOS Trajectory Task，也不会在启动时自动执行**。接线前必须先完成 IK、关节限位/速度/加速度和整臂碰撞门禁。

## PyBullet 整臂静态障碍物门禁

`stage5_collision_validation.py::validate_joint_path` 接收**已求出的关节路径（弧度）**，读取加载 URDF 的六个可动关节限位，按调用者注入的最大关节采样步长离散插值，然后通过 PyBullet 原生 `getClosestPoints` 检查机器人 **base 和所有 link** 对所有目标静态障碍体的接触或小于给定净距。

- 拒绝关节限位外的路径、非有限角度、无效配置与超出采样预算的路线。
- 只用于独立 PyBullet 预检，会在 `finally` 恢复机器人原始关节位置与速度；不在真实 GUI 运行控制线程内复位关节。
- 不使用 Python IK 冒充固件 C IK，也不复制项目 DH、URDF 角度边界或 PID 常数。
- 样本通过只能证明“按此采样密度，未发现与给定静态障碍体过近的连杆”，**不是连续时间的无碰撞数学证明**。本接口目前也不验证自碰撞和动态障碍。

## 复现

在根目录执行：

~~~bash
cmake -S firmware/freertos_demo/tests/avoidance -B build/stage5-avoidance -G Ninja
cmake --build build/stage5-avoidance
ctest --test-dir build/stage5-avoidance --output-on-failure
python -m pip install pybullet
python -m unittest discover -s simulation/pybullet_ur5/tests -p 'test_stage5_collision_validation.py' -v
~~~

还应运行仓库原有 `stage5_static_obstacle_probe.py` 单步 C 桥接测试。CI 见 PR #10 的 `Stage 5 integration and APF safeguards` workflow。每个测试的结果必须以对应提交 SHA 的最终 Actions Job 为准，不能仅凭测试文件存在声明通过。

## 未完成的关键集成

1. 将 PyBullet 静态场景经过统一接口送入 C APF，生成**真正绕过障碍**的可行完整航点序列（存在局部极小必须明确失败）。
2. 将 APF 每个末端航点接项目现有 C IK、解选优和 20 ms 轨迹任务，做全关节连续限位与速度/加速度检查；不可用 PyBullet IK 代替嵌入式端。
3. 在真实 QEMU/UART ↔ PyBullet 闭环里验证整臂碰撞门禁、速度抖动和故障停止。该步尚未通过验收。
