# Stage5: live QEMU/UART 六轴轨迹路线安全执行（实验）

## 当前阶段的严格边界

此候选分支将 `stage5_avoidance_preflight.py` 已通过生产 C APF -> C UR5 IK / FK -> PyBullet 六轴连杆几何筛查的关节航点，交给现有 `MotionSequencer`。它经现有 UART JOINT_MOVE 接口逐段请求 FreeRTOS 自有 C 梯形轨迹、Control/PID、反馈闭环，不是 Python 发出逐帧理想角度，也不是用 Python IK 替代固件 IK。

**同一 PyBullet 世界**：在创建 UR5 后且第一条控制请求下发前，创建静态长方体，使用这个物理世界的真 `getAABB` 规划；沿途检查 `getClosestPoints` 和实际物理状态，带 5/10 mm（按 APF 场景配置）净距阈值，任何失败抛出异常并停止本次 QEMU 虚拟执行。一次安全检查通过不代表连续时间绝对无碰撞。

## 控制组与绕障组

- `--stage5-route-case clear`：终点相对初始末端位置 (+3,+1.5,0) mm；障碍放在物理场景以外。验证最短运行链路：C APF 路线、C IK 解、实际 PyBullet 物理运动、每 10 ms 全连杆监测、UART 诊断均能一致通过。它**不是“真正绕过障碍”**。
- `--stage5-route-case blocked`：静态长方体放在末端目标直线附近，终点相对起点 (-150,+60,0) mm。先要求 C APF、C IK 及整臂路线预检全部通过才能下发关节请求。此组**未验证成功之前不得宣称动态避障完成**。任何一个门禁拒绝均视为测试场景不满足成功验收、但执行安全机制应拒绝下发。

两组均不修改固件模型、原始 PID、任务周期或机器人尺寸的唯一权威数据。六轴速度和加速度约束从 `firmware/freertos_demo/config/robot_defaults.json` 提取。

## CI 自动化

`.github/workflows/stage5-avoidance-uart.yml` 使用真实 ARM 工具链构建 FreeRTOS ELF、原 C APF/IK 动态库，启动 QEMU MPS2-AN386 连接 PyBullet DIRECT，运行控制组，并验证输出：`result.json` 必须通过、有效物理运动、全部分段完成、无固件故障、全过程碰撞监测计数为正，且记录完整 `joint_trace.csv`，保存 `avoidance_preflight.json`。

清场控制组成功与绕障成功是两项**互不替代**的验收。GUI 鼠标拖拽、30 分钟压力、动态障碍、自碰撞、关节间连续时间安全证明仍未包含在本用例内；本分支禁用对真实电机硬件的输出，只向虚拟 UART 传输命令。
