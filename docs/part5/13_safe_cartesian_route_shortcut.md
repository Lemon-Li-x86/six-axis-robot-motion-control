# Stage5/6: 生产 C APF 路线的安全压缩（仿真侧、显式可选）

## 用途与边界

原始生产 C APF 遇到障碍时会生成细密的笛卡尔航点，每个航点经生产 C IK/梯形规划和 FreeRTOS UART 要求独立关节动作。压缩器 `stage5_cartesian_shortcut.py` 是**应用层离线路线优化**，不是第二套 APF、UR5 运动学、控制器或嵌入式任务。默认 `shortcut_cartesian=False`，没有开启时行为保持不变。

- 输入保留**原始 C APF 路线**、每段任务端点的累计索引、PyBullet 真正 AABB、APF 配置 `clearance_mm`。
- 仅调用固件 C 几何函数 `robot_apf_segment_blocked()` 判断两航点之间的整条线段是否与膨胀后的静态长方体相交。
- **每个任务段的末尾航点必须保留**。这是将来第六阶段抓取点、中间避障点、放置点、返回原点等交由上层任务编排器控制的必要条件；优化器不处理抓取事件或业务状态。
- 对所选航点，必须重新运行**既有生产 C IK、解选优、FK、关节限位/速度/加速度**，再运行**真实 PyBullet 全连杆离散扫掠净距检查**；单凭 C 末端线段无障碍不构成“整臂安全”。
- 固定验证层次 `VERIFIED_SHORTCUT_SPANS=(None,8,4,2,1)`：从最大可见跨度尝试，若 IK 或整臂门禁失败就尝试更密航点；最终尝试原始路线。所有候选失败则任务整体拒绝执行。**不会降低全臂净距、跳过任意任务段或向串口下发部分已验证路线**。
- 返回报告包含 `raw_apf_waypoints`、`cartesian_waypoints`、`shortcut_attempts`、`shortcut_accepted_max_original_span` 以便审查路径变化，候选检查期间不会发送任何 UART 帧。

## 可维护性契约

真正的 UR5 几何/IK 残差来自生产 C，任务运动限制来自 `robot_defaults.json`，场景安全策略来自 `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json`。这份优化器只接收数据与函数，不再维护平行的硬编码机器人限位或实现 Python 版障碍检测。

`Stage 5 integration and APF safeguards` 在真实 PyBullet + 生产 C 动态库上运行 `test_c_geometry_shortcut_keeps_mission_leg_boundary`，验证中间任务点不能被删除、两段路线被整臂复检、原始 PyBullet 状态恢复。现有不启用压缩的默认回归仍应通过。

## 已知限制与阻挡动态实验

压缩只能减少**经过重新验证的任务关节路点数**；它不是任务稳定性修复，也不是时间最优/连续碰撞的证明。静态全臂检查是有限角度步长采样，不证明采样间的数学无碰撞，也未证明真实硬件安全。

长路线曾在实际 QEMU/UART 试验中发生 `NOT_READY (-7)`（[Issue #22](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/22)）；缩短到 4 段仍曾触发同类错误。本独立通用核心不包含失败的阻挡场景变更，也不关闭该 Issue。只有**真正挡路、全部物理分段到位、无 RTOS 故障、运行时全臂无碰撞**的单独验收通过，才能宣称 Stage5 动态避障成功。
