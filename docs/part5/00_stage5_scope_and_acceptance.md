# 第五阶段启动与验收范围（2026-10-27）

项目：六轴工业机器人嵌入式运动控制固件开发与仿真验证  
阶段名称：**系统集成与功能扩展**

## 0. 分支冻结点

- 第四阶段验收快照：`backup/stage4-verified-2026-10-27`
- 第五阶段开发：`feat/stage5-system-integration-2026-10-27`
- 两者创建时均精确指向 `1ee9ce02d6954e8e9ad4e095611dfd78f0795ab9`。
- `main`、旧Stage3备份、旧Stage4工作分支均保持不变。**备份是Git分支引用，不是GitHub权限级不可变锁；后续任何提交都不得写入备份分支。**

## 1. 原任务书要求（以项目大纲第5阶段为准）

| 项 | 任务书原始要求的核心内容 | 阶段产出 |
|---|---|---|
| 5.1 全模块系统集成 | 外设驱动、通信解析、运动学、轨迹、PID连成“指令接收→轨迹规划→运动学解算→PID控制→状态反馈”，按Git提交追踪 | 集成后的固件工程 |
| 5.2 静态场景避障 | PyBullet长方体静态障碍物；**C语言人工势场法**；嵌入路径规划；末端避障航点调整 | 集成后的固件/仿真 |
| 5.3 性能优化 | FreeRTOS任务CPU占用/任务运行统计；`.map`文件内存分析；中断优先级/周期抖动优化 | 实时性/稳定性分析 |
| 5.4 全链路联调与排障 | QEMU/GDB + UART日志定位时序、溢出、丢包、死机，记录症状/根因/修复 | 问题追踪记录 |

原大纲并未要求第五阶段直接对实体机器人执行运动，也未规定避障可以只检查末端点。因此不能将“末端点APF样例无碰撞”冒充六个连杆的工业机器人碰撞安全验证。

## 2. 开发顺序与每项门禁

1. **冻结与基线**：从第四阶段测试通过的精确SHA创建两条独立分支，并在新的CI上重新执行 Stage3/4 测试，完成后锁定生产代码变更范围。
2. **纯C人工势场（首个已启动的工作包）**：在 `algorithms/avoidance/` 中定义毫米坐标、AABB与参数注入接口；对 NaN、内部点、移动线段撞箱、局部极小、非法参数默认拒绝；单独 Host CTest 通过。**不直接改Control/Trajectory Task**。
3. **PyBullet长方体场景**：保持障碍几何测试夹具唯一来源，将真实C APF输出用于末端航点实验；记录距离、通行/失败次数、无法脱困情况，并增加整个机器人连杆碰撞检查。不得用Python重写一份APF。
4. **UART集成**：定义明确的方向、版本、seq、ACK/NAK、限幅/超时/拒绝语义，将安全的运动请求真正接到 `motion_pipeline_submit_*`。旧 `CMD_SET_JOINT_TARGETS` 是固件输出指令，不能擅自改为接收执行运动；先串口仿真无驱动测试。
5. **RTOS时间与内存**：采集任务运行统计/栈水位、周期实际值、最大值/P99，分析 ARM `.map`，记录改动前后对比（必须有实测数据）。
6. **异常与稳定性**：通信错误帧/分包/丢包/延时/反馈陈旧/任务取消及长时间QEMU/PyBullet联调；按问题单记录根因和修复SHA。

## 3. 当前安全边界

- Stage4 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF` 默认不得变动。
- 实体机械臂**不得**使用仅末端点的障碍物势场结果当作已验证安全路径。链接碰撞、奇异、关节速度/加速度、轨迹时序和失联急停需独立检查。
- 人工势场可能陷入局部极小值，停滞必须明确返回 `ROBOT_STATUS_ERROR_NO_SOLUTION`，不能伪造完整可达路径。
- Stage5算法参数必须由调用方注入，`Model`保留UR5几何唯一来源，`Test Fixture`保留具体障碍场景；禁止重复复制关键配置。
- 第五阶段开始 **不等于第五阶段完成**。完整验收需四条任务线都有实际测试证据和问题追踪记录。

## 4. 首批提交范围

- `algorithms/avoidance/potential_field.[ch]`：实验性纯C APF、静态AABB几何保护；仅检查**末端点**。
- `tests/avoidance/`：独立Host CTest、输入原子性和碰撞拒绝。
- `.github/workflows/stage5-system-integration.yml`：Stage5 CTest + 既有 Stage3/4 Host回归。

这些源码在第五阶段新分支上开发、单独测试，**尚未**接入完整轨迹、UART、FreeRTOS或PyBullet闭环；不允许将测试构建成功误记为集成完成。

## 5. 第五阶段首轮 CI 实际证据（2026-10-27）

[GitHub Actions Run #40427456854](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40427456854) 对应 Stage5 **首批已提交的C避障内核与PyBullet静态长方体探针**：

- `native-stage5-and-stage4-regression`：success。Stage5 APF严格C11编译与输入安全测试通过，并重新执行既有Stage3/4全部6个原生Host回归。
- `pybullet-static-cuboid-c-apf`：success。PyBullet DIRECT加载UR5和真实静态长方体碰撞体，调用 `getAABB()` 将几何交给与固件同源的C APF共享库，计算下一航点，并记录JSON诊断。
- 云端探针实测：`actual_step_mm=20.000000000000046`（数值舍入误差内符合20mm上限），最近表面距离`105.0mm`，`passed=true`。**只进行一段末端点无驱动航点计算，没有移动整台UR5。**

此证据仅证明“静态场景输入→真实C APF单步→边界与单元回归”首个垂直切片成立。后续还需要完整路径、真实六连杆碰撞检测、IK选解、时间参数化、UART/RTOS数据链路与故障处理后，才能称第5阶段避障功能完成。
