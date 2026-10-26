# 第4阶段·第一模块：独立关节空间轨迹规划（2026-10-26）

## 与项目手册的对应关系

对应第4阶段核心任务1：六关节三次/五次多项式插值与梯形速度规划。**仅完成独立源码和测试入口；第4阶段任务2（笛卡尔直线/圆弧）、任务3（增量式 PID）、任务4（FreeRTOS Control/Trajectory/Monitor Task）尚未实现。**

本分支是在第3阶段待本地验收修复提交 `4ea72433` 上创建。另有只读快照 `backup/stage3-review-snapshot-2026-10-26`；修复前备份 `backup/pre-teacher-review-2026-10-26` 也未改变。第3阶段的 near-singular 根因排查和 QEMU FK 回代**仍待用户本地验证**。

## 目录与职责

```text
firmware/freertos_demo/
├── algorithms/trajectory/
│   ├── trajectory.h              # 唯一公开规划 API、状态和单位
│   ├── polynomial_trajectory.c   # 三次/五次位置、速度、加速度
│   └── trapezoidal_profile.c     # 六轴同步梯形/三角速度规划
├── application/services/
│   ├── trajectory_request_adapter.h
│   └── trajectory_request_adapter.c  # 当前连续位置 + IK 选解结果 → 轨迹
└── tests/trajectory/
    ├── CMakeLists.txt            # 独立原生 CTest
    ├── trajectory_test_main.c
    └── trajectory_adapter_test_main.c
```

**没有修改原有** `main.c`、`firmware/freertos_demo/CMakeLists.txt`、FreeRTOS 任务、Motor Driver、Robot Model 或第3阶段算法。新增算法当前不属于正式 `freertos_demo.elf`，也不向电机发送控制指令。将来经过分阶段验收后由 Application 层显式加入固件构建。

## 公开接口与数据语义

公共类型沿用 `common/robot_types.h` 和 `common/error_code.h`，`trajectory.h` 只补充尚不存在的加速度、轨迹计划和采样输出类型。

- 输入、输出关节位置：连续 `int32`，单位为 **0.01°**，允许跨越 ±180°，不通过 canonical angle 回绕；转换比例使用 `ROBOT_JOINT_ANGLE_UNIT_DEG`。
- 速度、加速度：**°/s、°/s²**；时间：**秒**。
- `trajectory_polynomial_init_cubic`：六关节独立三次时间归一化曲线，匹配两端位置和速度，持续时间由请求提供。
- `trajectory_polynomial_init_quintic`：进一步匹配两端加速度，不保证全过程一定满足速度/加速度最大值或关节限位；规划后需要额外检查。
- `trajectory_trapezoid_init`：六关节以统一归一化进度同步加减速，默认起停速度为0；按照调用方每关节最大速度和最大加速度自动选择梯形或三角曲线；无运动时持续时间为0。
- `trajectory_polynomial_sample`、`trajectory_trapezoid_sample`：任意正时间采样，每次返回连续位置、速度和加速度。超过结束时间取终点；负时间或 NaN/Inf 拒绝。目标输出仅在整组六轴验证成功时写入。
- `trajectory_request_from_ik`：从当前连续位置与 `ik_selection_result_t.target_positions` 生成梯形计划，复用 `motion_limits_check_positions` 读取 Robot Model 的限位。纯数据适配，不负责获取锁或执行器驱动。

本阶段不自行保存“默认最大速度/加速度”，原因是当前 `robot_model_t` 只有几何与位置限位，尚未登记速度与加速度的可靠来源。由上层显式传入，避免在不同模块出现相互矛盾的魔法数字。

## 如何接入现有数据（后续集成示例）

1. Application 经 `motor_state_get_feedback` 复制同步观察数据，在控制时刻检查 `motor_state_feedback_is_fresh`；**不能由轨迹模块无锁调用 Motor Driver**。
2. 由第3阶段 IK Selector 给出 `ik_selection_result_t`，其 `target_positions` 已是连续角。
3. Application 用 `observation.feedback.unwrapped_positions` 和 `selection` 调用 `trajectory_request_from_ik`，传入有来源的速度、加速度配置。
4. 将返回计划交由未来 Trajectory Task，根据控制周期调用 `trajectory_trapezoid_sample`。后续 Control Task 才负责 PID、状态反馈和 Motor Driver 指令。

接口示意：

```c
/* 在 Application 层，获取新鲜且同步的 feedback snapshot 之后： */
trajectory_trapezoid_plan_t plan;
robot_status_t status = trajectory_request_from_ik(
    &observation.feedback.unwrapped_positions,
    &selection,
    &configured_max_velocity_deg_s,
    &configured_max_acceleration_deg_s2,
    &plan
);
if (status == ROBOT_STATUS_OK) {
    /* plan.total_time_s 及 sample() 可交给未来 Trajectory Task */
}
```

当前不调用 `motor_state_set_targets`；正式接入需要独立 Control/Trajectory Task、错误传播、反馈过期保护、碰撞/中间轨迹限位检查和 QEMU 验证。

## Windows / Linux 本机独立测试

从仓库根目录运行，不依赖 ARM 工具链、FreeRTOS、Unity、PyBullet：

```powershell
git fetch origin
git switch feat/stage4-trajectory-2026-10-26
git pull --ff-only

cmake -S firmware/freertos_demo/tests/trajectory -B build/stage4-trajectory
cmake --build build/stage4-trajectory
ctest --test-dir build/stage4-trajectory --output-on-failure
```

这会分别编译：

1. 纯轨迹数学测试：三次/五次两端约束、±180°连续角、梯形和三角速度曲线、六轴同步、限制边界与失败时输出保持不变。
2. 应用适配器测试：复用生产 UR5 Model 与 Motion Limits，检查合法/非法 IK 目标和限制失败保持原计划不变。

另有新增的 `.github/workflows/stage4-trajectory-host.yml`，对该特性分支的相关代码 push 运行 Linux CMake/CTest。**不能将新建测试或 CI 文件视作已有 PASS 结果**。

## 目前不能声称完成的内容

- 没有生成 ARM Cortex-M4 / QEMU 的第4阶段 ELF；`main.c` 与固件 CMake 均未接入新模块。
- 尚无本机编译日志、CTest 输出、速度/加速度跟踪图或单片机耗时基准。
- 三次/五次插值可能出现超调，需要后续边界/动态限位检测和轨迹段组合策略。
- 未做真实执行器安全、碰撞、笛卡尔轨迹、PID 参数整定或任务实时性分析。
- 不能以新模块的存在替代第3阶段老师要求的 IK 回归和 QEMU 端到端验收。
