# 第4阶段：路径规划与运动控制代码集成说明（2026-10-26）

## 验收定位

本分支按原始《六轴工业机器人嵌入式运动控制固件开发与仿真验证》PDF 第4阶段四项核心任务提供**全套待本地验证的源码**：

1. 关节空间：三次/五次插值、六轴同步梯形/三角加减速（`algorithms/trajectory`）；
2. 笛卡尔：末端位置直线/三点圆弧，旋转矩阵转四元数做最短路径球面插值，按步长采样并调用既有 FK/IK/最优解筛选；
3. 单关节增量式 PID：六关节实例、Kp/Ki/Kd、积分限幅、输出限幅、死区、反积分饱和；
4. FreeRTOS：Trajectory / Control / Monitor 周期任务，与已有 ProtocolRX / ProtocolTX 协同，通过队列/现有 `motor_state` 互斥锁交互。

**不是已验收完成**：本次无法在用户 Windows ARM/QEMU/PyBullet 环境里运行固件、采集实时调度数据或真正整定 PID。文档中的初始增益不是调优结论，第三阶段近奇异修复仍须复测。

## 分支与备份

- 原始备份 `backup/pre-teacher-review-2026-10-26` 不变。
- 第3阶段修复后快照 `backup/stage3-review-snapshot-2026-10-26` 仍停在 `4ea72433`。
- 第4阶段全部源码仅在 `feat/stage4-trajectory-2026-10-26`，**没有向 main 合并**。
- 第4阶段在现有第3阶段源码基础上继续开发；本次仅为了消除重复角度归一化而让 Driver 复用 Common `angle_utils`，相关 Host / QEMU / Unity 构建依赖同步调整。

## 模块与接口流

```text
                            Robot Model (UR5 facts/limits)
                              ↑          ↑
Host/ProtocolRX       IK/Selector        motion_limits
        │                    │            │
    [Application 请求接口 motion_pipeline_submit_joint/polynomial/cartesian]
                       │
                Queue: latest request (1)
                       │
              Trajectory Task, period from JSON
              ├── joint: trapezoid sample
              └── Cartesian: sample → IK → solution selector
                       │
                Queue: latest target (1)
                       │
              Control Task, period from JSON
                  motor_state_get_feedback() (mutex + freshness)
                       │
                 six-axis incremental PID
                       │
              clamp joint/velocity limits
                       │
                 motor_state_set_targets()
                  (ONLY if explicit output flag ON)
                       │
             existing ProtocolTX → PyBullet
                       
              Monitor Task → diagnostics / RTOS stack watermarks
```

**算法层无 UART、FreeRTOS、Motor Driver，任务层不复制 UR5 几何/限位。** 模块内无需动态分配；唯一队列和任务在 Application 服务启动时申请。所有队列容量及任务调度策略在 `application/services/motion_pipeline.c` / `task_config.h` 单一管理。

### 公开 API

- `trajectory_polynomial_init_cubic` / `init_quintic` / `sample`；
- `trajectory_trapezoid_init` / `sample`；
- `cartesian_line_init`、`cartesian_arc_init`、`cartesian_trajectory_sample`、`cartesian_trajectory_joint_waypoint`；
- `pid_init` / `pid_step` / `pid_reset`；
- `joint_controller_init` / `joint_controller_step`；
- `trajectory_request_from_ik`；
- `motion_pipeline_start`、`motion_pipeline_submit_joint`、`motion_pipeline_submit_polynomial`、`motion_pipeline_submit_cartesian`、`motion_pipeline_get_diagnostics`；
- `robot_joint_angle_normalize_raw` / `robot_angles_canonicalize` 归属 Common，单位与宏均来自 `robot_types.h`。

角度存储：continuous `int32_t`，每单位 0.01°；速度 °/s、加速度 °/s²、轨迹周期秒；笛卡尔平移 mm、旋转 `T_base_ee` 3×3，和第3阶段一致。

### 实时任务配置与安全默认值

| 配置项 | 值 | 唯一来源 |
|---|---:|---|
| Control 优先级/周期/栈 | 4 / 10ms / 512 words | `config/robot_defaults.json` → generated header → `task_config.h` |
| Trajectory 优先级/周期/栈 | 2 / 20ms / 1024 words | 同上 |
| Monitor 优先级/周期/栈 | 1 / 100ms / 384 words | 同上 |
| ProtocolRX / ProtocolTX | 原有任务 | 原有 `task_config.h` |
| 反馈过期阈值 | 3000ms（实验初值） | `config/stage4_motion_config.c` |
| 六轴最高角速度/加速度 | 15°/s、30°/s²（实验初值） | 同上 |
| PID Kp/Ki/Kd、死区、限幅 | 0.8/0/0，0.02°，3°等**未经整定初值** | 同上 |
| 笛卡尔路径周期 | 必须与 Trajectory Task 20ms 一致 | API 参数与 Task Config 比较 |

真实运动安全尚未验证。关节位置端点经过 Robot Model 限位，笛卡尔离散逆解也通过选解器、选中解的 FK 回代精度门限及速度跳变检查；但这**不等于完整连续空间碰撞检测、全部关节动力学/加速度约束都已验证**。绝对不能直接用在实体机器上。

### 编译选项（不默认改变第1～3阶段启动行为）

```text
ROBOT_ENABLE_STAGE4_TASKS=OFF           # 默认：编译算法；不启动新 RTOS 任务
ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF  # 默认：PID 计算，但不发送电机位置目标
```

任务需要显式启用，且任何 Stage4 输出必须同时启用任务。进入 PyBullet PID 联调前，先验证算法 + RTOS 栈水位。原先固件的初始关节目标发送行为仍存在，不等同于 Stage4 PID 使能。

### 分阶段本地运行

**A. 完整第4阶段 Host 算法测试（建议先做）**

```powershell
git fetch origin
git switch feat/stage4-trajectory-2026-10-26
git pull --ff-only
cmake -S firmware/freertos_demo/tests/trajectory -B build/stage4-trajectory
cmake --build build/stage4-trajectory --config Debug
ctest --test-dir build/stage4-trajectory -C Debug --output-on-failure
```

目标：关节轨迹、IK-to-trajectory 适配、笛卡尔/PID、笛卡尔 FK→IK→选解链，共四个 Host CTest。测试失败必须记录原始输出，不能手工忽略断言。

**B. Stage2 原有驱动回归**

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/motion-host
cmake --build build/motion-host
ctest --test-dir build/motion-host --output-on-failure
```

**C. Cortex-M4/QEMU 构建和第4阶段纯算法测试**

```powershell
cmake --preset arm-qemu
cmake --build --preset arm-qemu
```

运行独立 `qemu_stage4_trajectory_tests.elf`：

```powershell
qemu-system-arm `
 -M mps2-an386 `
 -kernel .\build\cmake-arm\firmware\freertos_demo\qemu_stage4_trajectory_tests.elf `
 -S -gdb tcp::1234 -serial null -monitor none -display none
```

另开 PowerShell：

```powershell
arm-none-eabi-gdb .\build\cmake-arm\firmware\freertos_demo\qemu_stage4_trajectory_tests.elf
```

```gdb
set pagination off
target remote localhost:1234
break qemu_stage4_test_done
continue
p g_stage4_test_finished
p g_stage4_test_report
```

**必须**看到 `finished=1`、`failed=0`，不能只看断点命中。

**D. FreeRTOS 新任务运行验证（初次仅 dry-run）**

```powershell
cmake --preset arm-qemu -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF
cmake --build --preset firmware
```

用正式 `freertos_demo.elf` 启动 QEMU/PyBullet，GDB 检查 `g_motion_runtime.diagnostics` 中 Monitor 更新、`control_stack_free_words`、`trajectory_stack_free_words` 和错误计数（先建立有效请求才有控制步骤）。请注意队列不会自己制造运动指令。需要应用层调用 `motion_pipeline_submit_joint`，或先将测试命令注入应用服务，不能通过未定义的 UART 指令“自动猜测”启动。

**E. PyBullet 闭环联调（需手工授权，非首次测试）**

```powershell
cmake --preset arm-qemu -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON
cmake --build --preset firmware
```

启用输出前必须确认反馈在规定过期窗口内、关节限位合法、仿真桥已运行，先小位移，再逐步记录跟踪误差、超调、稳态偏差和 PID 增益变更。此配置使用现有 `motor_state_set_targets` → ProtocolTX 链路，ProtocolTX 周期会在固件初始化时对齐控制周期。

### 两点尚未解决的工程问题

1. 第3阶段近奇异性与 FK 回代的老师评语需在本地独立确认，不能用 Stage4 算法结果替代。
2. 笛卡尔每个离散插值位姿由现有 IK 和选解器决定。可能遇到奇异、不可达或关节速度跳变而拒绝，属于预期安全停止；**还没有完整连续轨迹的碰撞/加速度证明**。

## 交付材料与实测

详细参数登记簿：`docs/part4/00_parameter_register.md`。调参记录模板：`03_pid_tuning_record.md`。多任务实时验证模板：`04_realtime_verification.md`。这些模板中的“待测”不是已通过报告。

**当前：代码已提交，未取得第4阶段本机/ARM/QEMU/PyBullet测试实测通过记录。** 

### 运行时安全性补充（同日修订）

- Trajectory Task 同时接收 `JOINT`、`POLYNOMIAL`、`CARTESIAN` 三类请求，三次/五次插补不再只是离线函数；多项式轨迹每个输出点验证位置、速度和加速度限值。
- 每个 Cartesian IK+Selector 的最终连续角，都先规范化并经既有 `kinematics_forward()` 回代，与原位姿比较（位置毫米距离和旋转矩阵最大元素差）。门限唯一归属 `config/stage4_motion_config.c`。
- 若控制任务遇到反馈超时或计算异常，会通过 `abort_requested` 取消规划任务的后续参考点；轨迹任务自己也会每周期检查反馈新鲜度，防止反馈断开后继续计时并在恢复时跳点。
- 目前阶段不新增 UART 运动指令字节协议；运动请求由 Application 公开 `motion_pipeline_submit_*` API 注入。上位机到运动请求的正式命令映射属于后续系统集成，不得在第4阶段未验收时悄悄复用现有反馈帧当新指令。

### Windows/MSVC Host 测试的数值精度说明（2026-10-26）

发现 `trajectory_test_main.c` 中3条运行时间预期使用了 `1e-10 s`（0.1纳秒）误差门限，与项目公共 `ROBOT_JOINT_ANGLE_UNIT_DEG=0.01F` 的 binary32 表示精度不相容。该浮点值提升为 double 后仍保留原来的单精度舍入：仿真测试对 1.5/2.5/1.0秒的计算产生约 45/45/11纳秒偏差。已将**仅这3条总时间、巡航时间断言**改为统一 `TRAJECTORY_TEST_TIME_TOLERANCE_S=1e-6`。这不修改生产规划算法，不更改关节速度/加速度/IK 容差；原来其他时间/物理量断言未更改。

Visual Studio 是多配置构建器，CTest 必须带 `-C Debug`，不能把缺少配置导致的 `Not Run` 当成函数测试不通过。

> 状态：补丁已提交，需本机重新编译与实际执行 CTest；此处不宣称已经通过。 
