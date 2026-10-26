# 第4阶段 FreeRTOS 实时性与联调验证模板

## 当前证据

- 新增代码：`application/tasks/{trajectory_task,control_task,monitor_task}.c`、`application/services/motion_pipeline.c`。
- Control/Trajectory/Monitor Priority: 4/2/1，协议接收/发送任务依旧为 3/1。
- Control/Trajectory/Monitor Period: 10ms/20ms/100ms，配置来自 JSON 生成宏。
- 三任务的栈深：512/1024/384 *words*，不是 bytes；增大只是给 IK 分支计算预留运行空间，**不是栈分析已通过**。
- Request Queue / Target Queue 都只保留最新一份；任务时间线有明确单一 writer。
- Control 通过 `motor_state_get_feedback()` 获取整组已加锁的六轴反馈，检查反馈有效性和超时。输出默认关断。
- Monitor 通过 `uxTaskGetStackHighWaterMark()` 读取剩余栈水位。

## 每次必须记录

| 测试 | 判据 | 本次结果 |
|---|---|---|
| ARM 编译 | Stage4 OFF / TASKS ON / OUTPUT ON 三种配置均构建 | 待测 |
| QEMU 纯算法测试 | `g_stage4_test_finished=1`, `failed=0` | 待测 |
| Host 原生算法 | 4个 Stage4 CTest 全通过 | 待测 |
| Stage2 Host 回归 | 4个既有 CTest 全通过 | 待测 |
| FreeRTOS 任务启动 | 5个任务均存在、无启动失败 | 待测 |
| Stack High Water Mark | 无溢出，记录每任务峰值余量 | 待测 |
| FreeRTOS 队列 | 新请求覆盖旧请求，Target 队列不会积压 | 待测 |
| 反馈过期 | 停止生成新 Motor Target；fault 计数上升 | 待测 |
| 控制周期抖动 | 采集执行时间、唤醒间隔、P99 和最大值 | 尚无采样器，待扩展 |
| IK路径故障 | 无解/近奇异/限位时报告拒绝且不继续指令 | 待测 |
| 闭环 PyBullet | 位置 MAE、RMSE、超调、稳态误差 | 待测 |
| 长时间 | 至少30分钟仿真无异常/泄漏/栈溢出 | 待测 |

## 开关与安全条件

`ROBOT_ENABLE_STAGE4_TASKS` 默认为 OFF。
`ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` 默认为 OFF。
必须明确打开，且 Task 开启成功以后才允许闭环控制。当前代码还没有完整的独立急停总线、机械碰撞检测或实物硬件实时验证。不要在实体机器人上启用此实验选项。

## 不能伪称已通过

没有 ARM/QEMU 工具链运行输出、GDB 变量和 PyBullet 数据就不能填写准确 PID 调优数值、CPU WCET、任务调度 jitter 或电机位置跟踪精度。
