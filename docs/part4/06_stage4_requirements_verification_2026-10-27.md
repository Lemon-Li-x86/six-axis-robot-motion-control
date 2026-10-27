# 第4阶段按原始项目大纲逐条复核与再次测试（2026-10-27）

## 一、权威依据、版本与边界

**外部权威要求**：用户上传的《六轴工业机器人嵌入式运动控制固件开发与仿真验证 实习项目大纲》PDF，第3页（第4阶段「运动控制与路径规划开发」，四项核心任务、三项阶段产出）。严禁仅因之前内部报告写了「完成」就把原始要求标注为全验收。

- 第4阶段正式冻结基线：`backup/stage4-verified-2026-10-27` / `1ee9ce02d6954e8e9ad4e095611dfd78f0795ab9`
- 此报告及新的重新验证工作流只写入 `feat/stage5-system-integration-2026-10-27`，不更改第四阶段冻结分支和旧备份。
- 自动回归与系统级物理串口闭环是不同证据级别。
- 状态用「核心代码已有」「指定回归通过」「文档/性能尚需补测」「未实现」区分。

## 二、第4阶段核心任务按项目大纲核验

| 原大纲具体要求 | 代码定位 | 已有实验证据 | 判断与不足 |
|---|---|---|---|
| 1. 三次、五次关节多项式；满足起终点位置、速度、加速度约束 | `algorithms/trajectory/polynomial_trajectory.c`、`trajectory_task.c` | 端点约束；32组×1001点密采样对照；ARM算法20项、FreeRTOS多项式8项 | **指定测试通过**。三次匹配起终位置/速度；五次还匹配两端加速度。执行前检查整段位置、速度与加速度的端点及内部极值，越限计划不发布首个参考。 |
| 1. 梯形速度曲线、最大速度和加速度可配置 | `algorithms/trajectory/trapezoidal_profile.c`、`config/stage4_motion_config.c` | Host测同步、梯形/三角、边界与速度/加速度样本 | **通过现有样本**，不是所有路径负载的动力学保证 |
| 2. 直线、圆弧插值，配合IK转离散关节序列 | `algorithms/trajectory/cartesian_line.c`、`cartesian_arc.c`、`cartesian_waypoint.c`、`application/tasks/trajectory_task.c` | `stage4_cartesian_pid_host` / `stage4_cartesian_ik_host` | **离散几何/IK通过**，没有实测全程连续关节加速度和全机器人连杆碰撞 |
| 2. 轨迹插补周期与控制周期匹配 | `application/tasks/trajectory_task.c`、`config/robot_defaults.json`、`motion_pipeline.c` | Trajectory 20ms，Control 10ms；请求周期校验；QEMU P99/最大周期偏差，见08报告 | **配置、运行和QEMU计时已有证据**；不代表实物硬实时保证 |
| 3. 单关节增量式 PID，可独立配Kp/Ki/Kd，积分限幅、输出限幅、死区 | `algorithms/control/pid.[ch]`、`joint_controller.c` | Host PID测试和六轴反馈调节仿真 | **实现且样本测试通过**；配置当前Kp=0.2, Ki=Kd=0，增益仍为小幅仿真初值 |
| 3. 依据仿真关节反馈整定，优化位置跟随精度 | `tests/simulation/stage4_sim_bridge.c`、`simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py`、`docs/part4/03_pid_tuning_record.md` | 单J2与六轴同步小位移；J2 RMSE约0.130/0.131°、超调0、最大最终误差0.02°；四档Kp对照 | **初步调优数据具备**；尚缺多速度、多负载、多起终点及独立Ki/Kd验证，不能称全面整定 |
| 4. FreeRTOS规划、PID、通信、监测任务及优先级/周期/堆栈 | `application/tasks/`、`application/protocol_tasks.c`、`config/robot_defaults.json` | QEMU 任务运行/诊断；Control 4/10ms/768words，Trajectory 2/20ms/1024words，Monitor 1/100ms/384words，RX3/TX1 | **已实现**，并经QEMU任务自检 |
| 4. 用队列/信号量同步，保障实时性 | `motion_pipeline.c` 单槽请求/参考队列；`motor_state.c` 的互斥锁；`task_monitor.c` 栈水位 | ARM/QEMU反馈失联故障注入13/13通过，RTOS正常任务计数增长 | **同步机制已实现**，但「保障实时性」需要 P99/最大周期偏差、执行时间、满载栈余量才能正式结项 |

## 三、3项阶段产出的实际结项判断

1. **路径规划与运动控制模块完整源码**：核心模块和API均具备；按任务书要求的算法软件交付基本成立。与外部UART的完整运动请求闭环属于原大纲第5阶段的「全模块系统集成」，不要反向把Stage5工作冒充Stage4已完成。
2. **PID参数调优记录与轨迹性能分析文档**：已有 `03_pid_tuning_record.md` 与真实PyBullet小幅试验，但**多场景/多负载调优及某些限幅统计尚缺**。可作为初期报告，不宜称覆盖充分的最终整定报告。
3. **多任务调度设计与实时性验证报告**：已有 `04_realtime_verification.md`、实际ARM/QEMU任务执行/失联检测，但是周期抖动P99/WCET、任务CPU占用、长时与满载栈水位没有完整实测。**报告还不完整**。CPU占用与 .map 全系统优化也是项目大纲第5阶段任务。

## 四、重新测试：不修改第四阶段冻点

触发 `.github/workflows/stage4-spec-acceptance.yml`，在新第5阶段分支上检验原有Stage3/4生产模块（第五阶段APF放在独立目录，未修改这些控制核心）：

- Host 六组CTest + 关键参数周期静态一致性；
- 真实ARM Cortex-M4 QEMU单独构建及13项反馈失联自检，保持命令输出OFF；
- 原生C轨迹/PID/限速共享库与PyBullet单轴、六轴动态测试，记录JSON/CSV；
- Windows PowerShell 解析检查；
- 第四阶段冻结备份使用SHA比较，不对冻结分支提交任何测试性修改。

注意：通过以上CI只表示核心功能**当前回归通过**。测试没有包含：完整UART运动指令路径、硬件独立急停、整机碰撞、30分钟满载、P99/WCET、多负载PID闭环。这些必须独立报告。

## 五、本机运行（仅在云端通过后需要时）

在无未提交改动的干净仓库中测试冻结版本：

```powershell
git fetch origin
git switch --detach origin/backup/stage4-verified-2026-10-27
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
```

脚本在 `build/stage4-final-validation.log` 保存日志。该操作**只切换本机工作树，不移动备份远端分支**；如果本机有未提交修改，应先保存自己的改动，不要强制checkout。

**整体验收结论**：Stage4核心代码及基线测试已经有先前证据，但仍有调优覆盖、实时性能报告的交付缺口。等此次CI真运行完成后补充对应运行号/结果，不能先填写「PASS」。

## 2026-10-27 本机重新复验结果（更新）

上面的历史段落反映创建本报告时的状态。当前更新以本机九步统一脚本 `tools/test_stage4_once.ps1` 为准：Stage2 Unity 25/25、Stage2 ARM/QEMU协议8/8、Stage3/4 Host CTest 6/6、Stage4轨迹QEMU14/14、IK选解并FK回代128/128、参考近奇异IK 2/2、FreeRTOS自检29项失败0、Native C Smoke通过，PyBullet单轴与六轴两场景通过。详细运行状态和指标定义见 [`phase1-4_acceptance_status_2026-10-27.md`](../phase1-4_acceptance_status_2026-10-27.md) 与 [`08_stage4_qemu_timing_and_stack_2026-10-27.md`](08_stage4_qemu_timing_and_stack_2026-10-27.md)。

更新不改变本报告中对多负载PID、长时高负载、任务CPU占比和实物实时性的限制判断。
