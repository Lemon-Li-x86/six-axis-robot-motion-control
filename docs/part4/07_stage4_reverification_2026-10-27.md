# 第四阶段项目大纲对应复验记录（2026-10-27）

## 验证基线和原则

- 原始依据：《六轴工业机器人嵌入式运动控制固件开发与仿真验证 实习项目大纲》第4阶段：关节空间、笛卡尔、增量PID、FreeRTOS任务与三个阶段产出。
- 冻结快照：`backup/stage4-verified-2026-10-27`，原始 SHA `1ee9ce02d6954e8e9ad4e095611dfd78f0795ab9`。
- 测试扩展与缺陷修复：只在 `feat/stage5-system-integration-2026-10-27`，不改冻结快照或main。
- 测试入口：`.github/workflows/stage4-spec-acceptance.yml` 与 `tools/ci_stage4_algorithm_qemu.sh`，并保留原 Windows `tools/test_stage4_once.ps1`。
- 此处只认可**实际执行日志**，不以“代码看起来齐了”代替测试。

## 首次完整复验（原PID增量D测试尚未加严）

[GitHub Actions #40442794981](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40442794981) 提交 `90bbbfbf`。五个Job完成且全部success：

| 验证层 | 实测判据 | 结果 |
|---|---|---|
| Stage2 Host | 4个原生CTest | PASS 4/4 |
| Stage3/4 Host | 6个原生CTest | PASS 6/6 |
| ARM QEMU路径 | `g_stage4_test_report.checks/passed/failed` | 14/14/0 |
| ARM QEMU运动学 | FK、IK、Selector与FK回代 | 128/128，全程无失败 |
| ARM QEMU 近奇异 | 条件化IK参考输入 | 2/2 |
| ARM QEMU FreeRTOS | 失联故障注入 | 13/13，failed=0 |
| Native C + PyBullet | J2小幅与六轴小幅运动 | 两个Job success，详细JSON/CSV上传为Artifacts |
| Windows PowerShell | 单命令验收脚本AST | PASS |

命令输出在ARM/QEMU构建中始终`ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF`，以上结论不代表实体机器人可安全执行。

## 新增微分项覆盖找到真实缺陷

原`stage4_cartesian_pid_host`对P项和I项有检查，但**没有证明Kd独立启用后启动历史误差正确**。本轮添加`test_pid_derivative_startup()`，将微分配置成Kd=0.5、Kp=Ki=0，并验证：

- 初始非零误差时不发生D kick；
- **连续两个完全相同的非零误差样本**不凭空产生D脉冲；
- 误差下降时D项输出正确反应；
- 后续误差恒定时D项归零。

新断言在 `d39818b6` 的 [Actions #40442942554](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40442942554) 中真实失败：PID用例45条检查中3条失败；不是环境或工具误报。

**根因**：`pid.c` 为防止首帧微分冲击把前前次误差临时设为负当前误差，但首帧结束错误地留存了0作为前前次误差。第二帧计算`e[n]-2e[n-1]+e[n-2]`时对持续误差产生虚假的脉冲。

**修复**：提交 `4e1ff112` 在首次调用计算P时显式使用当前误差、D恒为0，同时为下一帧存储恰当的两份当前误差历史；保留后续普通增量D的差分公式和积分/饱和逻辑。正常默认仿真`Kd=0`仍与此前一致。

本次修复后进入新的完整CI回归 [Actions #40443009690](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40443009690)。**在新CI实际完成之前，不应标注修复后整体PASS。**

## 尚不能按第四阶段原大纲完全结项的测量

1. 多轴动态性能数据仅覆盖小幅两类轨迹、默认负载和P初始增益；没有多速度、多负载与P/I/D完整调优矩阵。
2. 调度设计已有任务周期/优先级/栈配置和ARM/QEMU任务计数，但没有实测的周期抖动P99、最坏任务执行时间与长时满载栈水位。
3. 完整UART“运动请求→轨迹→PID→仿真反馈”的全链路属于原大纲第5阶段系统集成内容，不能把两条独立CI测试冒充同一条串口闭环。
4. 没有实体驱动独立安全停机和工业机器人安全认证，不能使用这些非驱动测试直接控制实体机械臂。

**判断**：第四阶段软件核心和已覆盖的计算/仿真/RTOS自检可以独立验收；PID调优数据覆盖和实时性测试报告尚需补强。被冻结的Stage4快照保留为原始状态，新的Kd问题/修复清楚记录在第五阶段开发历史。

## 2026-10-27 修复后完整回归结论（实际CI）

[GitHub Actions #40443009690](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40443009690)，被测代码含 `4e1ff112` 的Kd历史误差初始化修复。实际核对 **5/5 Job success**：

| 项目 | 实测 |
|---|---:|
| Stage2原生Host | 4/4 PASS |
| Stage3/4 Host（新增D项边界共45个该用例断言） | 6/6 PASS |
| ARM QEMU Stage4轨迹 | 14/14 PASS |
| ARM QEMU FK/IK/Selector/回代 | 128/128 PASS |
| ARM QEMU近奇异参考IK | 2/2 PASS |
| ARM QEMU FreeRTOS反馈中断故障注入 | 13/13 PASS |
| Windows PowerShell AST | PASS |
| Native C + PyBullet J2单轴、六轴同步 | 2/2场景PASS |

独立QEMU运动学实测上界：`worst_position_error_mm=0.0893113`，`worst_rotation_element_error=0.000167578`。这些是128组样本内的最大误差，**不能外推整个UR5工作空间**。

PyBullet真实数据：J2单轴跟踪RMSE `0.1300456°`，六轴同步试验中J2 RMSE `0.1306400°`；两场景J2超调均0°、六轴最终最大关节目标误差均 `0.02°`、样本各253。对默认 `Kp=0.2,Ki=0,Kd=0` 而言，本次修复不改变对应闭环数字；新增D项回归实际验证此前缺失的非零Kd行为。

**不可混淆的版本结论**：

- 第四阶段冻结备份仍精确指向 `1ee9ce02`，**原样保留，内含旧版Kd初始化缺陷**。它通过了冻结时的旧测试，但不能宣称通过2026-10-27新增的Kd边界测试。
- 缺陷修复 `4e1ff112` 只存在于当前Stage5开发分支，已由本次完整CI实际覆盖；未将Stage5避障模块回灌到冻结备份。
- 第四阶段原任务书的**核心算法回归**已在含修复的当前分支通过。实时周期抖动P99、最坏运行时间、长时满载栈水位以及更全面的PID整定仍属**没有验收证据**的交付缺口，不得标记100%完工。

## 2026-10-27 继续补全：短时QEMU实时性测量

此前本报告写“周期偏差P99和栈余量还没有实测”，是当时的历史结论。**后续已新增** `docs/part4/08_stage4_qemu_timing_and_stack_2026-10-27.md`，并在 [Run #40444477809](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40444477809) 的真实ARM/QEMU中获得三任务的P99唤醒间隔偏差、P99和最大处理经过时间、栈高水位，以及测试ELF内存分段和.map。历史报告保留原始结论方便审计，当前状态请看08。

量化数据仅代表约1.2秒健康模拟反馈期间的QEMU虚拟时钟观测；**不能**把它写成实际Cortex-M4 WCET/CPU占比或满负载30分钟验证结果。新自动验收门限为QEMU的P99周期绝对偏差不超过名义周期10%，且最大处理时间小于该周期，属于测试政策，不是部署控制参数。
