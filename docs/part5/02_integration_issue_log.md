# 第五阶段集成问题追踪台账

创建于 2026-10-27。用于登记现象、证据、根因、修改提交和复测。**未复现或未查清的根因不得编造。** 严重级别用于安排工作，并不代表已出现实物事故。

| ID | 优先级 | 当前现象 / 缺口 | 状态 | 证据 / 下一步 |
|---|---|---|---|---|
| S5-001 | 实机专用（本项目N/A） | 缺实体电机独立急停、失联制动和安全联锁设计；现有Control故障仅阻止继续下发新目标 | **纯仿真交付不阻塞**；禁止据此实机动作 | `docs/part4_archive/05_global_architecture_review_2026-10-26.md`；仅允许非驱动模拟 |
| S5-002 | P1 | 接收运动请求尚未与 `motion_pipeline_submit_*` 建立 UART 方向清晰的完整接口/ACK/NAK | 未实现 | 审核 `communications/protocol.h` 与 `protocol_commands.c`，再定兼容协议 |
| S5-003 | P1 | APF当前仅检查EE点到静态长方体膨胀AABB的单段，不包括整臂连杆碰撞 | 正在开发（第一切片CI通过） | CI [#40427456854](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40427456854)；下一步连杆碰撞与IK |
| S5-004 | P1 | APF可陷入局部极小/路线不完整；未做整段轨迹重规划和到达率测试 | 未实现 | 当前 `ROBOT_STATUS_ERROR_NO_SOLUTION` 明确拒绝，不伪造可达 |
| S5-005 | P1 | FreeRTOS任务CPU占比、控制周期P99/WCET、长时抖动缺可靠实测 | 待测 | 添加测量定义与任务状态统计后再量化 |
| S5-006 | P1 | 尚未做跨模块串口丢包、半帧、延迟和崩溃恢复的全链路故障注入 | 待测 | 复用Stage4已通过的反馈陈旧自检；增加串口流异常 |
| S5-007 | P2 | ARM固件 `.map` 文件的RAM/Flash/静态全局占用未形成优化前后对照 | 待分析 | 设立构建快照和明细归档，禁止凭感觉裁剪 |
| S5-008 | P2 | C/Python UART常量目前靠回归一致性检查，尚未统一生成schema | 待改进 | `tools/check_protocol_mirror.py` 是防漂移，不代表功能完整 |

## 记录规范

每次真实问题新增一条 `S5-xxx`：出现条件、具体日志/测试命令、原始表现、定位方法、**已验证根因**、修复提交、回归结果以及尚存风险。未定位根因写“待查”，不能用推测冒充结论。

初始条目是**系统缺口清单**，并非8个新触发的运行异常。新增Stage5 C算法与PyBullet探针的CI通过记录不意味着UART、APF全路径或任务CPU优化已验收。

## 2026-10-27 新增真实缺陷 S5-009：截断帧吞掉下一帧头

**级别**：P1，通信可靠性。**状态**：已修复，生产C字节流单元回归通过；UART/FreeRTOS端到端复验仍待做。

- **复现**：生产 `protocol_build_joint_target_frame()` 产生六轴数据帧；故意删除载荷中的1个字节后，直接追加一帧完整有效帧。旧 `protocol_parser_process_byte()` 将第二帧的 `0xAA` 当作前一损坏帧的校验字节。
- **首次真实失败证据**：[Actions #40445607304](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40445607304)：`protocol_stream_fault_host` 的14个断言中3个失败（帧数和解码输出不正确）。
- **根因**：`PROTOCOL_STATE_READ_CHECKSUM` 校验错误时直接重置，未处理当前校验字节本身也是新帧 `PROTOCOL_HEADER_0` 的情况。并非C/Python协议常量不一致。
- **修复**：`communications/protocol.c` 在校验失败后复位，若当前字节等于权威 `PROTOCOL_HEADER_0`，保留 `WAIT_HEADER_1` 解析状态。非法长度、未知状态仍按原逻辑处理。不添加第二套帧格式常量。
- **验证范围**：`tests/host/protocol_stream_fault_test.c` 新增分块、噪声、坏校验、超长、丢字节后的有效重发、显式超时复位测试。修复提交 `3a36ca62` 之后 [Actions #40445770760](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40445770760) 原生Stage5 Host测试成功；Stage4 Host套件同步成功。完整QEMU/物理仿真场景仍须查看其独立CI结果。
- **已知边界**：该解析器不自带接收超时功能；半帧停滞的超时策略仍应由UART/ProtocolRX拥有。这次测试证明软件字节流解析器恢复，不等于真实UART DMA/中断、串口反向协议以及端到端闭环已通过。


## 2026-10-30 进度复核（保留上述 10-27 原始问题状态作历史记录）

下表是**最新证据状态**，与文件开头建档当日的待办状态不同。正式阶段结项须优先查阅 `docs/part5/15_stage5_exit_review_2026-10-30.md`，不能将两天前的“未实现”当作当前结论。

| 编号 | 现状及证据 | 结项判定 |
|---|---|---|
| S5-001 | 本项目只运行虚拟 UART/ARM QEMU/PyBullet；真实伺服电机及安全联锁未启用，代码不得用于真机控制 | 不属仿真交付范围 |
| S5-002 | QEMU 经 `CMD_REQUEST_JOINT_MOVE` 接收关节轨迹，实际 ACK、反馈到位与失败处理均已验证；4/4与28/28静态避障路线成功 | 已验证仿真关节执行方向 |
| S5-003 | C APF/IK、真实 PyBullet AABB 和全臂离散碰撞预检，与运行时各关节实际物理净距检查成功 | **已验证静态长方体具体场景**；非连续时间/动态障碍证明 |
| S5-004 | 局部极小值仍可返回 `NO_SOLUTION` 安全拒绝；通过上层显式 via 与 C 几何检验的航点压缩成功避开一个实际障碍 | 算法限制已明确，非任意障碍保证 |
| S5-005 | QEMU Timer0/GDB 已采 CPU、处理窗口/周期抖动最大与 P99、FreeRTOS 栈余量；真实数值见结项审查报告 | **短时基线通过**，持续满载尾部时延后续验证 |
| S5-006 | 生产 C 字节流损坏/分片测试、QEMU UART 半帧超时和最终反馈过期处理有真实验证；计划中的全面故障矩阵仍未全覆盖 | 核心故障恢复已验证，其他边界需追加 |
| S5-007 | ARM GNU ld map 解析、ELF真实构建与同配置前后对比已完成；FLASH增加48B、RAM不变 | 内存审计完成，**没有证明内存缩减** |
| S5-008 | C/Python 协议镜像通过 `check_protocol_mirror.py` 防漂移测试 | 双侧一致性验证完成，独立 schema 生成仍可作未来改进 |
| S5-009 | 损坏帧字节复用帧头导致丢失下一帧：生产C解析器重同步修复，Host 回归通过 | 已修复并回归 |
| Issue #22 | 反馈 Tick 与最新样本不一致导致 `NOT_READY(-7)`，在 PR #25 中修复，PR #26 用28段不压缩真实路径确认 | 已关闭，有端到端证据 |
| PR #15 / #28 | 原生 GUI 旧版 xvfb 第2段超时，正在基于最新固件重复原生窗口测试 | **未签收**，不得拿 PyBullet DIRECT 通过代替 |

5.1~5.4 的阶段报告、ARM Map/FreeRTOS CPU/抖动/栈水位数据与 PR/CI 链接详见第 15 号文档。第六阶段的完整工业搬运、长时连续运行和交付资料独立验收。


## 2026-10-31 阶段结项更新：S5-010 与原生 SAFE JOG 实测

- **S5-010 / [Issue #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34)，OPEN**：旧 Linux 原生 GUI `six_axis --no-hold` 演示在首段轨迹 60s 后仍未满足独立物理收敛判据（0/3）。[Actions #41206909042](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41206909042) 的 `stage5-gui-nodrag-xvfb` 证据显示 `requests_accepted=1`，UART/固件故障计数零；最终物理状态距首段实际目标约 0.24°，超过本地权威 `SETTLE_TOLERANCE_DEG` 0.15°。**原因未确认**，不修改参数掩盖失败。
- **Windows 原生 SAFE JOG 通过**：上传的 `stage5-j6-manual-evidence.zip`、独立 Windows 鼠标重复验证 `stage5-caption-j6-verified` 均表现为原 C APF 4/4、J6 +0.5°、C 回位 1/1、最终误差 0.02°、固件故障零，按真实物理反馈、UART 和诊断结束；过程细节见 [16_safe_jog_click_review_2026-10-31.md](16_safe_jog_click_review_2026-10-31.md)。
- **Linux 同物理模型 J6 脚本化 GUI 自动回位通过**：同一 CI Run 的 `native-gui-scripted-j6-xvfb` 独立 PASS，注意脚本输入不代表 Windows 鼠标事件覆盖。
- **遗留窗口偶发无响应**：历史 GUI 断连日志不能证明窗口消息循环死锁；已补充 Windows 原生窗口健康探测、按钮接收记录和有限退出，仍保留复现观察，不虚构根因。
- **签收策略**：Stage5 静态避障主链路 **有保留结项**，全 GUI 兼容性不签收；[Stage 5 最终签收与 Stage 6 交接](17_stage5_final_acceptance_2026-10-31.md) 是最新权威结论。PR #33 保留 Draft 直到未通过的门禁问题解释并修复。


## S5-011（2026-11-01）：Windows 连续 SAFE GUI JOG +2° 被零笛卡尔路程保护误拒绝

- **原始现场证据**：用户上传 `stage5-final-acceptance.zip`，第一次 J6 +0.5° 已完成真实 C APF/UART 回位，HOLD 实测误差约 0.02°，随后同一窗口第二次请求 J6 +2° 被 `recovery_preflight_failed` 拒绝；`C APF zero-Cartesian-leg return too far in joint space`。出站的全连杆碰撞预检**通过**，不是碰撞故障。
- **具体根因**：第二次点击前实际 J6 14.17°，原 HOLD 关节目标 J6 14.15°；合法 +2.00° 点动后，与原目标的最大差变成 2.02°。原 C APF 笛卡尔终点已到时的“无授权任意 IK 跳变不得超过 2°”通用保护把合法 GUI 关节增量与前次跟踪余差混为一谈。
- **修复**：`stage5_safe_gui_jog.py` 记录按原有限位/净距和 C 回位预检确认为安全的**单关节**操作员点击（起始六关节、J6 轴、离散 0.5–2° 原始增量），通过 `stage5_avoidance_uart_gate.py` 只传给 `stage5_obstacle_hold_recovery.py` 的零笛卡尔路径特殊分支。要求点击前姿态在原有 0.15° HOLD 到位容差内；偏移后的实测状态必须与点击增量一致；**真正的每段全连杆离散碰撞检查、原始速度/限位/样本预算及 C/UART ACK 不变**。对没有合法 GUI 授权的普通三度跳变及错误 IK 分支仍原样拒绝。实际恢复完成后销毁该授权。
- **防复发**：添加与真实 PyBullet/生产 C APF 接驳的 0.02° HOLD 残差 +2° 回位测试及伪造授权/3° 跳变负例，Linux/Windows CI 同时运行；新增 Linux Xvfb 原生 GUI 0.5°→回位→2°→回位自动闭环测试；Windows `-RequireTwoStepJog` 参数与独立 `--two-step` 审核要求 J6 两次成功，任何被拒绝的点动都使最终 PASS 无效，杜绝“0.5° PASS 掩盖 2° 拒绝”。
- **状态**：修复代码已在 PR #35；必须等待同提交的自动化结果及 Windows 同一轮实际鼠标复测才可关闭此项。完整流程及不可省略的判据见 [18 号 Windows 验收指南](18_stage5_windows_final_two_step_acceptance.md)。


## S5-012（2026-11-01）：GUI 点动次数错误封顶、拒绝仍被判 PASS、第二次回位末端残差

- **现场输入证据**：Windows 操作员提供 `stage5-windows-final-2step.zip`；同一运行 `safe_gui_jog_applied=3`、`safe_gui_jog_completed=3`、`hold_recoveries=3`，但 5 次点击中第 4、5 次 `safe_gui_jog_rejected`，原因为 `not_idle_or_recovery_pending_or_limit`；QEMU 控制/规划故障计数均为零，碰撞净距通过，原报告居然 `passed: true`。此故障不是用户误操作，也不是末端物理碰撞。
- **根因 A（交互设计）**：SAFE GUI JOG 的 `JOG_MAX_SUCCESSES=3` 对整个 UI 会话封顶，把临时 CI 样本次数泄露为用户按钮终身次数；即使前一次 C/UART 已实测回位，按钮也不再可用。
- **根因 B（验收误判）**：原运行裁决仅检查 `applied == completed == hold_recoveries` 且 `applied >= 1`，未统计后续被拒绝的按钮；独立 checker 又将 `expected=1/2` 误写成严格**总动作数**而不是**规定前缀**，导致正常多次点动不合格或错误地掩盖问题。
- **根因 C（集成联调）**：过去 Linux 原生 GUI 双次真实 C APF/UART 回位的 CI，第二次 J6 +2° 固件已 ACK 但停在距 HOLD 约 0.42° 并保持 BUSY；同一末端收敛问题需要受控的真实反馈闭环纠偏，而不是延长 timeout 或放宽误差门禁。
- **修改**：正常操作按钮无会话成功次数限制；只有 `BUSY`、非 HOLD、FINISH/CANCEL、关节机械限位、全臂离散碰撞或生产 C 回程预检失败会拒绝。每次真实 UART ACK + 物理稳定到位后重新 READY，合法点击无次数耗尽。生产裁决直接要求 `refused==0`、FINISH 事件和所有物理回位完成；独立 checker 的两步验收仅规定前缀 `J6 +0.5°，J6 +2°`，**其后无限合法动作继续计数，且必须 100% 回位**。新增 `native-gui-j6-five-repeat-xvfb` 从同一 QEMU/FreeRTOS/PyBullet 真实循环验证 `[+0.5,+2,+2,+2,+2]` 并核对每次动作及全连杆净距。第二次回位独立新增最多 2 次、有改善且碰撞校验通过的实测端点重规划，原 0.15° 到位标准保持不变。
- **当前状态**：修改在 PR #35，`main`/Stage4 未更改。**仅在相同 HEAD 的 Windows/Linux、双步和五次连续集成 CI 均 PASS，再完成用户本机五次按钮验收后才关闭**。GUI 回归修复不等于原 PDF 5.2 固件内完整 APF 或 5.3 全部性能优化已经完成。


## S5-013（2026-11-01）：静态障碍 APF 原先未在固件规划任务真正运行

- **现象**：历史 Stage5 虽已把 `potential_field.c`、`stage5_apf_joint_path.c` 编入 ARM 固件，完整 APF 路线仍由仿真主机同源码共享库编排，然后按关节航点通过 UART 控制；原 PDF 要求的是“使用 C APF，嵌入嵌入式固件的路径规划模块”，两者不能视为等价。
- **根因**：现有 `motion_pipeline` 只有 JOINT/POLYNOMIAL/CARTESIAN 三种 FreeRTOS request；`CMD_REQUEST_APF_STEP` 是 ProtocolRX 上直接计算末端下一点的**非驱动预览**，既没有 ARM IK，也没有调入 Trajectory Worker，更没有物理全连杆安全门禁。
- **修复**：新增事务式 `CMD_REQUEST_APF_JOINT_PROPOSAL=0x05`→`CMD_APF_JOINT_PROPOSAL_RESPONSE=0x88`；ProtocolRX 只处理字节及入队，FreeRTOS Trajectory Task 从实时 `motor_state_get_fresh_feedback` 读取连续关节角，调用生产 C `robot_apf_step` → C FK/IK 解算、最近合法关节选解、FK 残差、梯形速度/加速度检验，打包 UART 真实候选点返回。**规划响应绝不直接驱动电机**。PyBullet 使用真实 UR5 URDF 全连杆扫掠路径预检，批准后才发送原有 `CMD_REQUEST_JOINT_MOVE` 单独执行。无隐式驱动、无越过全连杆门禁。解析失败/浮点非法/不可达/过期反馈均必须失败，不返回旧候选。
- **实测**：[Actions #41265049762](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41265049762)，工件 `stage5-embedded-arm-apf-verified-physical`。ARM FreeRTOS 实际 APF+IK 候选 1 次、返回 `[3046,-4683,6287,1613,539,-716]` centidegree，PyBullet 采样 5 次全连杆，最小测得净距 40.95 mm，批准后 QEMU C 轨迹/PID 执行，真实关节变化最大 4.38°、15 帧收敛、最终误差 0.07°，新 UART 错帧 0。主机未运行 C APF 共享库替固件决策。
- **整路验收更新**：[Actions #41266022023](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41266022023) `stage5-embedded-apf-uart-full/result.json`：FreeRTOS C APF+IK **28 段在线迭代规划→全连杆检查→C/UART 实际执行 28/28**，`route_complete=true`，最终六轴误差 0.05°、EE 末端位置偏差 0.7977mm、UART 校验错误 0。场景 JSON 供应高层 via，PyBullet 全连杆碰撞负责外部安全批准，不是任意障碍无监护的全自主控制。**原 PDF 5.2 的指定静态场景固件内 C APF 整路演示已具备可复现证据**。

## S5-014（2026-11-01）：FreeRTOS 高耗时监测与链接内存裁剪及实时性测量

- **现象**：原 Monitor 任务每 100ms 都调用三次 `uxTaskGetStackHighWaterMark`，该 FreeRTOS 高水位统计对 StackType_t 栈检查存在随分配栈深度增长的成本。原控制周期抖动 QEMU 测量存在，但没有同编译配置真正优化对比。ARM map 尚有不可达函数和没有引用的全局数据占用 FLASH。
- **修复**：`robot_defaults.json` 唯一配置 `stage5_monitor.stack_watermark_sample_period_ms=1000`，由生成器验证与 monitor 周期整数倍关系，Monitor 仍每 100ms 监测一次，但每 1000ms 才重新扫描栈高水位。Stage5-only `ROBOT_ENABLE_STAGE5_MONITOR_STACK_OPT` 可关闭以 A/B；同构建增加 `ROBOT_ENABLE_STAGE5_LINK_GC` 对 ARM 函数和数据启用 section 级 `--gc-sections`。Stage4 冻结路径默认 OFF，PID/电机参数不改、任务优先级不在无证据情况下盲调。
- **CPU/调度 A/B**：在同提交 [Actions #41265306540](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41265306540) 上真实 QEMU/GDB/Timer0 约 4.72s 自测，Monitor CPU **0.07% → 0.04%**、Control **1.09% → 1.06%**、Trajectory **0.19% → 0.18%**。但本次 Control 抖动 P99 **0.01328ms → 0.18532ms**、Trajectory **0.03136 → 0.18788ms**、Monitor **0.02184 → 0.13488ms**，出现退化；另一短时 run 则 Control 抖动改善。说明虚拟 QEMU 时间基准负载波动较大，**不能宣称该优化已经稳定降低控制抖动，不能宣称 WCET 保证**。
- **ARM .map 同条件 A/B**：同一提交相同编译器/版本/使能特性、仅切换链接 GC，FLASH **105256 → 87384 B**（−17872 B，约 −17.0%），RAM 链接静态占用 **33796 → 33796 B**（无下降）；真实 map 文件、CPU 时序和 JSON 由上述 Actions 工件 `stage5-freeRTOS-cpu-trace` 归档。**RAM 无法说成优化成功**。
- **IRQ/调度审查**：当前 QEMU MPS2-AN386 NVIC 为 8 priority bits，`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=0x80`；UART RX/TX ISR、Timer1 ISR 都使用 0x80，满足 FromISR 优先级约束。Control/ProtocolRX/Trajectory 优先级来自 JSON 为 4/3/2，协议解析只入队；重 APF/IK 在低优先级 Trajectory 执行，避免阻塞高优先级 UART RX。调度使用现有 `motion_task_wait_next_period` 对过期激活丢帧而非忙循环。**未有单独 IRQ 0x80 vs 不同 IRQ 配置对照实验，不能写“IRQ 优先级优化使抖动降低”**。
- **验收限制**：CPU 数据来自短时 QEMU 自测（低负载）、不代表完整 PyBullet 长时高负载，更不是物理 Cortex-M4 芯片绝对实时性担保；需要进一步相同负载多次独立 jitter 对比才可严格签收“降低控制周期抖动”。

## S5-010 复验补记（2026-11-01）：最新 CI 已恢复 3/3，历史根因仍待证实

- **同一代码提交**：上游基线 `feat/stage5-rtos-apf-closure-20261101` 的 `7c14a567aff3`，真实 [Stage5 GUI CI #41279595520](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41279595520) 中 `native-gui-no-drag-xvfb` 为 success。该任务运行原来的 `six_axis --no-hold`，实际 worker 日志报告物理段 `1/3`、`2/3`、`3/3`，而非只通过单元测试。
- **结论的边界**：CI 新成功证明这一提交的指定虚拟环境可完成三段，但 2026-10-31 发生的 0.24° 最终命令偏差并无独立、可重复的根因隔离实验；因此保留 [Issue #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34) 的历史证据和稳定性风险，不应因一次绿灯宣称问题彻底根治。
- **防止假阳性**：本修复分支在 `.github/workflows/stage5-live-gui.yml` 里追加独立的 `result.json` 检查，要求 `passed=true`、真实 `3/3`、时钟对齐、每段物理收敛记录存在、无坏 ACK/checksum、故障计数为零。只看子进程退出码或 GUI 动画不再是这项门禁的唯一依据。
- **文档版本**：2026-10-31 旧版“固件内 APF 尚未运行”的状态仅作历史快照；当前固件内 C APF/IK、候选返回与 PyBullet 外部门禁应以 2026-11-01 的 [19 号审查](19_stage5_original_requirements_evidence_review_2026-11-01.md) 和实测结果为准。


## S5-015（2026-11-01）：5.3 CPU 与抖动取样窗口错位、低负载重复试验

- **发现**：原性能工作流中，默认 `ROBOT_STAGE4_QEMU_HEALTHY_RUN_MS=1200`，Timer0 抖动在健康阶段结束处冻结，但 CPU 快照由 Monitor 每100ms刷新直至含后续故障/多项式测试后停机，导致过去约4.7秒 CPU 时间与约1.2秒 Timer0 样本不在同一工作阶段。两者不能直接作为“同一监测窗口”性能指标。
- **修复**：仅在 CPU Trace 自测构建下，令 `stage4_rtos_selftest.c` 在 `stage4_timing_capture` 之后、故障注入之前捕获 CPU；关闭 Monitor 每100ms的额外自测快照；两套 QEMU ELF 均显式运行5000ms健康阶段。新增六组交错 `AB/BA` 测量脚本、成对差值汇总和缺样本/错误频率/溢出/回归负例。**不更改生产 PID、任务周期、速度或碰撞安全门禁**。各任务 Timer0 仍只保留最后128条，不可把不同跨度统计写成完全同窗。
- **失败证据**：[首次 CI #41308263682](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308263682) 比较器拒绝仅约1.23s健康阶段，未删除失败记录；真实固件 C 构建、GDB采集均通过。
- **同提交成功证据**：[六组性能 CI #41308538374](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308538374)，[原始工件](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41308538374/artifacts/13084018842)。Monitor CPU 6/6组下降（中位数0.05%→0.02%）；Control/Trajectory P99 抖动各5/6组下降但有1/6组退化；Monitor jitter 5/6组退化。新同提交 ARM map FLASH106464→88600B（−17864B）、RAM33936B不变。
- **状态**：**测量方法已修复、指定短时 QEMU 性能优化与 A/B 已有可重现证据，但“所有任务稳态抖动降低/硬件 WCET”不可签收**。详见 [5.3 复验专项报告](20_stage5_5_3_repeated_performance_evidence_2026-11-01.md)。下轮如需进一步闭环，应设置完整动态 UART/PyBullet 对照负载或独立硬件实验，不能为通过而盲调中断优先级或延长门限。
