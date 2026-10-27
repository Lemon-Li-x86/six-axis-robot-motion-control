# 第五阶段集成问题追踪台账

创建于 2026-10-27。用于登记现象、证据、根因、修改提交和复测。**未复现或未查清的根因不得编造。** 严重级别用于安排工作，并不代表已出现实物事故。

| ID | 优先级 | 当前现象 / 缺口 | 状态 | 证据 / 下一步 |
|---|---|---|---|---|
| S5-001 | 实机专用（本项目N/A） | 缺实体电机独立急停、失联制动和安全联锁设计；现有Control故障仅阻止继续下发新目标 | **纯仿真交付不阻塞**；禁止据此实机动作 | `docs/part4/05_global_architecture_review_2026-10-26.md`；仅允许非驱动模拟 |
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


## 2026-10-27 待定位问题 S5-012：原生拖拽大扰动后无法回位

**优先级**：P1（仿真完整闭环）。**状态**：现象已在 Windows 桌面观察，根因待查，不得标记为已修复。

- **复现条件**：`fix/stage5-native-pybullet-interaction-20261027` 原生 `p.GUI` 完成三段请求后进入 HOLD，用 PyBullet 原生鼠标拾取将连杆大幅拉偏。
- **用户提供的最后现场**：执行角/实测角 `J1=12.55/12.50°, J2=-32.46/-32.43°, J3=75.35/75.29°`，而 `six_axis` 最终请求目标 `[30,-45,60,20,5,-10]°`；`RX=2851, ACK=2187, age=0.297s, checksum_bad=0`；父进程报 `Physics worker exited 1`。**当前尚未获得本轮 worker.log 真正的子进程异常行**，不能将其直接归因于 PID 或 `last_fault`。
- **不等价的证据**：此前另一轮报告 `Diagnostics timeout: last_fault`，只能证明该字段的诊断响应未及时到达，并不表示 C 控制器返回了 `last_fault` 错误码。QEMU 反馈 ACK 计数落后目标帧数是异常线索，但不能未经协议追踪计算“丢包数”。
- **已经提供的取证功能**：仅仿真选项 `-ObserveHold` 跳过自动重复重规划，让已有 C PID 在原参考角下持续运行；`-DiagnosticsTimeoutSeconds 30` 对单条诊断查询设置有限延长等待。UART 失联、QEMU 停机、命令 ACK 和真实故障仍保持硬门禁；未恢复仍写 `passed=false`，不宣称回位完成。
- **下一步必须验证**：原始 `worker.log` 的末尾异常，`uart_live.jsonl` 的 `hold_deviation`/`hold_observe_stalled`/`hold_replan`/`diagnostics_timeout`，`result.json` 的 `controller_faults`/`feedback_stale_count`，以及在鼠标完全释放、无新增运动请求时 `joint_trace.csv` 距最终请求目标的误差趋势。验证物理拾取是否保持约束，不能通过增加超时掩盖控制停用。


## 2026-10-27 新证据 S5-013：第二段运行时 UART 诊断超时与反馈积压

**级别**：P1。**状态**：根因待定位，尚未修复。

- **本机真实复现**：`-ObserveHold -DiagnosticsTimeoutSeconds 30`，第一段 `[SETTLED]`，第二段 `[ACCEPTED]` 后始终未完成。工作进程最终报 `Diagnostics timeout: controller_steps; UART targets=7900, ACK=5275`，墙钟等待 `33.937s`，`segments_settled=1`，绝未进入 `HOLD`。
- **最后成功获取的不同时间点计数**：`requests_accepted=2`、`trajectory_samples=2865`、`controller_steps=444`、`controller_faults=0`、`planner_faults=0`、`feedback_stale_count=0`、`last_fault=0`、`monitoring_ticks=260`。这些都是诊断**历史快照**，不是超时发生时的控制器实时状态。
- **关键区别**：`7900 * 0.01=79s` 是 Python 依赖 **UART 目标帧数量**推进的 PyBullet 仿真时间，而不是独立的 FreeRTOS 时钟证据；`7900-5275=2625` 是目标帧数与反馈 ACK 数的计数差，**不能直接命名为丢帧**。
- **架构风险点**：`stage5_qemu_uart_visual_demo.py` 每收到一个 UART 目标帧，在同一个阻塞轮询中运行 10 次 PyBullet 步进、生成反馈再读取下一批串口数据；原生 GUI 的渲染/鼠标交互可能放大积压。C 的 `protocol_commands.c` 通过 `uart_tx_manager_send_frame()` 提交诊断响应，但忽略返回状态；缓冲区满时可能静默丢失诊断响应。此两点目前仅为**源码级候选原因**。
- **只加取证，不改控制**：在 Python 复用已存在的 C 协议 `DIAGNOSTICS_METRIC_RX_DROP_COUNT=0`，以 `uart_rx_dropped_bytes` 查询 UART RX 溢出，并在 `uart_live.jsonl` 输出 `uart_batch_delay` 事件（帧数、处理墙钟耗时和 ACK 滞后），不降低 UART 频率、不丢弃目标帧、不掩盖故障、不修改 FreeRTOS/PID/URDF。补丁提交 `612b02fa`、`8ad479fc`。
- **下一步验证**：如 RX 丢字节快速增加，检查控制器调度/输入环形缓冲区；如处理批次耗时长且 ACK 滞后增加，检查 GUI 同步物理步进造成的 UART 积压；如两者正常却诊断丢失，再审查 TX 缓冲与诊断回包发送返回值。不能简单增大超时宣称已修复。


### 2026-10-27 S5-013 第二轮日志与定向优化（待 Windows 验收）

- **无需拖拽即可复现**：Windows `p.GUI --opengl2 --no-hold`，`segments_settled=0`，在 `monitoring_ticks` 查询超时退出；最后 `target_frames=730`, `feedback_acks=382`、`uart_rx_dropped_bytes=0`（历史查询）。
- **实际积压实证**：同一 `uart_live.jsonl` 中 `uart_batch_delay` 由 26帧/0.172s -> 73帧/0.750s -> 109帧/1.563s -> 236帧/3.953s -> **481帧/7.172s**。接收与GUI物理推进在同一线程串行，吞吐低于固件持续输出时，处理大批次阻塞诊断超时检查。此处是可观测的同步批处理瓶颈，不等价于已经证明某条UART帧丢失。
- **关键对照**：同机 Windows `--headless-observer --no-hold` 获得 `passed=true`，`segments_settled=3`，末端误差最高0.07°，`target_frames=640`, `feedback_acks=641`, `uart_rx_dropped_bytes=0`, 三类固件故障均0。问题高度关联GUI路径，而非基本C运动算法。
- **定向优化 `c1ff3e83`**：保留权威的 `FEEDBACK_STEP_S=0.01s`、`PHYSICS_STEP_S=0.001s`。GUI通过Bullet原生 `setPhysicsEngineParameter(numSubSteps=10)` 使每个UART帧**仅调用一次** `stepSimulation()`，而该调用内部仍积分10个1ms子步；原来的10次跨GUI线程命令成为1次。DIRECT原有10次1ms步进不变。启动预热总物理时间同为0.1秒。调试轨迹显示已独立节流到10Hz。
- **回归 `602a4b02`**：采用Mock检查两种模式调用步长、子步和启动预热步数。**这项Mock测试不等同于Windows GUI端到端性能通过。**
- **后续验收**：先执行仅GUI无鼠标的三段闭环并检查 `uart_batch_delay` 是否消失、`target_frames-feedback_acks` 是否不再持续增长、`result.json.passed`。不合格时再排查原生OpenGL2、UART实时协调，不可宣称拖拽恢复通过。


### 2026-10-28 S5-013 第三轮数据与反馈信用流控（端到端验证中）

- **GUI 原生子步优化仍不足**：Windows p.GUI 未拖拽，30秒限时 `segments_settled=1/3`，`target_frames=2847`, `feedback_acks=2584`, `uart_rx_dropped_bytes=0`（历史快照）；批量处理由241帧/1.359s渐增到**481帧/2.781s**，说明虽降低了单帧 PyBullet 成本，输入仍持续超过可消费吞吐。用户另观察到明显、较大幅度非指令运动，**该 GUI 模式不具备安全验收资格**。
- **结构性原因**：仿真固件 `ProtocolTX` 每10ms独立发送新的命令，即使 Python 同一线程尚在调用 GUI 物理引擎、无法及时返回真实测量值；未设反馈信用会累积过时目标，从而使物理演算逐帧追赶过去而非实时执行 C 控制。旧版 GUI 单纯调整超时/轨迹绘制频率/子步批处理均未阻止串口命令债务。
- **最小修复 `9a4bca1c`**：仅当编译宏 `ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION` 开启时，`task_protocol_tx()` 才按现有 `motor_state_observation_t.feedback.sample_sequence` 的真实被接受反馈变化发给 GUI **最多一个**新 UART 目标帧；没有新反馈时仍保持10ms调度，但不重复发目标；如完整 UART 目标帧无法入 TX Ring Buffer，不消耗该次信用，下一周期重试。无需新增全局状态、协议命令、PID或限位常量；真实固件路径原样。**这不代表无限等待或禁用安全门禁。**
- **自动压测 `19ae5318`、`5e5b5296`**：在 DIRECT 下主动模拟每帧18ms额外工作耗时，保持 PyBullet 真实10个1ms物理步进和实际 QEMU/FreeRTOS/UART反馈，用于测试生产者不会因慢消费者而积攒历史命令。Windows/Linux workflow 在原有 3段 DIRECT 验收后再运行压测并保留产物。需确认真实 CI 结果；与 `p.GUI --opengl2` Windows 鼠标交互不可划等号。
- **仍待验证**：校验新固件 DIRECT 基准及慢消费者压测，之后才应由用户本机执行单次 **GUI 无拖拽** 复验；大扰动抓取与回位必须单独验收，不能通过增加 GUI 超时宣称问题解决。旧版 `0.07°` 终点误差仅为修改流控前的 DIRECT 基线。


#### 2026-10-28 验证进度更新
- <https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40538029092> Linux CI：C固件编译、三段 DIRECT 闭环、`--headless-step-delay-ms 18` 慢消费端压力测试均 **success**；UART流控至少在此环境没有破坏正常闭环，且压力条件下可以完结。
- 同一 Linux CI 中原有三次 PyBullet 外部转矩注入/回位断言为 **failure**。历史记录证明该测试的外力未产生预期位移（`peak_displacement_deg` 过小），不可将它视为大幅拖拽回位验收。完整 workflow 结论因此仍为 failure，不隐瞒。
- 同轮 Windows CI 安装/运行步骤尚未得到最终结果；Windows 原生 `p.GUI` 鼠标拾取、大扰动、实际回位不属于 CI DIRECT 验证范围，必须另行确认。


### 2026-10-28 S5-013 时间同步缺陷：模拟已到位不等于闭环动态合格

- **新本机现象**：Windows GUI 无拖拽仍大幅逐渐摆动，流控版无法稳定完成三段。暂停 Windows GUI 验收，禁止将其标记为已修复。
- **核验的原始 Linux native-GUI 产物**：Actions [#40539424716](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40539424716) 的 `stage5-gui-nodrag-xvfb` 附件 `result.json` 显示 `passed=true`、`segments_settled=3`、196帧×10ms=**1.96s PyBullet物理时间**，而 `trajectory_samples=362`×20ms=**7.24s 固件轨迹任务名义运行时间**，比值 **3.69**；真实墙钟 8.36s。终点误差 0.06° 不能掩盖时间失配。该CI绿色是**错误验收标准下的假阳性**。
- **流控自身的回归**：Actions [#40538029092](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40538029092) Linux `result.json` 中原有DIRECT亦仅196帧/1.96s，而 `trajectory_samples=388`×20ms=7.76s（时间比值3.96）；慢消费者DIRECT 196帧/1.96s、`trajectory_samples=380`×20ms=7.60s（3.88）。因此 `9a4bca1c` 的反馈信用流控**阻止目标队列膨胀的同时破坏时间对应关系**，并非可交付修复。
- **撤回流控**：`72b03aee` 在唯一生产 `firmware/freertos_demo/application/protocol_tasks.c` 中精确恢复流控前源码，不改变PID与URDF。
- **验收加固**：`c38a971c`、`96453585` 添加每次 C `trajectory_samples` × JSON唯一参数源中的20ms 与 PyBullet物理时间比值，默认差异超过25%就记录 `clock_alignment_failed` 和 `passed=false`；这一量只是任务样本数对应的**名义时间估计**，并非高精度硬件计时，但足以揭示上面的3～4倍明显错位。旧版 DIRECT 用户实测 319样本×20ms=6.38s 与6.40s物理时间匹配。
- **当前结论**：最强实证是**GUI/QEMU异步时间轴不对齐导致闭环动态不能被现有终点误差证明**；Windows大幅振荡的每关节时域根因仍须用用户失败的`joint_trace.csv`查证。解决方向需让QEMU虚拟控制时钟与实际物理步进一致、或明确拆分可验证的DIRECT控制物理与异步GUI交互；不能再用目标帧背压或视觉帧节流代替正确的同步设计。
