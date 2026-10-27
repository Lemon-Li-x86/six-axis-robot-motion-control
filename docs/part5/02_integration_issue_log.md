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
