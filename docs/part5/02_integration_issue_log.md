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
