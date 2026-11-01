# 第五阶段：原始任务书 5.1–5.4 严格验收及性能证据（2026-11-01）

> **状态**：软件仿真集成任务已扩展到 **ARM Cortex-M4 FreeRTOS 内的 C APF/FK/IK 实际规划请求**、PyBullet UR5 全连杆障碍安全门禁及 C/UART 反馈执行。此页是按原 PDF 四项交付逐一核证的审查，**不是实体工业机器人的安全认证**。任何未达到原 PDF 文字要求的条目均须保留差距，不可用 GUI 按钮“PASS”代替。

## 0. 唯一版本与证据原则

开发分支：`feat/stage5-rtos-apf-closure-20261101`，建立自 Stage5 GUI 修复 PR #35 的 `692c529d6ca4`，原 `main` 和 Stage4 冻结基线未更改。所有固件功能均由 CMake 宏在 QEMU-only 固件上显式启用；真实电机输出、独立于仿真机的运动授权仍需额外设计。

统一配置来源：`firmware/freertos_demo/config/robot_defaults.json`；生成器：`tools/generate_firmware_defaults.py`；固件编译期生成 `firmware_defaults_generated.h`；不能在任务源码中重新复制 PID、任务周期、栈水位扫描间隔等配置。

证明等级依次区分：①源码存在；②Host C 单测；③ARM QEMU FreeRTOS 运行并取得 UART 回执；④PyBullet 物理反馈、逐连杆碰撞检查及最终到位；⑤用户 Windows 本机鼠标验收；⑥实体工业机器人认证（**不属于当前项目**）。

## 1. 5.1 完成全模块系统集成

```text
PyBullet / UART 指令接收
    ↓  CRC/checksum + 半帧失效重同步 + 帧长度检查
ARM ProtocolRX 任务（仅入队）
    ↓  FreeRTOS request_queue（单槽，覆盖前命令）
ARM Trajectory 任务（真 C APF、FK/IK、限位、梯形规划）
    ↓  经过外部真实 UR5 全连杆校验的单独运动命令
ARM Trajectory 任务（C 轨迹采样） → reference_queue
    ↓
ARM Control 任务（C 增量式 PID + 速度/加速度 limiter）
    ↓
QEMU UART 目标角 → PyBullet 真实虚拟物理 → UART 六轴反馈
    ↺
Monitor 任务 / GDB / .map / 丢帧 / 故障日志
```

- 驱动层：UART Driver、Timer、MotorState；通信层：Parser、Ring Buffer、任务级带 Mutex 的 UART TX；模型层：UR5 DH/URDF、角度规范与连续位姿；算法层：C FK/IK、APF、梯形速度、多项式、PID、运动限幅；FreeRTOS 层：ProtocolRX、Trajectory、Control、Monitor。
- 必须区分 **已收到 UART 请求 ACK** 与 **真实物理完成**。`MotionSequencer` 只有关节误差连续满足 0.15°、反馈鲜活及全连杆无违例才会报到位。重复用户按钮无需会话次数硬上限；每一次动作需独立安全授权及真实完成回位。
- QEMU/PyBullet 运行的 UART 闭环已在 [Stage5 集成 CI](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions) 反复验证，包括最初 4/4 被障碍阻挡路线、J6 连续五次点击、逐段 ACK、断包恢复及错误计数。
- 代码变更通过 Git 分支/PR 分离、Issue Log 追踪，避免将 Stage4 PID 参数与本阶段示范逻辑混入其它版本。

## 2. 5.2 长方体静态障碍、C APF 与固件路径规划

### 2.1 环境与安全架构

`simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json` 定义静态 AABB、目标、APF 增益、净距与离散碰撞采样预算；`blocked` 场景直线被遮挡。算法复用同一份 `potential_field.c` 和 `stage5_apf_joint_path.c`，避免主机 Python 重写数值 APF。

重要局限：APF 仅计算末端方向，**不保证肘、腕、前臂不撞箱体**。因此固件只负责提出候选关节目标，PyBullet 在收到 **ARM C APF+IK 的实际 UART 候选**后检查 URDF **所有连杆**，只有完整扫掠采样满足场景最小净距，才另外发出原有 `CMD_REQUEST_JOINT_MOVE`。整个链路任何错误均不批准动作。

### 2.2 固件真运行，而非编译符号冒充

| 接口 | 所属层 | 实际行为 |
|---|---|---|
| `CMD_REQUEST_APF_JOINT_PROPOSAL=0x05` | PyBullet → ARM UART | 通过真实关节反馈坐标发送单个静态 AABB、目标偏移、C APF 参数；请求无隐式电机输出 |
| `protocol_commands.c` | ProtocolRX | 只解析和原子入队，不在 256-word RX 栈上跑 IK |
| `stage5_apf_proposal.c` | ARM FreeRTOS Trajectory | 从同步获取的新鲜 MotorState 启动 C APF、FK、IK/最近分支选择、FK 残差、梯形速度约束检验 |
| `CMD_APF_JOINT_PROPOSAL_RESPONSE=0x88` | ARM UART → PyBullet | 返回生产 C 提出的 xyz mm、六轴 canonical centidegree、goal_reached、错误码；没有运动授权 |
| `stage5_collision_validation.py` | PyBullet 静态 AABB | URDF 模型全部连杆按约定关节步长采样，拒绝触碰/过近/预算不足，验证后恢复原姿态 |
| `CMD_REQUEST_JOINT_MOVE=0x03` | PyBullet → ARM | **只有全连杆检查通过后**才重新请求 C 梯形轨迹及 PID，等待真实物理反馈到位 |

**已通过真实 QEMU ARM C APF/IK 物理回路**：[Actions #41265049762](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41265049762)，工件 `stage5-embedded-arm-apf-verified-physical`。实际数据：ARM 产生 1 次 C APF/IK 候选；PyBullet 全连杆扫掠采样 5 次；测得最小净距 40.95mm；UART 单独运动请求 ACK 1 次，真实运动最大关节变化 4.38°，连续 15 帧到位，最终误差 0.07°，校验错误 0。**原 PDF“嵌入固件规划模块”从代码层已满足，且已实测单步规划—运动。**

**完整路线新增实测**：[Actions #41266022023](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41266022023)，工件 `stage5-embedded-arm-apf-verified-physical/stage5-embedded-apf-uart-full/result.json`。在带长方体的 `blocked` 场景中，从原点根据 JSON 指定的中间目标到最终目标，**ARM FreeRTOS C APF/IK 反复规划 28 次**，28 次都分别经过 PyBullet 全连杆离散预检、得到单独 UART 运动 ACK 并物理反馈到位，`route_complete=true`，最终末端目标误差 **0.7977mm**、最大六轴末段误差 **0.05°**、协议校验错误 **0**。此实测满足“固件 C APF + 静态 AABB 场景整路迭代调整与执行”的软件仿真要求。

**安全与自主性边界**：场景 JSON 指定 **via（高层途经目标）**，PyBullet 掌握外部全臂真实几何并批准每一步，因此不能写“固件可以完全独立发现任意障碍、决定全局绕行策略，也不依赖外部碰撞验证”。这仍是明确设计边界，而非固件 C APF 未运行。

## 3. 5.3 性能优化：CPU、RAM/FLASH、优先级与抖动

### 3.1 代码级优化及单一参数来源

- `Monitor` 每 100ms 运行。原来每周期调用三次 `uxTaskGetStackHighWaterMark` 扫描任务栈；新 JSON `stage5_monitor.stack_watermark_sample_period_ms=1000` 使栈扫描改为每 1000ms 一次，同时仍每 100ms 更新 `monitoring_ticks`。生成器校验它必须是 Monitor 周期整数倍，拒绝非法配置。优化可以独立关闭，Stage4 frozen self-test 默认 OFF。
- 仅 Stage5 ARM 目标启用链接器 section 级 GC：`-ffunction-sections -fdata-sections -Wl,--gc-sections`；支持开关，实现同条件 before/after，而不是随意删除功能源码。构建期只剔除不可达静态代码/全局数据。
- 原先重 APF/IK 不在运行时固件规划路径；现在高成本 IK 放在低优先级 `Trajectory` 任务而不是 `ProtocolRX`，避免堵塞接收和影响高优先级 Control。
- 控制优先级 4、ProtocolRX 3、Trajectory 2、Monitor 1（来源 JSON）；Cortex-M NVIC 8 priority bits、FreeRTOS syscall ISR 阈值 0x80，UART RX/TX、Timer1 ISR 均 0x80。均满足 FreeRTOS `FromISR` 调用约束，**无单独证据证明重新分配 IRQ 数值会降低周期抖动，不盲调 IRQ**。已用 `motion_task_wait_next_period` 丢弃逾期任务激活，避免赶时钟出现高优先级忙循环。

### 3.2 同提交 QEMU/GDB CPU 和 Timer0 抖动 A/B

[Actions #41265306540](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41265306540)，工件 `stage5-freeRTOS-cpu-trace` 中保留原始 GDB 采样 `observed.json`、`observed-timing.json` 及完整 A/B。相同运行时间约 4.72s，二者切换 `ROBOT_ENABLE_STAGE5_MONITOR_STACK_OPT`，未调整任务周期、PID 或控制阈值。

| FreeRTOS 任务 CPU | 原 100ms 扫描 | 优化 1000ms 扫描 |
|---|---:|---:|
| Monitor | 0.07% | **0.04%** |
| Control | 1.09% | 1.06% |
| Trajectory | 0.19% | 0.18% |

| 周期抖动 P99 | 基线 | 优化 |
|---|---:|---:|
| Control | 0.01328ms | **0.18532ms（退化）** |
| Trajectory | 0.03136ms | **0.18788ms（退化）** |
| Monitor | 0.02184ms | **0.13488ms（退化）** |

另一次同配置短窗口 [Actions #41265249322](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41265249322) Control P99 `0.09428→0.01808ms`（改善）而 Trajectory P99 `0.02160→0.02932ms`（小幅退化）；方向不一致。**严禁只摘录有利的一次结果说“优化降低了控制周期抖动”**。这是 4–5s QEMU 虚拟低负载窗口；要可靠完成 PDF 这句，还需同一真实负载、不同 seed、多轮可重复控制抖动 A/B 和更长运行时间；此处暂不符合无保留 WCET/稳定性声明。

### 3.3 ARM GNU ld .map 同条件 A/B

相同提交的 Stage5 完整 ARM UART RTOS 配置，关闭/开启 `ROBOT_ENABLE_STAGE5_LINK_GC`：

| 静态内存 | 关闭裁剪 | 开启裁剪 | 差值 |
|---|---:|---:|---:|
| FLASH | 105,256B | **87,384B** | **−17,872B（约−17.0%）** |
| RAM `.data+.bss` | 33,796B | 33,796B | **0B** |

这是真实 ARM .map 输出，不等于运行时堆峰值。**不能凭 FLASH 节省宣称 RAM 节省，更不能从“堆未溢出”推断所有路径都无栈溢出**。原始 map 与 JSON 同上 Actions 工件保留。

## 4. 5.4 GDB+QEMU 串口全链路调试和问题排查

GDB 在 QEMU ARM 固件读取 FreeRTOS task handle、任务 CPU 占比、Timer0 控制/轨迹/Monitor 的周期 P99 及峰值、堆栈高水位、任务故障计数；UART FrameStream、Ring Buffer、CRC/checksum、半帧超时重同步均有实际 QEMU/Host/C 单测。此处 GDB 证据为任务快照、计时与实际断点检查，不冒充穷举所有时序和硬件 ISR 故障。

故障与修复登记：
- S5-009：半包/丢一个字节后 Parser 错误恢复；C Host/QEMU 字节流故障注入。
- Issue #22：Feedback tick/读取时序竞态造成 `NOT_READY(-7)`，改同步读取后复验多段轨迹。
- S5-011：Windows J6 +2° 合法点击被原 HOLD +0.02° 余差误算成越过任意 IK 跳变阈值，限制在真实单次授权+全连杆安全上下文内。
- S5-012：GUI 成功次数上限（3）造成第 4/5 次任意按钮 REFUSED 且原报告仍 PASS；删除用户寿命次数限制、两层 PASS 校验 fail-closed、原生 GUI 五次真实闭环 CI。
- S5-013：主机侧 C APF 编排并未等于固件内规划，新增 FreeRTOS 请求队列+ARM C FK/IK+UART candidate+Bullet 安全批准。
- S5-014：逐周期扫描整段栈造成性能开销及 map 未裁剪，添加同配置 A/B，保留抖动退化和 QEMU 波动事实。

根因与修复细节见 [项目 Stage5 问题追踪](02_integration_issue_log.md)。

## 5. Windows 本机一次性仿真验收命令

在 PowerShell 打开的正确仓库中，**先运行 `git status --short` 确认本地无未保存修改**。保留原分支，可在独立 worktree 检查本阶段：

```powershell
git fetch origin
git worktree add .stage5-final origin/feat/stage5-rtos-apf-closure-20261101
cd .stage5-final
git rev-parse --short HEAD
```

**（A）非交互式固件内 C APF：完整静态长方体绕行 28 段；QEMU ARM/FreeRTOS、逐段 PyBullet 全连杆门禁、真实 C/UART 和物理到位：**

```powershell
powershell -ExecutionPolicy Bypass -File .\\tools\\run_stage5_embedded_apf_windows.ps1 `
  -Python python `
  -Qemu qemu-system-arm `
  -TimeoutSeconds 300 `
  -OutputDir build/stage5-windows-embedded-apf
```

要求 `result.json` 的 `passed=true`、`route_complete=true`、`joint_motion_acks=new_apf_proposals`，全部 `approved_legs[].settled=true`，`uart_checksum_errors=0`，最终 `final_cartesian_error_mm` 小于场景 C APF tolerance + 1mm（数值上真实 CI 为 0.80mm）。完整 PASS 自动打包 `build/stage5-windows-embedded-apf.zip`。

**（B）交互式 Windows GUI：等待旧静态场景 4/4 到位，然后 J6 +0.5° 回位→+2° 回位，再继续至少三次合法按钮并分别等待 RETURN COMPLETE，最后点击 FINISH。**

```powershell
powershell -ExecutionPolicy Bypass -File .\\tools\\run_stage5_live_windows.ps1 `
  -Python python `
  -Qemu qemu-system-arm `
  -Stage5RouteCase blocked `
  -SafeJog -RequireTwoStepJog `
  -SafeJogHoldSeconds 600 `
  -OutputDir build/stage5-windows-final-2step
```

此 GUI 路径按设计仍使用主机同源 C APF 整条路，**不能拿 GUI 路径的结果替换（A）真实 ARM 内 APF 的工件**；（A）也不能代替（B）的真人鼠标交互验证。所有失败日志在 `build/` 下，不删除或用“跳过碰撞检查”制造 PASS。切记本仓库是软件仿真，**这些命令不能直接驱动实体机器人的伺服电机**。

## 5. 严格签收表和最终交付

| 原 PDF 核心要求 | 当前已完成并有证据 | 无保留签收所缺条件 |
|---|---|---|
| **5.1 全层完整控制闭环** | QEMU ARM/C FreeRTOS UART→C规划/IK→PID→Bullet物理反馈、断帧/超时与重复按钮 CI | 最后 Windows 手动完整会话仍需亲自做一次 |
| **5.2 固件内 C APF，静态长方体避障** | FreeRTOS C APF/IK 真正反复运行 28 段、PyBullet 全连杆安全门禁逐段批准、真实 UART C PID 28/28 到位，最终 EE 0.80mm；场景定义高层 via | **软件仿真整路验收通过**；“无外部几何检查、无途经点、任意障碍均可自主绕行”不在本项目证明范围 |
| **5.3 性能优化** | FreeRTOS task CPU A/B、降低 Monitor 栈扫描 CPU；FLASH −17,872B，RAM 0；IRQ/调度优先级经审查 | 目前 QEMU 控制周期 jitter 优化结果不一致，不能把“降低控制抖动”当成已证明 |
| **5.4 排障与追踪** | GDB/FreeRTOS、Timer0、QEMU UART 错帧/丢帧/超时、Issue Log 和修复 CI | 真正实体设备异常注入并不在本项目软件仿真范围；最终同版本全工作流及 Windows 操作记录待封版 |

交付源码在本分支；真实 CI 文件 `result.json`、`observed*.json`、`stage5_firmware.map` 由各 Actions 工作流工件保存，不得用此文件文字取代原始证据。原始 Python 用户验收入口为 [Windows 连续按钮验收](18_stage5_windows_final_two_step_acceptance.md)；脚本 `tools/run_stage5_live_windows.ps1` 接收 `-SafeJog -RequireTwoStepJog`，必须至少连续五次合法按钮，各自完整 C/UART 回位，未发生拒绝、碰撞与反馈过期，才能由操作员验收。

**当前结论：ARM FreeRTOS C APF 完成了给定长方体场景 **28/28 段**整路规划/执行，且每段有外部全连杆门禁；CPU 与 FLASH 优化已取得实测，RAM 不变。唯独“控制周期 jitter 在可重复相同负载下确实降低”目前数据方向不一致，不能无保留宣称全部实时性指标优化完成。软件仿真与实体设备必须分开。**
