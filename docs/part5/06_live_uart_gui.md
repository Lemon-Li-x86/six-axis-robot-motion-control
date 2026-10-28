# Stage5: Windows 实时 QEMU / FreeRTOS / UART / PyBullet

在仓库根目录 PowerShell 运行：

```powershell
./tools/run_stage5_live_windows.ps1
```

入口使用 `CMakePresets.json` 的 `stage5-live-uart` 配置构建真实 ARM ELF。
依赖本机 CMake、Ninja、arm-none-eabi-gcc、qemu-system-arm，以及已安装
PyBullet 的 `.venv/Scripts/python.exe`。可用 `-Python`、`-Qemu` 指定路径。
使用 `-NoHold` 可在结束后自动关闭窗口。

运行时有两个窗口：

1. PowerShell 显示每轴 `目标角/当前角`（度），以及 UART 目标 RX、反馈 ACK、
   实际绘制数、显示丢帧数、校验错误和最新样本年龄。
2. PyBullet GUI **就是实际物理仿真世界**：原生鼠标拾取、关节电机和实际测量在同一个客户端。Headless CI 使用相同仿真入口的 DIRECT 模式。

目标角来自 C 固件发出的 UART 帧，当前角来自 PyBullet 实测；不从 CSV 回放。
C 固件继续负责轨迹、PID、限幅和 FreeRTOS 调度，没有修改 PID 或机器人配置。
机器人默认配置仍由 `firmware/freertos_demo/config/robot_defaults.json` 生成，
Stage5 构建开关集中在 CMake preset，入口和本 CI 使用同一 preset。

## Windows 停滞审查

2026-10-27 本机对照记录在 `build/stage5-review-*`（构建输出不提交）：

- 旧版 GUI 到第 1 段后 35 秒超时；CSV 最后目标/实测均约为
  J1=38.63°，请求终点是 38°。不是 UART 完全停发。
- 更换 IPC 后，无 GUI 的另一轮仍在第 1 段停到 65 秒；UART 目标和
  ACK 约 80 帧/秒持续增长，关节固定在 J1=37.75°。
- socket 就绪等待、TCP_NODELAY 单独使用时，GUI 仍可停在 J1=36.19°；
  运行中固件 controller_steps / trajectory_samples 持续增长，
  controller_faults、planner_faults、feedback_stale_count 都是 0。

因此，实测问题不只是窗口没刷新：Windows 主机时序下，QEMU 墙钟驱动的
RTOS 控制采样和逐 UART 目标推进的 10ms 物理反馈不同步，会出现停在
非请求终点的情况，而旧程序只在全部到位后查询诊断，表面一直停留在
`ACCEPTED`。这解释了本机的可观测故障；不能据此证明所有 Windows
机器上的唯一底层原因，也不能把无 GUI 的 Linux 通过当作 Windows 通过。

Windows 现在使用 QEMU `-icount shift=auto,sleep=on`，让真实 ARM 执行的
虚拟时钟自适应推进；Linux 保持原有墙钟后端。**不是此前尝试过的固定
shift 指令计数模式**。每个 UART 目标仍推进原有物理步数，再把测量帧
写回 QEMU UART。时序改变不等于硬件毫秒实时保证，日志分别记录墙钟时间
和物理模拟时间，`result.json` 标明 `qemu_clock`。

其余修复：

- IPC 使用单槽共享内存，读写锁只尝试一次；无管道 feeder、无无限排空
  循环。渲染慢或暂停只合并显示样本，控制进程继续运行。
- 控制进程的 Python 输出写 `worker.log`，QEMU stderr 写独立文件，
  不让 Windows 控制台选择暂停或未读取的 stderr 管道卡住控制。
- UART 非阻塞发送保存尚未写完的字节，短写/暂不可写后重试；64KiB
  上限超出明确失败，不能把部分 `sendall` 失败当完整成功。
- 无数据时等 socket 就绪，避免依赖 Windows 短 sleep 的调度粒度。
- GUI 关闭时通知 worker 正常清理 QEMU；worker 墙钟超时由父进程监督。
- 运行前移除旧结论；异常写 `passed=false`，避免复用旧 PASS 或旧轨迹。

## 证据文件

默认目录 `build/stage5-live-windows/`：

| 文件 | 用途 |
| --- | --- |
| `joint_trace.csv` | 每个目标的六轴 UART 目标/物理实测/误差，运行中持续写入 |
| `uart_live.jsonl` | 每秒墙钟/物理时间、RX/ACK，以及持续轮询的真实固件计数器 |
| `observer.jsonl` | 控制进程非阻塞遥测快照、目标/当前角、采样年龄、观察者跳帧；不伪报 GUI 绘制帧数 |
| `observer_result.json` | 遥测接收和观察者丢帧统计；GUI 由物理引擎自行渲染 |
| `worker.log` | 接受/到位事件、错误堆栈；不启动定时后台线程栈采样 |
| `qemu_stderr.log` | QEMU 本身的错误输出 |
| `run_metadata.json` | ELF、当前配置和 URDF 的 SHA256，Python/平台和时钟模式 |
| `result.json` | 真实闭环验收；只有全部到位且固件故障计数为 0 才通过 |

`observer_drop` 是非阻塞观察者未看到的遥测更新，不是 UART 丢帧；原生 PyBullet GUI 的实际渲染帧数不在这里计算。
UART 短写和暂不可写次数在 `result.json`；校验错误单独统计。
最终固件诊断重新采集完整一轮，不能用中途旧值冒充最终诊断。

## 验证范围

本机 3 次连续有图形运行完成 3/3 段，每次最大最终误差 0.07°；
之后通过 PowerShell 原生入口重新构建并完成 3/3 段。
各次结论有独立 `result.json` 和 `observer_result.json`。
后续修改的最终验证见同目录审查记录；不要用这些历史结果替代新提交 CI。

CI `.github/workflows/stage5-live-gui.yml` 在 Windows 和 Linux 上分别运行
IPC/协议单元测试及真实 ARM/QEMU/UART/DIRECT 闭环并上传日志。
CI 使用 `--headless-observer`，不证明 OpenGL 可见画面。
无头 CI 只检验物理反馈/协议/状态，不检验原生 GUI 的鼠标拾取；Windows 实际 GUI 需要单独手工验收。


## 2026-10-27 架构清理：恢复原生 PyBullet 交互（待桌面验收）

第五阶段曾将物理仿真放在 `DIRECT` worker，同时在父进程创建另一个
`GUI` 机器人镜像。旧版调用 `resetJointState()` 显示收到的测量角，
并禁用 `COV_ENABLE_MOUSE_PICKING`，因此不得不额外维护
`MouseDrag / DragMailbox / PhysicalDrag` 以及鼠标事件收发。
这是模拟器架构导致的人为复杂度，**不是 FreeRTOS/PID 必须识别鼠标松手**。

新分支 `fix/stage5-native-pybullet-interaction-20261027`：

- worker 仍为独立进程，负责 QEMU、UART 和唯一 PyBullet 物理世界；桌面
  使用 `p.GUI`，CI 使用 `p.DIRECT`，二者运行同一条固件闭环数据流。
- 原生 `COV_ENABLE_MOUSE_PICKING=1`。GUI 鼠标左键直接作用于真实
  PyBullet 物理世界，取消第二个显示机器人、伪造姿态、跨进程鼠标邮箱
  和自写 point-to-point 约束。
- 保留现有 C 轨迹规划、PID、速度/加速度限制、`0x01/0x81` UART
  反馈和固定的最终关节请求目标。控制从来不依赖 Python 的鼠标位姿。
- HOLD 时若真实反馈关节偏离最终请求位置超过 0.5°，记录
  `hold_deviation`；优先允许既有 PID 消除误差，若剩余误差长期停滞，
  则有界地通过 `0x03` 向 C 规划器重新提交**同一个目标**。
  不再把“松开鼠标”作为重新规划的必要条件。
- GUI 中的历史轨迹线改为有限 `lifeTime=10s`，避免连续 HOLD 无界增长。
- 父进程只接收最新遥测并输出控制台；`observer_drop` 属于可视化观察
  丢样，不反映 UART 断流。按 Q（GUI 焦点内）或 Ctrl+C 退出。

正常桌面命令：

```powershell
./tools/run_stage5_live_windows.ps1
```

无 GUI 物理扰动回归改用 PyBullet 自带的 `applyExternalTorque`，
而不是模拟鼠标点击和自己造约束：

```powershell
.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py --headless-observer --elf build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf --test-disturbance-recovery --disturbance-cycles 3 --hold-seconds 65 --no-hold --output-dir build/stage5-disturbance
```

这条 CI 测试只证明施加仿真外力、UART 状态反馈和目标恢复。**不能用
DIRECT 外力测试冒充桌面鼠标操作通过**。桌面仍需记录左键拖拽、释放、
归位误差、Q 关闭以及异常退出后的 `worker.log`、`joint_trace.csv`。
新分支代码写入不等于已取得 Windows GUI 验收结果。

### 历史证据（旧的双 PyBullet 世界实现）

此前使用鼠标事件共享内存和自写约束，记录在
`build/stage5-drag-rapid-mouse/` 等旧路径，历史数据不因本次修改而改变。
但这些记录**不能**当作原生 GUI 新架构的验收结论。旧版
`drag_events.jsonl`、`mouse_events.jsonl` 及相关自定义拖拽测试现已弃用。

## 实验性被动 HOLD：不提前判定回位失败

若原生 GUI 大幅拖拽后无法回到最终请求角，不能简单扩大 PID 增益或
关闭固件失联保护。先使用**被动观察模式**区分“现有 C PID 自行回位”
和“Python 因静止而反复下发新运动请求”。

```powershell
git fetch origin
git switch fix/stage5-native-pybullet-interaction-20261027
git pull --ff-only
.\tools\run_stage5_live_windows.ps1 -ObserveHold -DiagnosticsTimeoutSeconds 30
```

- `-ObserveHold` 仅对 *HOLD 期间的额外重规划器* 生效：关节偏离原始
  目标仍会记录 `hold_deviation`，出现长期几乎不动会记录一次
  `hold_observe_stalled`，但不会因该情况提交重复运动请求或抛出
  “回位不收敛”异常。真正的 C 控制器仍保持原目标、继续工作。
- `-DiagnosticsTimeoutSeconds 30` 将每条查询诊断的墙钟等待从默认
  5 秒延长为 30 秒，可在 5～60 秒之间配置，**没有无限等待**。
  QEMU退出、UART 8 秒完全断流、运动请求 ACK 超时、错误校验和
  固件真实故障不会因为这一开关被忽略。
- 正常的闭环验收仍按原标准。观察模式结束时，即使最终误差超标，
  `result.json` 仍保留 `passed=false` 和 `hold_final_error_deg`；
  在控制链路本身正常的情况下允许正常退出，以免把“实验观测完成”
  错报成“程序崩溃”。控制台会显示 `OBSERVED (NOT PASS)`。
- `-NoHold` 与 `-ObserveHold` 不兼容。观察完成后在 GUI 中按 Q
  或在 PowerShell Ctrl+C 退出，检查原始位置、固件故障计数和
  `hold_recovered` 是否出现。
- 若诊断查询最终仍超时，程序保存 `diagnostics_timeout` 事件和当时
  UART 目标/反馈 ACK 计数；这是通信/诊断通路问题，不能默默跳过。

重点区别：`target/actual` 是**固件电机命令/物理反馈**，不等于
“最终请求目标/物理反馈”。原始 `six_axis` 最终关节目标为
`[30,-45,60,20,5,-10]` 度。机械臂被拉开后，执行角贴着实测角
只说明低层电机跟踪，并不代表完成高层回位。

## Windows 原生退出 3221225477 / 0xC0000005

用户机器的 Python 3.11.15 曾在 python311.dll 中发生访问冲突，日志同时
显示周期线程栈采样被异常截断。独立的纯 Python 复现仅包含 Event 轮询
和 `faulthandler.dump_traceback_later(..., repeat=True)`，不加载 QEMU 或
PyBullet，也出现了同样的访问冲突。因此已移除后台定时栈采样，保留
`faulthandler.enable` 的崩溃诊断，以及在正常控制循环中写出的 UART JSONL。
不能把这样的定时 `Timeout (...)!` 当作运动控制超时。

异常退出时父进程另写 `worker_exit.json`，记录十进制/十六进制退出码和
最后显示样本；新运行前清除旧记录。更长时段的扰动回归用于覆盖以往
20/40 秒以后才会暴露的问题，早先十几秒的 CI 通过不覆盖这种原生崩溃。



## 2026-10-28 规范化角度与连续物理关节角（PR #3，待本机大拖拽复验）

用户 Windows 原生 GUI 在三段结束后拖出大圆：实际 J1 已到 260.48°，
随后 Python `struct.pack("<6h", ...)` 因超过 int16 抛异常，导致物理进程退出。
日志此前出现反馈 ACK 滞后，不能仅将此现象视作 Python 打包异常：
`protocol.c::decode_joint_payload` **只接受 [-180°,180°) 的规范化反馈**，
而原仿真把 PyBullet 连续角度直接通过 0x81 发送，实测超过 180° 已被 C 协议拒收。

**保持现有合同而非升级/复制协议：**

- `robot_types.h` 定义 UART 为 `int16` 规范化角度、Motor Driver 在 C 内部维护 `int32` 连续位置。PyBullet 真实连续角度仍不变；仅在 UART 0x81 边界按整圈等价关系规范化。例如真实 +350° 传 -10°，真实 +181° 传 -179°，C 的 `motor_driver.c` 按邻近采样恢复连续运动。不是强行截断到 327.67°，也不是 `resetJointState`。
- C 返回的规范化控制目标须结合 **URDF `getJointInfo` 实际上下界**，通常相对**上一帧连续执行器目标**逐轴解绕，不能每帧重新贴合用户鼠标拖动后的物理角度，也不能由某一轴的 ±180° 越界触发全六轴解释方式切换。只有固件新运动请求被 ACK 接受时才从真实物理姿态重置一次解绕锚点。未复制 URDF 关节行程参数或 PID。关节跟踪 CSV 区分 `command_*`（C UART 规范角）、`servo_target_*`（连续执行器目标）、`actual_*`（真实 PyBullet 连续角），`error_*` 仅在连续物理坐标下相减。
- **模型关节行程仍然有效**，并非任意连续无限圈。现有 URDF/C Model：J3 是 ±180°，其他为 ±360°；如果从 +30° 朝正方向拖完整 +360°，末端 +390° 已超出 J1 当前模型允许范围。GUI 现采用后述“交互硬限位”：采样前纠正同一 PyBullet 机器人真实越界关节、清除该轴速度并记录修正前位置；DIRECT 仍严禁静默越界。
- 对 **单个反馈周期内 ≥180° 的不可辨识跳变**报告 `ambiguous_multi_turn_feedback` 并安全停机。原 C Motor Driver 采用最短角差解绕，不可能仅凭两次规范化反馈重建这种跨半圈跳变。
- 对真正的错误格式保留 `JointFeedbackRangeError` 和详细 JSON / UART 审计，保留旧错误试验可核查；与支持合法多圈连续反馈不冲突。

**本机复验**：在本分支、默认无 `-NoHold` 模式下，先尝试 +170° 到 +190°、
+310° 到 +350°（在 J1 当前 URDF 行程内的方向），观察
`observer.jsonl` 中 ACK 是否持续增长、`joint_trace.csv` 的规范角与连续角是否同时合理。
再分别试图越过模型行程，确认应报明示的安全故障而不是让固件在越界姿态下盲控。
Windows 原生鼠标抓取仍必须由真人桌面验收；CI 的 DIRECT 自动外力不等于 GUI 拖拽。




## 2026-10-28 PyBullet 原生鼠标拾取的物理关节挡块（PR #3，待 Windows GUI 人工验收）

- 当前六轴 URDF 全部是 `revolute`，均有实际 `<limit lower/upper/effort>`，不允许为方便交互偷偷改成 `continuous` 或扩大范围。
- Bullet 默认已能从 URDF 加载 `btMultiBodyJointLimitConstraint`，但约束由数值求解。用户再次在 Windows 原生鼠标拖拽中观察到实际关节超过 +360°，**证明单靠 `jointLimitForce` 和 solver iteration 并不足够**。GUI 模式仍采用 `changeDynamics(...jointLowerLimit=URDF值,jointUpperLimit=URDF值,jointLimitForce=...)` 作为首层物理限位，但不再把它宣称为绝对挡块；限位来源仍为 `getJointInfo`，无第二套数值、无鼠标事件邮箱和第二套机器人。
- 约束强度系数和 solver 迭代属于**仿真数值夹具**，不对应实物机械强度或安全额定值。额外 0.10° 的 GUI 求解器穿透容差只决定是否触发重大越界故障，**不是允许的机械行程**；实际测量不裁剪，依然存 CSV 并按原 UART 协议规范化传输。
- 新增 `test_stage5_native_joint_stops.py`：在真实 PyBullet DIRECT 中只于初始化时设置 J1 接近上限的姿态，连续施加真实关节力矩，测量最大位置，再反向施力验证没有被冻结。此测试**不能证明 GUI 原生鼠标抓取已成功阻挡**，后者需要 Windows 人工反复拖拽观察并记录位置峰值与 ACK。
- 拖拽靠近上限应由 Bullet 物理约束阻挡，**不应仅靠 Python 检测越界后冻结进程**。若原生拾取仍能将关节推出 URDF 物理范围，收集 `result.json` 的 `fault_type` 和 `joint_trace.csv` 原始角度再修，绝不绕过固件安全门禁或伪造实际位置。



### Windows 重复大幅拖拽仍越界：补充 GUI 交互硬限位（2026-10-28）

真实用户再次观察到 J1 被原生 PyBullet mouse-picking 拖到 +360° 之外，
证明之前只加大 Bullet 约束力的版本 **未通过原生鼠标验收**。
CI 在 DIRECT 下施加固定电机力矩并未模拟桌面原生 mouse-picking 约束；
该测试只能证明物理 joint stop 在该力矩夹具下工作，不能外推。

现在仅在 **GUI 物理反馈路径** 每次 `stepSimulation()` 结束后调用
`enforce_gui_joint_hard_stops()`：
1. 读取 **同一个 PyBullet 物理机器人** 的六轴真实 `getJointStates()`；
2. 对超过同一 URDF `getJointInfo` 上下界的关节，以 `resetJointState` 
   把**真实物理状态本身**纠正到最近行程边缘，并将该轴速度置 0；
   六轴其他正常状态不修改。**这是有意识的 kinematic hard-stop
   workaround，不是 Bullet 物理碰撞反作用，也不是伪造额外 GUI 机器人**。
3. 修正发生在本轮 UART 连续角采样与下一帧规范化反馈之前；
   所以 C 反馈、GUI 视觉、CSV 轨迹取自纠正后的单一物理状态。
4. `uart_live.jsonl` 增加 `gui_urdf_hard_stop`（按墙钟最多每秒一个含实际
   越界角度的样本）；`result.json` 增加总修正次数、末次修正前角度与
   限位值。用户看到“撞限位”不等于纯物理验证通过。
5. 若发生非有限角度或单帧不可解绕的异常，仍按独立的严重故障处理，
   不应伪造反馈、吞异常或把限制方案默认为硬件安全认证。

局限：GUI 当前每条 UART 反馈执行一次带 10 个内部 1ms substeps
的 Bullet API 步进。因此边界保护只保证**每条反馈采样后的 GUI
物理状态**不越界，不能证明内部每一个 1ms substep 完全未穿透；
原生 GUI 渲染由独立线程驱动，极端鼠标强拉仍可能看见短暂视觉抖动。
若要求连单个内部 substep 都严格限位，需进一步改鼠标拾取交互机制
或在每个子步执行约束；不能再谎称“完全物理、绝不越界”。

手动验收：原生 GUI HOLD 拖拽 J1/J3 向上下限反复施力，
观察 `[JOINT STOP]`、UART RX/ACK 是否持续增长，放手能否回位；
停止运行后比较 `joint_trace.csv` 的最大实测角、
`uart_live.jsonl` 的修正前角和 `result.json` 计数。
此项尚未取得 Windows 桌面鼠标通过证据，PR 保持 draft。


### 2026-10-28 松手回位“先猛甩，再慢慢转”：执行器角度及扭矩纠正

用户原生 GUI 实测：拖拽到行程边界以后，松手时末端首先快速摆动，
然后才按照固件轨迹慢速回位。此前实现存在两个独立且可复现的风险：

- 旧 Python 代码根据 **全部关节是否处于 [-180°,180°)** 决定是否
  解绕六个目标，任一关节跨 ±180° 会同时切换全部六轴电机目标的解释。
  现删除该全局判断；每轴**逐帧相对上一条连续电机命令**解绕，
  而不跟随鼠标拖动的实测角度临时切换等效圈数。
- 固件在新请求 ACK 后会重置连续轨迹/限幅器，Python 只在这类
  **新请求 ACK** 后使用实测角度重建电机目标的绝对圈数，
  不在任意物理偏差期间擅自重基。
- 旧 GUI 六轴电机统一使用 `150 N·m`。
  现在从 PyBullet 已加载 URDF `getJointInfo()[10]` 获取六轴 effort
  （模型前三轴 150 N·m，腕部三轴 28 N·m），保持原生
  `POSITION_CONTROL`，不添加第二套 PID 或外部插值器。
  GUI 的物理挡块约束力是另一回事，不能充当电机额定输出。
- 单元测试覆盖独立轴跨 ±180°、其他轴被拖到 +350° 时目标不突然
  改变圈数，和固件 ACK 后在 +350° 附近重基、持续解绕；
  物理 GUI 原生拾取仍需 Windows 人工检验。
- 本次不调整 C PID、速度/加速度限制以及真实 URDF 行程；
  `joint_trace.csv` 的 `command_*`、`servo_target_*`、`actual_*`
  用来排查剩余“甩头”是目标跳变还是物理/鼠标约束瞬态。


### 2026-10-28 HOLD 366.76° 假误差、重复重规划及松手猛甩

Windows 手动拖拽再次复现：松手后先快速甩动，后长时间回位，
最后 `RuntimeError('Hold recovery did not converge: 366.76deg')`。
异常是 Python HOLD 逻辑抛出的，不代表物理关节损坏，
也**不是** PowerShell 启动器自身发生故障。

- 此前 HOLD 始终把 `sequencer.goals[-1]` 的规范化角度（例如
  +30°）当作**连续物理目标**；然而真正 C 规划器的
  `motion_limits_unwrap_nearest` 可从实测 -336.76° 选择合法等效目标
  -330°。两者在物理连续转数上差 360°；Python 计算的 366.76° 并非
  C 实际规划的回位误差。HOLD 现在在重新发出原规范化目标时，
  使用现有 `closest_physical_motor_targets` 与 URDF 上下界计算
  此次计划的连续目标，**只在 replan ACK 后**将其作为判定目标；
  原始规范化请求以及 C 规划器均不变。
- 旧 HOLD 仅凭 0.5s 无明显位移，就在一次未完成的长距离
  C 轨迹期间反复请求重规划，最多八次后主动崩溃。
  现在同一次偏离事件最多发送一次自动 C 重新规划请求，
  ACK 后持续观察同一 C 计划；若仍无法到位，则审计
  `hold_recovery_waiting` 且保留 GUI/UART，不将其报成 PASS。
  完成回位并再次受到新扰动，可开始新的回位事件。
- 以前的 PyBullet `POSITION_CONTROL` 仅用力矩限制，不控制
  可达到的执行器追赶速度。GUI 模式现在向官方原生
  `setJointMotorControlArray(maxVelocities=...)` 传每轴上限，
  取**C 固件默认配置 `stage4_motion.max_velocity_deg_s`**与
  已加载 URDF `getJointInfo()[11]` 速度上限的较小值。
  无第二套速度数值、无 Python PID/自主轨迹生成。
  这是执行器速度约束，**不限制鼠标原生拾取约束施加的外力运动**。
- 保留 DIRECT 原有物理验收路径；新增测试覆盖“+30° 与 -330°
  等效导致 366.76° 假误差”、正向 +350° 需回到 +30° 而不是越过
  +360°、速度限幅来源，以及真实 Bullet `maxVelocities` 接口。
- 用户 Windows 原生鼠标释放的实测验证仍然缺失；若继续甩动，
  应检查 `joint_trace.csv` 的连续 `servo_target_*`、
  `actual_*` 以及 `uart_live.jsonl` 的 `hold_replan` /
  `hold_replan_ack` / `hold_recovery_waiting`，不能把通过单元测试当成
  桌面交互已解决。PR #3 保持 draft。
