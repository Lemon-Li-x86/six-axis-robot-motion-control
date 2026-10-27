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
