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
2. PyBullet 原生 GUI 实时镜像 DIRECT 物理引擎的实测姿态和末端轨迹。

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
| `observer.jsonl` | 显示样本序号、目标/当前角、延迟、接收/绘制/丢帧 |
| `observer_result.json` | 本轮最终显示统计，headless 时 rendered=0 |
| `worker.log` | 接受/到位事件、错误堆栈；长时间运行另有线程栈快照 |
| `qemu_stderr.log` | QEMU 本身的错误输出 |
| `run_metadata.json` | ELF、当前配置和 URDF 的 SHA256，Python/平台和时钟模式 |
| `result.json` | 真实闭环验收；只有全部到位且固件故障计数为 0 才通过 |

`display_dropped` 是发布样本被覆盖或锁竞争丢弃的显示帧，不是 UART 丢帧。
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
GUI 的 render 调用计数也只证明提交了姿态更新，不能替代逐帧视觉验收。
