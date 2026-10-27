# Stage5 QEMU UART ↔ FreeRTOS ↔ PyBullet 多段运动可视化

## 验证目标与真实边界

本演示直接运行 **Cortex-M4 / QEMU 中的实际 FreeRTOS 固件**：主机仅发送 `CMD_REQUEST_JOINT_MOVE (0x03)` 关节目标，固件执行 C 语言轨迹规划、PID、限幅与任务调度，经 UART0 TCP 输出 `CMD_SET_JOINT_TARGETS (0x01)`；PyBullet 的 `POSITION_CONTROL` 模拟执行器，按物理仿真反馈 `CMD_JOINT_STATE (0x81)`。

- 不使用 Python 轨迹或 PID 替代固件。
- 与自动验收入口 `stage5_qemu_uart_closed_loop.py` 分离；**不修改**该入口的默认目标、超时或安全判据。
- 支持连续三个关节运动段，只有收到固件成功入队 ACK 且 PyBullet 实际测量连续 `15` 个反馈帧误差不超过 `0.15°`，才提交下一段。避免一槽队列被提前覆盖。该 `0.15°` 仅是演示切段门限，不是固件的控制器配置。
- 演示使用仿真关节请求，不构成机械臂全连杆碰撞安全、力矩级控制或硬件可用性验证。
- **当前 UART 协议只暴露关节移动请求**，Cartesian 直线/圆弧的 C 内部规划接口尚无对应的串口提交命令，故不声称本演示已经验证末端直线或圆弧闭环。

## Windows PowerShell 启动

在仓库根目录执行（要求 CMake、Ninja、ARM GCC、QEMU、项目 `.venv` 和 PyBullet）：

```powershell
cmake -S . -B build/stage5-local-uart -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake' -DCMAKE_BUILD_TYPE=Debug -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON -DROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=ON -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF
cmake --build build/stage5-local-uart --target freertos_demo

.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_visual_demo.py `
    --elf build/stage5-local-uart/firmware/freertos_demo/freertos_demo.elf `
    --scenario six_axis `
    --output-dir build/stage5-visual-six-axis
```

脚本**自己启动 QEMU**，无需额外启动一个 QEMU 进程。默认打开 PyBullet GUI，增加每帧 `50ms` 的墙钟延迟，让六轴运动肉眼可见；运动结束后，按 Enter 才关闭 GUI。`--scenario joint_sweep` 只使 J2 进行大幅往返；`--frame-delay-ms 0` 取消额外延迟；`--no-hold` 结束时立即关闭；`--headless` 用 DIRECT 快速运行同一闭环。

## 输出与判据

输出目录包含：

- `result.json`：阶段请求数、到位数、UART 反馈/目标帧数、固件诊断计数器与 PASS/FAIL。
- `joint_trace.csv`：6轴目标/实际/误差（`deg`），实际末端坐标（`m`），仿真物理时钟（`s`）。
- `joint_tracking.png`：安装 `matplotlib` 时保存六轴目标/实际曲线；缺少该可选包时仍输出 CSV。

演示失败时抛出错误并退出非零状态（如运动请求拒绝、ACK 序号错误、无反馈、固件控制/规划故障、未到位超时）；发生异常也尽可能保留已经获取的 CSV。运行中自动生成的测试结果位于被 Git 忽略的 `build/`。

这里的 PASS 代表**指定多段关节请求在软件仿真里经完整 UART 回路实际到位**，不代表 Cartesian 算法、整臂安全性或长期稳定性已验收。UI 展示的末端蓝色线条为 **PyBullet 实测的轨迹**，不是笛卡尔规划的期望路线。

## 开发者检查

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p test_stage5_visual_demo.py -v
```

本测试覆盖切段状态机、ACK 拒绝、串口编码及 CSV 字段，不需要启动 QEMU 或图形界面；端到端是否成功仍必须在有 ARM/QEMU/PyBullet 环境的机器上运行。不要因为单元测试通过就标注 GUI/集成验收通过。

## Windows GUI 不动或第一段 ACK 后超时：分离录制和回放

原 GUI 入口在 UART 收发循环中执行 PyBullet OpenGL 渲染、画轨迹线和
sleep。Windows 图形负载可能拖慢 UART 反馈，表现为第1段请求
\`[ACCEPTED] Segment 1/3\` 后的 \`No UART frames from QEMU\`。
这属于需要本机确认的时序风险；仅凭旧日志不能断定 FreeRTOS 已停止。

**推荐入口**是 \`stage5_qemu_uart_trace_replay.py\`：
首先使用未修改的 QEMU + FreeRTOS + UART + PyBullet DIRECT 跑完整三段，
并记录真实物理仿真的实测关节角；校验 \`result.json\` 的 PASS 和实际位移后，
再打开 PyBullet GUI 按记录的物理时间回放。回放不是二次闭环，也不重新
计算 C 轨迹或 PID。可见动画不占用固件的反馈时序。

在 Windows PowerShell 的仓库根目录执行（保留 Stage5 测试专用 ELF）：

\`\`\`powershell
.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_trace_replay.py \`
  --elf build/stage5-local-uart/firmware/freertos_demo/freertos_demo.elf \`
  --scenario six_axis \`
  --speed 0.35 \`
  --repeats 2 \`
  --output-dir build/stage5-visual-replay
\`\`\`

这条命令**自行启动并结束 QEMU**，通过无窗口 PyBullet 完成录制，然后才
显示 GUI。终端中的 \`[MEASURED]\` 将打印六轴**实测活动范围**，而不是
请求角度。默认按录制的物理时间 0.35 倍速回放两遍，结束后按 Enter 关闭。

已有成功录制后，仅回放而不重新运行 QEMU：

\`\`\`powershell
.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_trace_replay.py \`
  --replay-only --speed 0.35 \`
  --output-dir build/stage5-visual-replay
\`\`\`

CI 使用 \`--replay-only --verify-only\` 检查帧时间和实际位移，不启动 GUI。
**GUI 是否在 Intel Windows 显卡上正确刷新，仍需 Windows 本机确认**。
不要把 CI 中无窗口回归 PASS 当作 Windows GUI 运行成功。

已知限制：当前仅验证三段**关节空间**轨迹，不能把末端实测蓝线描述为
Cartesian 直线/圆弧规划效果。回放只为查看已通过仿真的轨迹，不包含碰撞、
力矩级电机或实时图形控制性能验收。
