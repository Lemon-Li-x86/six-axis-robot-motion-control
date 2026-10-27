# Stage5: QEMU/FreeRTOS/UART 实时 PyBullet 可视化

## 实时与回放的区别

- 实时运行：simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py。
  主进程首先打开 PyBullet GUI；工作进程同时运行真实 ARM QEMU、FreeRTOS、
  UART 帧收发及 PyBullet DIRECT 物理计算和反馈。**GUI 在固件仍运行时显示
  实际测得的当前关节位置，而不是等待完整 CSV 后回放。**
- 录制回放：stage5_qemu_uart_trace_replay.py，闭环完成后再从 CSV 逐帧播放。
  保留作为可复现的历史诊断工具。
- 原有单进程 visual_demo.py 的 GUI 模式有图形渲染/反馈时序干扰风险，
  本轮不再推荐用其 frame-delay-ms 来实现动画慢放。

## 职责与安全边界

1. 工作进程：重用 stage5_qemu_uart_visual_demo.run 的完整串口运动队列、
   实际 C FreeRTOS 轨迹/PID、PyBullet POSITION_CONTROL、反馈、诊断和验收。
2. GUI 主进程：只镜像最新实测六轴关节角和末端位置。其 resetJointState
   **不是新的物理闭环，不回传到 QEMU，也不重新计算控制量**。
3. 工作进程通过非阻塞有界消息队列发送姿态。图形慢时丢弃过期的**显示帧**，
   不丢弃串口反馈帧。状态以同一 worker 的 joint_trace.csv 和 result.json 为准。
4. GUI 显示反映闭环状态随时间变化，但不是“硬件毫秒级实时性能证明”；
   两个进程有调度/IPC/渲染延迟，也不能证明实际电机伺服和安全。
5. 仅 QEMU 虚拟 UART 电机目标，不接真实机械臂。

## Windows 运行

在项目根目录的 PowerShell，工作区无须保留改动时可切换实时功能分支：

~~~powershell
git status --short
git fetch origin
git switch --track origin/feat/stage5-live-qemu-pybullet-gui-20261027
~~~

如果同名本地分支已存在，使用 git switch feat/stage5-live-qemu-pybullet-gui-20261027，然后 git pull --ff-only。

使用唯一 CMake Stage5 UART 编译门控重新构建实际 ARM ELF（与以前一样的配置）：

~~~powershell
cmake -S . -B build/stage5-live-uart -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake' -DCMAKE_BUILD_TYPE=Debug -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON -DROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=ON -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF
cmake --build build/stage5-live-uart --target freertos_demo

.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py --elf build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf --scenario six_axis --output-dir build/stage5-live-windows
~~~

**观察预期**：PyBullet GUI 先打开；终端输出 [LIVE]；
三个运动段逐步更新 GUI 姿态与末端轨迹；工作进程报告
[LIVE/CAPTURE] PASS、父进程输出 [LIVE] PASS；窗口保持至按 Enter。
这不是录制后重放。若 GUI 反应慢，可能跳过显示中间帧，但底层物理
目标/反馈仍按 worker 实际执行。

## 自动化与验收边界

- 真实 QEMU 测试：.github/workflows/stage5-live-gui.yml 中使用
  --headless-observer 来验证**固件运行期间**确实收到实际物理关节帧。
- Python unit：tools/tests/test_stage5_live_gui.py 检查显示队列溢出时不阻塞、
  姿态跟踪和无效数据拒绝。
- 只有用户本机可以验收 Windows OpenGL GUI 实际显示；
  headless CI 不能冒充 GUI 图形测试。
- 保存 joint_trace.csv、result.json 作为证据；不修改原 PID 参数或
  重写 Cartesian 规划，也不能据此断言实物硬件安全。
