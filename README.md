# six-axis-robot-motion-control

六轴工业机器人嵌入式运动控制固件开发与仿真验证。


当前已完成 Cortex-M4 / FreeRTOS 基础运行环境、UART 双向通信链路，以及 UR5 正运动学、解析逆运动学和关节限位算法的主要实现。

当前 Motion Limits 已加入固件构建并配置独立 QEMU 测试目标，但尚未完整接入实际运动指令链路。

轨迹规划、PID 闭环控制以及独立的 Trajectory / Control Task 属于后续开发内容。


## 快速回归：运动限位（本机 C 编译器）

不依赖 ARM 工具链、QEMU 或 Unity，复用现有 Motion Limits 测试用例：

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/motion-host
cmake --build build/motion-host
ctest --test-dir build/motion-host --output-on-failure
```

此测试不能替代 ARM 构建及 QEMU/PyBullet 端到端验证。

## 第1阶段运行方法

### 1. 启动 Cortex-M4 / FreeRTOS 通信端

在项目根目录执行：


配置 ARM / QEMU 工程
```text
cmake --preset arm-qemu
```

构建正式固件
```text
cmake --build --preset firmware
```

如需构建所有 QEMU 测试目标
```text
cmake --build --preset arm-qemu
```

然后启动 QEMU UART TCP：

```powershell
qemu-system-arm `
    -M mps2-an386 `
    -kernel build/cmake-arm/firmware/freertos_demo/freertos_demo.elf `
    -serial tcp:127.0.0.1:5555,server=on,wait=off `
    -monitor none
```

QEMU 启动后会监听：

```text
127.0.0.1:5555
```

---

### 2. 启动 Python / PyBullet 通信桥

打开另一个终端，运行：

```powershell
python simulation\uart_bridge\ur5_uart_bridge.py
```

程序将自动：

- 连接 QEMU UART TCP；
- 启动 PyBullet；
- 加载 UR5；
- 接收 Cortex-M4 下发的目标关节角；
- 控制机械臂运动；
- 读取实际关节状态并返回 Cortex-M4。

---

### 3. 预期结果

初始目标角由 `firmware/freertos_demo/config/robot_defaults.json` 的
`initial_joint_targets_centideg` 决定，单位为 0.01°。下方日志仅为示意，
不表示当前配置或启动瞬间已经达到目标。

随后可以看到状态反馈：

```text
TX 0x81 实际角度:
[目标附近的反馈角度, ...]
```

以及 Cortex-M4 返回的确认：

```text
RX 0x82 MCU 已确认状态:
[目标附近的反馈角度, ...]
```

同时出现：

```text
RX 0x01
TX 0x81
RX 0x82
```

表示 Cortex-M4、Python 与 PyBullet / UR5 的双向通信链路工作正常。

---

## 第1阶段文档

系统技术方案设计：

```text
docs/part1/01_system_design.md
```

环境搭建与通信链路验证：

```text
docs/part1/04_env.md
```