# six-axis-robot-motion-control

六轴工业机器人嵌入式运动控制固件开发与仿真验证。

当前已完成第1阶段基础环境搭建与通信链路验证。

## 第1阶段运行方法

### 1. 启动 Cortex-M4 / FreeRTOS 通信端

在项目根目录执行：

```powershell
cmake --build build/cmake-arm
```

然后启动 QEMU UART TCP：

```powershell
qemu-system-arm `
    -M mps2-an386 `
    -kernel build/cmake-arm/firmware/freertos_demo/freertos_demo.elf `
    -serial tcp:127.0.0.1:5555,server=on,wait=off `
    -monitor none
```

也可以在 VS Code 中直接运行任务：

```text
Run FreeRTOS UART TCP
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

当前测试目标为：

```text
Joint 1 = 60°
Joint 2~6 = 0°
```

终端应出现：

```text
RX 0x01 目标角:
[60.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

PyBullet 中 UR5 第一轴将运动到约 60°。

随后可以看到状态反馈：

```text
TX 0x81 实际角度:
[60.0, ...]
```

以及 Cortex-M4 返回的确认：

```text
RX 0x82 MCU 已确认状态:
[60.0, ...]
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