# six-axis-robot-motion-control

六轴工业机器人嵌入式运动控制固件开发与仿真验证。

> **开发状态（2026-10-27）**：第1至第4阶段的基础工程、驱动/协议、UR5 FK/IK、轨迹/PID和FreeRTOS任务均已实现。Stage2 Unity 25/25、ARM/QEMU协议8/8、UART TX 7/7、Motor抽象11/11、GPIO Shadow 13/13通过；Stage3/4 Host与QEMU回归及PyBullet动态验证均有记录。当前分支 `feat/stage5-system-integration-2026-10-27` 的本机更改仍未提交；阶段状态与证据见[第1至4阶段验收状态](docs/phase1-4_acceptance_status_2026-10-27.md)。

目前的架构：
- `common/` 定义公共类型、角度编码及统一数学常量。
- `model/` 统一UR5 DH、Frame与关节行程。
- `algorithms/{math,kinematics,motion,trajectory,control}/` 提供无FreeRTOS的纯计算。
- `application/{services,tasks}/` 管理请求、反馈、Trajectory/Control/Monitor任务。
- `tests/` / `simulation/` 提供Host、ARM/QEMU和同源C控制器的PyBullet验证。

**第1至4阶段仿真开发核心已完成，性能覆盖和完整文档交付仍需补齐**。现已有短时QEMU抖动/栈水位数据及Stage2目标测试，但30分钟高负载、全工作空间奇异性和更全面PID场景还没有完成。运动任务与目标输出CMake默认均为OFF。

### 统一非驱动本机验收（Windows PowerShell）

在仓库根目录，关闭占用调试端口的旧QEMU/GDB，再执行完整的第1至4阶段非驱动回归：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
```

日志保存在 `build/stage4-final-validation.log`。脚本验证UART常量一致性、Stage2 Unity、Stage2协议与UART TX/Motor/GPIO QEMU、六组Stage3/4 Host CTest、ARM算法及近奇异测试、FreeRTOS启动/任务自检、Native C桥和PyBullet单轴/六轴。命令输出默认关闭。

本轮新增多项式整段位置/速度/加速度预检（ARM轨迹20项、FreeRTOS拒绝/恢复8项），并将六轴独立PID、速度/加速度上限和超时策略集中到`firmware/freertos_demo/config/robot_defaults.json`。配置修改后重新构建；10项生成器测试拒绝非法输入。详见[程序补强记录](docs/part4/09_polynomial_preflight_and_json_config_2026-10-27.md)。

详细资料：
- [第四阶段参数登记簿](docs/part4/00_parameter_register.md)
- [第四阶段全局架构审查与验收缺口](docs/part4/05_global_architecture_review_2026-10-26.md)
- [PID调参真实数据](docs/part4/03_pid_tuning_record.md)
- [实时性和FreeRTOS自检结果](docs/part4/04_realtime_verification.md)

## 快速回归：运动限位与电机驱动（本机 C 编译器）

不依赖 ARM 工具链、QEMU 或 Unity，复用现有 Motion Limits 与 Motor Driver 测试用例：

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/motion-host
cmake --build build/motion-host
ctest --test-dir build/motion-host --output-on-failure
```

此测试不能替代 ARM 构建及 QEMU/PyBullet 端到端验证。

## 第1阶段遗留示例：UART ↔ PyBullet通信链路

> 以下仅用于Stage1通信与固定目标演示，**不是Stage4运动轨迹命令或完整串口PID闭环的测试流程**。现阶段实际生产运动请求由Application API提交。此路径保留供历史复现。

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
