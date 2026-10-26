# six-axis-robot-motion-control

六轴工业机器人嵌入式运动控制固件开发与仿真验证。

> **开发状态（2026-10-26）**：Stage4 核心轨迹规划、解析 IK、六轴 PID、FreeRTOS Control/Trajectory/Monitor 和非驱动仿真回归已经实现。用户确认本机8步统一测试通过；最新GitHub CI补测了失联故障注入 **13/13**、Host回归、PyBullet单轴/六轴及PowerShell语法测试。它们不是实体工业机器人安全认证。开发分支尚未合并main，两份备份不动。

目前的架构：
- `common/` 定义公共类型、角度编码及统一数学常量。
- `model/` 统一UR5 DH、Frame与关节行程。
- `algorithms/{math,kinematics,motion,trajectory,control}/` 提供无FreeRTOS的纯计算。
- `application/{services,tasks}/` 管理请求、反馈、Trajectory/Control/Monitor任务。
- `tests/` / `simulation/` 提供Host、ARM/QEMU和同源C控制器的PyBullet验证。

**第4阶段核心软件回归通过，但系统验收尚未完成**。后续需要真实UART↔PyBullet端到端运动请求、反馈断链与故障场景、周期P99/WCET与长时运行、实物急停/硬件保护。Stage4任务和命令输出CMake默认均为OFF；未经独立安全设计，不得用于实体机器人运动。

### 统一非驱动本机验收（Windows PowerShell）

在仓库根目录，关闭旧QEMU/GDB，再执行：

```powershell
git fetch origin
git switch feat/stage4-trajectory-2026-10-26
git merge --ff-only origin/feat/stage4-trajectory-2026-10-26
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
```

日志保存在 `build/stage4-final-validation.log`。脚本验证 UART C/Python常量一致性、6组Host CTest、ARM编译、三组算法QEMU、FreeRTOS启动/故障注入、Native C桥、PyBullet单轴/六轴。脚本不发送真实电机指令。

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