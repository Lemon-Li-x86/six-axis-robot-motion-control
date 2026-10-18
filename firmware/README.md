# Six-Axis Robot Motion Control

六轴工业机器人嵌入式运动控制固件开发与仿真验证

**目标平台：** ARM Cortex-M4 / FreeRTOS / QEMU MPS2-AN386  
**仿真环境：** Python / PyBullet / UR5  
**构建系统：** CMake + Ninja + ARM GNU Toolchain  
**当前开发基线：** `refactor/architecture-cleanup-20261016`

---

## 1. 项目简介

本项目旨在设计、开发和验证一套基于 ARM Cortex-M4 与 FreeRTOS 的六轴工业机器人运动控制固件，并通过 QEMU 和 PyBullet 建立可重复的软硬件协同仿真环境。

整体系统分为：

1. **嵌入式固件层**：负责实时任务调度、通信、机器人状态管理以及运动学算法。
2. **通信与仿真接口层**：通过 QEMU UART TCP 接口连接 Python，实现固件与仿真环境的数据交换。
3. **机器人仿真层**：通过 PyBullet 加载 UR5 模型，执行关节位置控制并返回实际关节状态。
4. **测试与验证层**：通过 CTest、QEMU、GDB 和 Python 测试程序验证各模块的正确性。

### 1.1 当前开发进度

**已经实现：**

- Cortex-M4 裸机启动和 FreeRTOS 运行环境
- ProtocolRX 和 ProtocolTX 两个 FreeRTOS 业务任务
- UART 中断收发、Ring Buffer、任务通知
- 自定义通信协议及协议解析状态机
- UART 完整帧发送互斥保护
- Motor Driver 仿真接口
- 六轴目标角、反馈角、连续角和速度估计
- 电机反馈同步访问、快照及时间戳管理
- UR5 机器人模型与关节限位参数
- 正运动学（FK）和解析逆运动学（IK）
- 连续角展开与 Motion Limits 算法
- 性能监测与运行时参数配置
- 本机 CTest 和 ARM/QEMU 独立测试固件

**尚未实现或尚未完整集成：**

- 独立的 Trajectory Task
- 独立的 Control Task
- PID 闭环控制
- 轨迹规划和插补
- 将 IK / Motion Limits 完整接入运动执行链
- 实际电机控制硬件接口
- 反馈超时后的安全停机和故障恢复
- 真实机器人硬件验证

注意：已经存在某个算法文件，并不等于该算法已经接入机器人实际执行流程。

---

## 2. 系统总体架构

### 2.1 运行链路

```text
┌────────────────────────────────────────────┐
│        ARM Cortex-M4 / FreeRTOS            │
│                 QEMU                       │
│                                            │
│  ProtocolTX Task                           │
│       │                                    │
│       ▼                                    │
│  Motor State / Motor Driver                │
│       │                                    │
│       ▼                                    │
│  Protocol Encoder                          │
│       │                                    │
│       ▼                                    │
│  UART TX Manager / UART Driver             │
│                                            │
│  ProtocolRX Task                           │
│       ▲                                    │
│       │                                    │
│  Protocol Parser                           │
│       ▲                                    │
│       │                                    │
│  UART RX Driver                            │
└───────┼────────────────────────────────────┘
        │
        │ QEMU UART TCP
        │ 127.0.0.1:5555
        │
        ▼
┌────────────────────────────────────────────┐
│           Python UART Bridge               │
│                                            │
│  Protocol Codec                            │
│  Angle Conversion                          │
│  Socket Communication                      │
└──────────────────┬─────────────────────────┘
                   │
                   ▼
┌────────────────────────────────────────────┐
│              PyBullet                      │
│                                            │
│              UR5 Robot                     │
│                                            │
│  Joint Position Control                    │
│  Physics Simulation                        │
│  Joint State Feedback                      │
└────────────────────────────────────────────┘
```

### 2.2 通信方向

固件到 PyBullet：

```text
ProtocolTX
    ↓
获取六轴目标位置
    ↓
构造协议帧
    ↓
UART Driver
    ↓
QEMU TCP
    ↓
Python Bridge
    ↓
PyBullet Joint Position Control
```

PyBullet 到固件：

```text
PyBullet Joint State
    ↓
Python Bridge
    ↓
QEMU TCP
    ↓
UART RX Interrupt
    ↓
Ring Buffer
    ↓
ProtocolRX Task
    ↓
Protocol Parser
    ↓
Protocol Commands
    ↓
Motor State / Motor Driver
    ↓
反馈角度、连续角度、速度和时间戳
```

---

## 3. 项目目录结构

```text
six-axis-robot-motion-control/
│
├── .github/
│   └── workflows/
│       └── native-motion-regression.yml
│
├── cmake/
│   └── arm-none-eabi-gcc.cmake
│
├── docs/
│   ├── architecture.md
│   ├── development.md
│   ├── protocol.md
│   ├── system_design.md
│   ├── cortex_m4_notes.md
│   ├── part1/
│   ├── part2/
│   ├── testing/
│   └── weeklyReports/
│
├── firmware/
│   ├── hello/
│   └── freertos_demo/
│       ├── main.c
│       ├── startup.s
│       ├── linker.ld
│       ├── memory.c
│       ├── board.c
│       ├── board.h
│       ├── FreeRTOSConfig.h
│       ├── CMakeLists.txt
│       │
│       ├── common/
│       ├── config/
│       ├── model/
│       ├── drivers/
│       ├── communications/
│       ├── application/
│       ├── algorithms/
│       ├── diagnostics/
│       └── tests/
│
├── simulation/
│   ├── pybullet_ur5/
│   └── uart_bridge/
│
├── third_party/
│   └── README.md
│
├── tools/
│   ├── generate_firmware_defaults.py
│   ├── analyze_fk_batch.py
│   └── analyze_ik_batch.py
│
├── CMakeLists.txt
├── CMakePresets.json
├── requirements.txt
├── .gitignore
└── README.md
```

其中 `build/` 和 `.venv/` 为本地生成目录，不包含在 Git 仓库内。

---

## 4. 固件文件说明

以下文件均位于：

`firmware/freertos_demo/`

### 4.1 固件入口与平台配置

| 文件 | 作用 |
|---|---|
| `main.c` | 固件主入口，完成各驱动、服务、测试和 FreeRTOS 任务初始化 |
| `startup.s` | Cortex-M4 启动汇编、中断向量表及复位入口 |
| `linker.ld` | 固件 FLASH、RAM、代码段和数据段布局 |
| `memory.c` | 裸机环境所需的基础内存操作 |
| `board.c` | 板级初始化，包括时钟及 GPIO 配置接口 |
| `board.h` | 板级时钟参数和公共接口 |
| `FreeRTOSConfig.h` | FreeRTOS Tick、堆、调度和中断优先级等配置 |
| `CMakeLists.txt` | 固件和独立 QEMU 测试目标的构建定义 |

`main.c` 的初始化大致包括：

```text
Board Initialization
    ↓
UART / Timer / GPIO Initialization
    ↓
Runtime Config
    ↓
UART TX Manager
    ↓
Performance Monitor
    ↓
Optional Startup Tests
    ↓
Motor Driver Initialization
    ↓
Initial Joint Targets
    ↓
Motor State Synchronization
    ↓
Create Protocol Tasks
    ↓
Start FreeRTOS Scheduler
```

如果 QEMU 正式固件启动后没有 UART 输出，应检查是否卡在 `main.c` 的启动自测或初始化失败循环中，而不是立即认定 UART 出错。

### 4.2 公共定义 `common/`

| 文件 | 作用 |
|---|---|
| `robot_types.h` | 六轴关节角、连续位置、速度、变换矩阵等公共类型 |
| `error_code.h` | 模块统一返回状态和错误码 |

关节角单位约定：

| 类型 | 存储方式 | 单位 |
|---|---|---|
| Canonical Joint Angle | `int16_t` | 0.01° |
| Continuous Joint Position | `int32_t` | 0.01° |
| Joint Velocity | 浮点数 | °/s |

Canonical Angle 范围为 `[-180°, 180°)`。

通信协议使用规范角，连续角用于处理跨越 ±180° 的运动和多圈位置状态。

### 4.3 机器人模型 `model/`

| 文件 | 作用 |
|---|---|
| `robot_model.h` | 机器人模型结构体和访问接口 |
| `ur5_model.c` | UR5 几何参数、坐标变换和关节限位 |

该目录保存机器人静态模型，不保存实时运动状态。

更换机器人型号时，应首先检查这里以及 PyBullet 对应的 URDF 模型。

### 4.4 驱动层 `drivers/`

| 文件 | 作用 |
|---|---|
| `uart_driver.c/.h` | UART MMIO、中断、Ring Buffer 收发、RX Drop 统计 |
| `timer_driver.c/.h` | Timer 计数、频率查询、周期中断 |
| `gpio_driver.c/.h` | GPIO 输出控制，当前 QEMU 使用软件 Shadow Backend |
| `motor_driver.c/.h` | 六轴目标、反馈、连续角及速度状态管理 |

**Motor Driver 当前属于 Simulation Backend，并非真实伺服驱动。**

`motor_driver_update_feedback()` 会先计算六轴的新状态，并检查连续位置是否发生整数溢出。只有整组反馈有效时才提交更新，避免失败后出现部分关节已更新、部分未更新的情况。

### 4.5 通信层 `communications/`

| 文件 | 作用 |
|---|---|
| `protocol.c/.h` | 通信帧编码、解码、校验及字节流解析 |
| `ring_buffer.c/.h` | UART 接收和发送环形缓冲区 |

协议帧：

```text
AA 55 | Command | Length | Payload | Checksum
```

主要命令：

| 命令 | 方向 | 功能 |
|---|---|---|
| `0x01` | MCU → Python | 六轴目标角 |
| `0x81` | Python → MCU | 六轴状态反馈 |
| `0x82` | MCU → Python | 状态反馈 ACK |
| `0x02` | 请求 | 修改运行参数 |
| `0x85` | 响应 | 参数配置 ACK |
| `0x83` | 请求 | 查询诊断信息 |
| `0x84` | 响应 | 诊断结果 |

具体数据长度和协议约束见 `docs/protocol.md`。

### 4.6 应用层 `application/`

| 文件 | 作用 |
|---|---|
| `protocol_tasks.c/.h` | 创建 ProtocolRX/ProtocolTX，处理任务通知、收发和调度 |
| `services/protocol_commands.c/.h` | 完整协议帧的业务处理、状态 ACK、参数与诊断响应 |
| `motor_state.c/.h` | 使用 FreeRTOS Mutex 统一保护电机状态读写 |
| `uart_tx_manager.c/.h` | 保证多个任务发送的完整协议帧不交叉 |
| `runtime_config.c/.h` | 管理运行时可修改参数 |
| `task_config.h` | 任务优先级、周期和栈大小的配置接口 |

**并发要求：**

当前 ProtocolRX 是反馈的唯一写入任务。

ProtocolRX 和 ProtocolTX 运行期间，电机反馈与目标读写需要通过 `motor_state` 提供的同步接口。

未来 Control Task 必须遵循同样约束，避免绕过 Mutex 直接读取 Motor Driver 的内部共享状态。

`motor_state_feedback_is_fresh()` 只负责判断接收到的反馈是否过期，目前没有自动触发安全停机。

### 4.7 数学与运动学 `algorithms/`

| 文件 | 作用 |
|---|---|
| `math/matrix4.c/.h` | 四阶齐次变换矩阵运算 |
| `kinematics/kinematics.h` | 对外提供 FK/IK 接口 |
| `kinematics/kinematics_forward.c` | 正运动学 |
| `kinematics/kinematics_inverse.c` | 逆运动学入口、分支组合、去重及结果输出 |
| `kinematics/ur5_analytic_ik.c` | UR5 解析 IK 的关节数学求解 |
| `kinematics/kinematics_common.c` | 运动学内部共享计算和转换 |
| `kinematics/kinematics_internal.h` | 运动学内部接口 |
| `kinematics/kinematics_private.h` | 模块私有声明及辅助接口 |
| `motion/motion_limits.c/.h` | 关节限位检查及连续角展开 |

注意：运动学结果是候选位置解，不能不经轨迹规划、限位和安全检查直接视为可执行运动指令。

### 4.8 性能诊断 `diagnostics/`

| 文件 | 作用 |
|---|---|
| `performance_monitor.c/.h` | 协议解析耗时、任务唤醒延迟和性能统计 |

用于验证 ISR → Task 通知路径以及协议解析的执行性能。

---

## 5. Python 和仿真文件说明

### 5.1 `simulation/uart_bridge/`

| 文件 | 功能 |
|---|---|
| `ur5_uart_bridge.py` | 正式 QEMU ↔ PyBullet 双向通信与仿真入口 |
| `protocol_codec.py` | Python 侧通信协议编解码 |
| `angle_utils.py` | 规范角归一化、连续角展开 |
| `uart_receiver.py` | 仅接收并显示 UART 数据的早期测试工具 |
| `communication_soak_test.py` | 长时间通信稳定性测试 |
| `communication_stress_test.py` | 不同发送速率下的通信压力测试 |
| `burst_stress_test.py` | 突发数据帧压力测试 |
| `protocol_fault_test.py` | 错误帧与异常输入后的协议恢复测试 |
| `parameter_config_test.py` | 运行时参数配置功能测试 |
| `performance_baseline.py` | UART 通信往返延迟 RTT 测量 |
| `internal_performance_metrics.py` | 读取固件内部计时器和任务性能指标 |

这些测试程序大多直接连接 QEMU 的 TCP UART。

**同一时刻不要让 PyBullet Bridge 与通信压力测试程序同时占用同一个 UART 连接。**

### 5.2 `simulation/pybullet_ur5/`

```text
pybullet_ur5/
├── models/
│   └── ur5/
│       ├── urdf/
│       │   └── ur5.urdf
│       └── meshes/
│           └── ur5/
│               ├── collision/
│               └── visual/
└── src/
    ├── hello_pybullet.py
    ├── scene_demo.py
    ├── ur5_demo.py
    ├── qemu_uart_receiver.py
    └── fk_cross_validation.py
```

| 文件 | 作用 |
|---|---|
| `ur5.urdf` | UR5 关节结构、连杆和坐标定义 |
| `collision/*.stl` | 碰撞模型 |
| `visual/*.dae` | 可视化模型 |
| `hello_pybullet.py` | 最基本的 PyBullet GUI/物理仿真测试 |
| `scene_demo.py` | UR5、地面和障碍物场景演示 |
| `ur5_demo.py` | 手动滑块控制 UR5 关节 |
| `qemu_uart_receiver.py` | 仿真目录中的 UART 接收示例 |
| `fk_cross_validation.py` | 使用 PyBullet URDF 验证 FK 结果 |

---

## 6. 测试文件说明

所有固件测试位于：

`firmware/freertos_demo/tests/`

### 6.1 Host Tests

路径：`tests/host/`

- `CMakeLists.txt`：定义三个本机 CTest 目标。
- `motor_feedback_safety_test.c`：测试反馈状态、长时间角度展开、整数溢出、更新失败回滚和快照行为。
- `README.md`：Host Tests 的独立说明。

三个测试目标：

```text
motion_limits_host
motor_driver_host
motor_feedback_safety_host
```

### 6.2 QEMU Target Tests

| 目标 | 测试内容 |
|---|---|
| `qemu_protocol_tests` | Protocol 和 Ring Buffer |
| `qemu_uart_tx_tests` | UART TX Buffer 边界 |
| `qemu_motor_tests` | Motor Driver |
| `qemu_gpio_tests` | GPIO Shadow Backend |
| `qemu_model_limits_tests` | UR5 模型关节限位 |
| `qemu_motion_limits_tests` | Motion Limits |
| `qemu_fk_batch` | FK 批量验证数据生成 |
| `qemu_ik_batch` | IK 批量诊断数据生成 |

`tests/qemu_target/test_startup.s` 是独立测试固件的精简启动程序，不是正式 FreeRTOS 固件的启动文件。

### 6.3 Kinematics Self-Test

路径：`tests/kinematics/`

| 文件 | 作用 |
|---|---|
| `kinematics_self_test.c/.h` | 统一执行运动学自测 |
| `kinematics_fk_test.c/.h` | FK 测试 |
| `kinematics_ik_solver_test.c/.h` | IK 分步求解器测试 |
| `kinematics_ik_test.c/.h` | IK 公共接口测试 |
| `kinematics_dedup_test.c/.h` | 逆解去重测试 |
| `kinematics_test_support.c/.h` | 测试辅助函数 |

默认可以在固件启动 Scheduler 前执行。测试失败可能导致固件停在初始化阶段。

### 6.4 Unity Tests

路径：`tests/unity/`

包含早期 Protocol、Motor、Ring Buffer 和 UART 驱动的 Unity 单元测试。

需要单独下载 Unity 到 `third_party/Unity/`。

此测试工程独立于 Host Tests 和 ARM 构建预设。

---

# 7. Windows 开发环境配置

以下命令均在项目根目录的 PowerShell 中执行。

## 7.1 系统工具

需要安装：

- Git
- CMake ≥ 3.20
- Ninja
- ARM GNU Toolchain
- QEMU
- ARM GDB
- Python 3.11
- uv（推荐）
- Visual Studio C++ Build Tools 或 MinGW GCC（Windows 本机测试）

检查：

```powershell
git --version
cmake --version
ninja --version
arm-none-eabi-gcc --version
arm-none-eabi-gdb --version
qemu-system-arm --version
uv --version
```

ARM GDB 用于远程连接 QEMU。如果安装的 ARM 工具链没有提供 `arm-none-eabi-gdb`，需要额外安装对应调试器。

## 7.2 克隆 FreeRTOS

```powershell
git clone --depth 1 --branch V11.1.0 https://github.com/FreeRTOS/FreeRTOS-Kernel.git third_party/FreeRTOS-Kernel
```

检查：

```powershell
Test-Path .\third_party\FreeRTOS-Kernel\tasks.c
```

预期：

```text
True
```

如果 FreeRTOS 文件夹已经存在且完整，不必重新克隆。

## 7.3 建立本地 Python 虚拟环境

使用 Python 3.11：

```powershell
uv venv --python 3.11 .venv
```

安装项目依赖：

```powershell
uv pip install --link-mode copy --python .venv\Scripts\python.exe -r requirements.txt
```

验证：

```powershell
.\.venv\Scripts\python.exe --version
.\.venv\Scripts\python.exe -c "import pybullet; print('PyBullet OK')"
```

VS Code 中执行：

```text
Ctrl + Shift + P
→ Python: Select Interpreter
→ .venv\Scripts\python.exe
```

注意：`tools/analyze_fk_batch.py` 和 `tools/analyze_ik_batch.py` 还使用 NumPy、SciPy、Robotics Toolbox 等包，目前不属于基础 `requirements.txt`。需要运行对应专项分析时再安装。

---

# 8. CMake 配置与 ARM 固件构建

在根目录执行：

```powershell
$python = (Resolve-Path .\.venv\Scripts\python.exe).Path

cmake --preset arm-qemu "-DCMAKE_BUILD_TYPE=Debug" "-DPython3_EXECUTABLE=$python"
```

构建正式固件：

```powershell
cmake --build --preset firmware
```

构建正式固件及全部 QEMU 测试目标：

```powershell
cmake --build --preset arm-qemu
```

正式固件输出路径：

```text
build/cmake-arm/firmware/freertos_demo/freertos_demo.elf
```

如果 CMake 配置异常，可以清理该构建缓存后重新配置：

```powershell
Remove-Item -Recurse -Force .\build\cmake-arm
```

注意：仅删除构建目录，不要删除源码或 `.venv`。

---

# 9. 本机 CTest 调试

执行：

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/motion-host

cmake --build build/motion-host

ctest --test-dir build/motion-host --output-on-failure
```

通过标准：

```text
100% tests passed, 0 tests failed out of 3
```

预期三项：

```text
motion_limits_host
motor_driver_host
motor_feedback_safety_host
```

如果出现本机 C 编译器缺失，应配置 Windows MSVC 或 MinGW GCC。

ARM 交叉编译器不适合作为这些 Windows 原生测试程序的编译器。

---

# 10. QEMU + GDB 调试

这里测试的是 ARM Cortex-M4 程序是否真正在 QEMU 中运行，而不仅是编译生成 ELF。

## 10.1 QEMU 测试目标

首先构建：

```powershell
cmake --build --preset arm-qemu
```

例如运行 Motor Driver 测试。

### 终端 A：QEMU

```powershell
qemu-system-arm `
  -M mps2-an386 `
  -kernel .\build\cmake-arm\firmware\freertos_demo\qemu_motor_tests.elf `
  -S `
  -gdb tcp::1234 `
  -serial null `
  -monitor none `
  -display none
```

其中：

- `-M mps2-an386`：使用 Cortex-M4 模拟开发板。
- `-kernel`：指定测试固件。
- `-S`：暂停 CPU，等待调试器连接。
- `-gdb tcp::1234`：开放 GDB 远程调试端口。
- `-serial null`：测试程序不通过 UART 输出结果。

### 终端 B：GDB

```powershell
arm-none-eabi-gdb .\build\cmake-arm\firmware\freertos_demo\qemu_motor_tests.elf
```

进入 GDB 后：

```gdb
set pagination off
target remote localhost:1234
break qemu_motor_test_done
continue
```

读取结果：

```gdb
p g_motor_test_report.total
p g_motor_test_report.passed
p g_motor_test_report.failed
p g_motor_test_report.results
```

通过标准：

1. GDB 命中测试结束断点。
2. `failed == 0`。
3. `passed == total`。

**仅命中结束断点不代表测试通过。**

## 10.2 各测试目标对应的 GDB 断点

| 测试固件 | 断点 | 报告变量 |
|---|---|---|
| `qemu_protocol_tests.elf` | `qemu_target_test_done` | `g_qemu_target_test_report` |
| `qemu_uart_tx_tests.elf` | `qemu_uart_tx_test_done` | `g_uart_tx_test_report` |
| `qemu_motor_tests.elf` | `qemu_motor_test_done` | `g_motor_test_report` |
| `qemu_gpio_tests.elf` | `qemu_gpio_test_done` | `g_gpio_test_report` |
| `qemu_model_limits_tests.elf` | `qemu_model_limits_done` | `g_model_limits_test_report` |
| `qemu_motion_limits_tests.elf` | `qemu_motion_limits_done` | `g_motion_limits_test_report` |
| `qemu_fk_batch.elf` | `qemu_fk_batch_done` | `g_fk_samples` |
| `qemu_ik_batch.elf` | `qemu_ik_batch_done` | `g_ik_samples` |

每次切换测试固件前，应关闭上一组 QEMU，避免 GDB 1234 端口占用。

FK 和 IK 批量测试需要额外导出内存数据，通过 Python 检查误差。完成采样不等于精度测试通过。

## 10.3 正式固件调试

调试 FreeRTOS 正式固件时，使用：

```text
freertos_demo.elf
```

而不是 `qemu_motor_tests.elf` 等独立测试固件。

建议优先设置的断点：

```text
main
protocol_tasks_start
task_protocol_rx
task_protocol_tx
protocol_commands_dispatch_frame
motor_state_update_feedback
```

其中 `task_protocol_rx`、`task_protocol_tx` 是 `protocol_tasks.c` 中的静态函数，需要使用包含调试符号的 ELF。GDB 中可通过源码位置或相应符号设置断点。

---

# 11. PyBullet 仿真调试

## 11.1 先验证 PyBullet 能运行

```powershell
.\.venv\Scripts\python.exe simulation\pybullet_ur5\src\hello_pybullet.py
```

预期：

- PyBullet GUI 正常启动
- 可以加载地面
- 物理仿真正常运行

然后验证 UR5 模型：

```powershell
.\.venv\Scripts\python.exe simulation\pybullet_ur5\src\ur5_demo.py
```

预期：

- 加载 UR5
- 可以使用滑块控制指定关节
- 终端显示 Target 和 Actual Angle

如果 GUI 正常但 UR5 加载失败，应检查 URDF 和 mesh 文件路径。

## 11.2 启动 QEMU 正式固件

PowerShell 终端 A：

```powershell
qemu-system-arm `
  -M mps2-an386 `
  -kernel .\build\cmake-arm\firmware\freertos_demo\freertos_demo.elf `
  -serial "tcp:127.0.0.1:5555,server=on,wait=off" `
  -monitor none `
  -display none
```

此命令会让 QEMU 在：

```text
127.0.0.1:5555
```

提供 UART TCP 服务。

## 11.3 启动 Python 通信桥

PowerShell 终端 B：

```powershell
.\.venv\Scripts\python.exe simulation\uart_bridge\ur5_uart_bridge.py
```

程序将：

1. 连接 QEMU UART TCP。
2. 打开 PyBullet。
3. 加载 UR5。
4. 读取 MCU 发送的六轴目标角。
5. 控制仿真关节。
6. 返回实际关节状态。
7. 接收 MCU 对状态帧的 ACK。

## 11.4 通信检查

预期出现：

```text
RX 0x01
TX 0x81
RX 0x82
```

对应：

| 标识 | 意义 |
|---|---|
| `RX 0x01` | Python 收到 MCU 发出的目标关节角 |
| `TX 0x81` | Python 向 MCU 发送实际关节状态 |
| `RX 0x82` | Python 收到 MCU 的状态接收确认 |

目前默认目标：

```text
[9000, 0, 0, 0, 0, 0]
```

单位为 0.01°，即：

```text
[90°, 0°, 0°, 0°, 0°, 0°]
```

预期 UR5 第一轴向 90° 位置运动。运动是否稳定以及最终误差，需要观察实际关节状态，不能仅凭通信日志判定。

---

# 12. 通信稳定性测试

停止 PyBullet Bridge 后，必要时重启 QEMU，然后运行：

```powershell
.\.venv\Scripts\python.exe `
  simulation\uart_bridge\communication_soak_test.py `
  --duration 60 `
  --rate 10 `
  --ack-timeout 0.2 `
  --progress-interval 10
```

此测试持续 60 秒，目标发送速率为 10 Hz。

关注：

- Sent：发送帧数量
- ACK：成功确认数量
- Timeout：超时数量
- ACK Success：成功率
- RX Drop：接收字节丢弃数量

通信压力测试：

```powershell
.\.venv\Scripts\python.exe simulation\uart_bridge\communication_stress_test.py
```

突发压力测试：

```powershell
.\.venv\Scripts\python.exe simulation\uart_bridge\burst_stress_test.py
```

协议异常恢复测试：

```powershell
.\.venv\Scripts\python.exe simulation\uart_bridge\protocol_fault_test.py
```

这些测试应分别运行，避免多个 Python 程序同时占用相同的 QEMU UART TCP 连接。

---

# 13. 常见问题排查

| 问题 | 可能原因 | 处理方法 |
|---|---|---|
| CMake 找不到 `tasks.c` | FreeRTOS 未下载 | 克隆 FreeRTOS Kernel V11.1.0 |
| CMakeLists.txt 标红 | 依赖缺失或 CMake 缓存错误 | 修复依赖后重新 Configure |
| ARM GCC 找不到 | 工具链未安装或 PATH 错误 | 检查环境变量 |
| Python 3.11 找不到 | 使用 uv 管理 Python | `uv venv --python 3.11 .venv` |
| PyBullet 安装很慢 | Windows 正在编译 C++ 扩展 | 检查 MSVC 编译日志 |
| uv hardlink 警告 | 缓存和 venv 在不同磁盘 | 使用 `--link-mode copy` |
| 本机 CTest 无法编译 | 没有 Windows 本机 C 编译器 | 配置 MSVC/MinGW |
| QEMU 调试启动后不运行 | 使用了 `-S` | 用 GDB 连接并执行 `continue` |
| PyBullet 连接失败 | 5555 未监听或端口冲突 | 检查 QEMU 正式固件启动 |
| 有目标帧但没有 ACK | UART RX、协议解析或反馈更新异常 | 检查 ProtocolRX 和 Motor State |
| 关节角跨 ±180° 跳变 | 规范角与连续角混淆 | 检查 `angle_utils.py` 和 Motor Driver |
| 固件没有正常发送数据 | 启动自测或初始化可能失败 | 使用 GDB 定位 `main.c` |
| 配置修改没有生效 | 生成头文件尚未更新 | 重新执行 CMake Build |

---

# 14. Git 与开发规范

当前 Cleanup 工作分支：

```text
refactor/architecture-cleanup-20261016
```

建议后续新增独立功能时，从 Cleanup 基线创建功能分支，避免将不同开发阶段混在同一批提交中。

检查状态：

```powershell
git branch --show-current
git status
git diff --check
```

开发原则：

1. 不直接修改 `main` 或历史备份分支。
2. 不将 `.venv`、`build` 或第三方内核源码提交到仓库。
3. 不在应用层重复实现通信协议解析。
4. 不让纯算法直接依赖 UART、FreeRTOS 或 Python。
5. 多任务电机状态访问必须遵守统一同步策略。
6. 新增功能需要对应的正常、边界和异常测试。
7. 区分编译通过、QEMU 执行通过和 PyBullet 联动通过。

---

# 15. 后续开发计划

### 下一阶段：运动控制链集成

计划内容：

- IK 结果有效性检查
- 关节限位与连续解选择
- 轨迹规划与插补
- 轨迹执行目标接口
- 独立 Trajectory Task
- PID 控制器
- 独立 Control Task
- 反馈超时和故障安全策略

建议数据链路：

```text
Target Pose / Joint Command
        ↓
Inverse Kinematics
        ↓
Joint Limits
        ↓
Continuous Solution Selection
        ↓
Trajectory Planning
        ↓
Trajectory Sampling
        ↓
Control Task
        ↓
Motor State / Motor Driver
        ↓
Python Bridge
        ↓
PyBullet UR5
        ↓
Joint Feedback
        ↓
Motor State
        ↓
Control Task
```

以上属于规划架构，不代表当前已经全部实现。

### 验证原则

每次新增关键功能后，至少完成：

1. 本机 CTest 回归。
2. ARM 固件重新编译。
3. 对应 QEMU/GDB 运行验证。
4. 必要的 PyBullet 联动验证。
5. 边界与异常场景检查。
6. 测试结果和对应 Git Commit 记录。

**最终目标是形成一个能够构建、运行、调试、测试并重复验证的六轴机器人运动控制固件项目，而不仅是一个能够在仿真中转动机械臂的演示程序。**
