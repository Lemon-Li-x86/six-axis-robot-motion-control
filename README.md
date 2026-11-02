# 六轴工业机器人嵌入式运动控制固件与仿真验证

**交付入口：先看本 README，再运行相应测试。** 历史设计、日志和分支快照参见 [HANDOFF_BOOK](HANDOFF_BOOK.md)，不得将其中旧分支当作最新代码。

本项目用 **C11 / ARM Cortex-M4 / FreeRTOS / QEMU UART / Python 3.11 / PyBullet** 验证 UR5 六轴关节控制。ARM 固件负责协议、FK/IK、轨迹、PID、限幅与任务调度；PyBullet 提供虚拟执行器和反馈，Windows 原生 GUI 用于操作者安全点动。

> **范围：** 这是仿真项目，不接实体电机；自动化 GUI 与 QEMU 验证不等于实体硬件安全认证，也不代替使用者的本机 GUI 操作签收。

## 1. 从哪里开始

- **只想确认代码能编译/算法正确：** 运行第 3 节的 Python 单测与本机 CTest。
- **想确认 ARM FreeRTOS + QEMU + PyBullet 真正闭环：** 运行第 4 节的 Windows ARM C APF 自动验收。
- **想打开 GUI 做 J6 安全点动：** 先通过 ARM 自动验收，再按第 5 节人工操作。
- **想看哪份测试对应哪个功能、如何判断 PASS：** 看第 6 节。
- **想找源码/测试文件：** 看第 8 节逐文件索引；参数所有权看第 7 节。

### 当前基线

- 默认分支：`main`。此前 Stage5 工作已合并，不再要求切到 `fix/stage5-verdict-docs-20261101`。
- 已完成的自动化基线：`6a25faab0860544db3c89454c48aacff80bc1ff7`，[Stage5 live QEMU PyBullet observer（13/13 jobs success）](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41399964643)；[ARM/原生固件回归（success）](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41399911610) 对应前一合并提交。
- 上述 PASS **仅属于对应提交**；请在 [Actions](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions) 核对自己检出的 HEAD。文档/注释整理分支的提交不能自动继承旧 CI 结论。
- Stage1–4 核心自动化通过；Stage5 指定仿真工况闭环通过；**Windows 本机真人 GUI 连续操作尚待签收**；Stage5 性能中的控制周期 P99 稳定改善未获证实；Stage6 搬运/抓取尚未完成。

## 2. 环境准备（Windows PowerShell，在仓库根目录操作）

### 工具与版本边界

| 组件 | 用途 | 安装/检查 |
|---|---|---|
| Git | 克隆与版本核对 | `git --version` |
| CMake >=3.20、Ninja | ARM 构建和部分原生测试 | `cmake --version`、`ninja --version` |
| ARM GNU Toolchain | `arm-none-eabi-gcc` 编译 Cortex-M4 | `arm-none-eabi-gcc --version` |
| QEMU | `qemu-system-arm` 模拟 MPS2-AN386 | `qemu-system-arm --version` |
| Python 3.11 | PyBullet 桥接、生成配置、测试 | `py -3.11 --version` |
| 本机 C/C++ 编译器 | CTest 和 C APF DLL | MSVC Build Tools 或 GCC/Clang；需匹配 CMake 生成器 |
| ARM GDB（Stage4/底层诊断） | 读取目标端断点/变量 | `arm-none-eabi-gdb --version` |
| 可显示窗口的 Windows 会话 | PyBullet **人工 GUI** | 远程无头环境不能代替操作者点击 |

```powershell
git clone https://github.com/Lemon-Li-x86/six-axis-robot-motion-control.git
cd six-axis-robot-motion-control
git switch main
git pull --ff-only
git log -1 --oneline

# CMake 需要此非仓库依赖；已有匹配版本的目录无需重克隆
git clone --depth 1 --branch V11.1.0 https://github.com/FreeRTOS/FreeRTOS-Kernel.git third_party/FreeRTOS-Kernel

py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -c "import numpy, pybullet; print(numpy.__version__)"
```

**依赖分离：** `requirements.txt` 只含已经在 Windows Stage5 CI 固定组合验证的 NumPy/PyBullet 运行依赖；`requirements-analysis.txt` 是 `tools/analyze_ik_batch.py` 的可选 Robotics Toolbox 分析栈（NumPy 1.x），请在**另一虚拟环境**中安装。CMake、ARM GCC、QEMU、FreeRTOS、MSVC/GDB 都不由 pip 安装。已有 Windows 本机旧虚拟环境若存在 CPython ABI 不一致，使用 `tools/stage5_existing_python.py` / `tools/resolve_stage5_python.ps1`，不要直接把旧 site-packages 强塞进不匹配解释器；详见 [人工验收指导](docs/part5/18_stage5_windows_final_two_step_acceptance.md)。

Linux CI 采用 `apt` 安装 QEMU/构建工具，并在各作业安装 NumPy/PyBullet；本地 Linux 的图形测试要使用 Xvfb。编译器、第三方依赖以 [CI workflow](.github/workflows/stage5-live-gui.yml) 和 [third_party/README](third_party/README.md) 为准。

## 3. 单元测试：最快确认基础功能

### Python：通信镜像、配置生成、GUI 逻辑

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_firmware_defaults.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_readme_links.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_repository_architecture.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_stage5_live_gui.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_stage5_gui_safe_jog.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_stage5_embedded_apf_runtime.py" -v
# 所有 tools/tests/test_*.py（包含更重的可选测试），见文件索引
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p "test_*.py" -v
```

**PASS：** 进程退出码为 0，unittest 输出 `OK`（必要时有 `skipped`；请记录跳过数量）。仅此不证明 QEMU 串口闭环或实体物理运动。

### 本机 CTest：无需 ARM/QEMU 的算法和协议测试

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/tests-host
cmake --build build/tests-host --config Debug
ctest --test-dir build/tests-host -C Debug --output-on-failure

cmake -S firmware/freertos_demo/tests/trajectory -B build/tests-trajectory
cmake --build build/tests-trajectory --config Debug
ctest --test-dir build/tests-trajectory -C Debug --output-on-failure

cmake -S firmware/freertos_demo/tests/avoidance -B build/tests-avoidance
cmake --build build/tests-avoidance --config Debug
ctest --test-dir build/tests-avoidance -C Debug --output-on-failure
```

**PASS：** CTest 输出 `100% tests passed` 且无失败。目录分别覆盖反馈/协议/IK 选解，轨迹/PID/限幅，APF/IK/FK 路线。额外的 `tests/simulation`、`tests/feedback_clock` 和 `tests/unity` 有独立 CMake 入口；Unity 源码需单独准备，详见 [third_party](third_party/README.md)。

### ARM 固件构建

```powershell
cmake --preset arm-qemu
cmake --build --preset firmware
# Stage5 真正 UART 闭环必须用其专用配置，不用普通构建代替：
cmake --preset stage5-live-uart
cmake --build --preset stage5-live-uart
```

**PASS：** 构建退出码为 0 且存在 `build/stage5-live-uart/firmware/freertos_demo/freertos_demo.elf`。**仅编译成功不等于运动控制测试通过**。

## 4. 自动完整闭环：ARM C APF → QEMU UART → PyBullet

以下命令在 Windows PowerShell 下会构建 ARM 固件并运行真实虚拟串口的完整静态障碍路线，**不需要打开 GUI 或点击鼠标**：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_embedded_apf_windows.ps1 `
  -Python .\.venv\Scripts\python.exe `
  -Qemu qemu-system-arm -TimeoutSeconds 300 `
  -OutputDir build/stage5-teacher-apf-check
```

**PASS 必须同时满足：** `result.json` 内 `passed=true`、`route_complete=true`、`full_link_safe=true`、`joint_motion_submitted_only_after_gate=true`、`uart_checksum_errors=0`、`runtime_full_link_queries >= 7*joint_motion_acks`，且至少三个新规划提案及三个运动 ACK。该脚本另生成 `build/stage5-teacher-apf-check.zip` 证据包；失败时保留同名目录日志。APF 提案不是运动许可，UART ACK 不是物理到位。

**局部快速诊断：** 将上条命令补上 `-ThreeStepOnly` 只检查 3 次提案，不可据此宣称整条路线 PASS。完整路线段数由场景/规划动态决定，不写死为 28、29 或 30。

另有 Stage4 一键回归（需 ARM GDB + QEMU + 本机 PyBullet）：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
```

## 5. Windows 人工 GUI 验收（老师/操作者）

**先运行第 4 节，确认自动完整路线 PASS。** 在 Windows 桌面会话打开 PyBullet 原生 GUI：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\run_stage5_live_windows.ps1 `
  -Python .\.venv\Scripts\python.exe `
  -Qemu qemu-system-arm -Stage5RouteCase blocked `
  -SafeJog -RequireTwoStepJog -SafeJogHoldSeconds 600 `
  -OutputDir build/stage5-teacher-gui-check
```

待路线所有段落物理到位并进入 HOLD，进行 **J6 +0.5° → 等待 RETURN COMPLETE → J6 +2° → 等待 RETURN COMPLETE → 再完成至少 3 次合法操作且每次独立回位**，最后点击 **SAFE JOG FINISH/EXIT**，不要直接关闭窗口。手动操作说明见 [Windows 人工验收指导](docs/part5/18_stage5_windows_final_two_step_acceptance.md)。

```powershell
.\.venv\Scripts\python.exe tools/check_stage5_safe_gui_acceptance.py build/stage5-teacher-gui-check/result.json --two-step
```

**PASS：** 脚本成功退出；要求 `passed=true`、物理段落全部 settled、所有操作均有 C/UART 回位、拒绝/校验错误与故障计数为零、`clock_alignment_passed=true`、末端 HOLD 精度满足现有阈值。**这项真人验收尚未完成，不能使用 CI 的 Xvfb 自动模拟点击替代签字。**

**Windows 已有环境特殊情况：** 若原虚拟环境内扩展是 CPython 3.11、解释器却是其他版本，可以在上述两个 PowerShell 入口显式传入 `-Python <CPython3.11路径> -ExistingSitePackages <原site-packages路径>`。先用 `tools/stage5_existing_python.py --site-packages ... --probe` 验证；参数必须与实际路径相符。

## 6. 测试矩阵与 PASS 含义

本表是 [Stage5 live workflow](.github/workflows/stage5-live-gui.yml) **在基线提交 `6a25faab` 的 13 个成功 job**。运行本地某条命令只证明对应范围，不等于 13 项全部完成。

| GitHub Actions job | 目标 | PASS 证据 |
|---|---|---|
| `live-ipc-unit-tests (ubuntu-latest)` | Linux Python IPC/GUI逻辑 | unittest 通过 |
| `live-ipc-unit-tests (windows-latest)` | Windows Python IPC/GUI逻辑 | unittest 通过 |
| `native-firmware-regression` | Host C 控制/反馈/协议 | CTest 通过 |
| `native-gui-no-drag-xvfb` | Linux 原生 GUI 三段自动运行 | `passed=true`、`segments_settled=3`、时钟一致 |
| `native-gui-scripted-j6-xvfb` | GUI J6 脚本点动 | 独立 `check_stage5_safe_gui_acceptance.py` 判定 |
| `native-gui-j6-two-step-xvfb` | GUI J6 +0.5/+2° 回位 | 验收脚本 `--two-step` |
| `native-gui-j6-five-repeat-xvfb` | GUI 五次点动和五次回位 | 验收脚本 `--repeat` |
| `embedded-apf-c-rtos-uart-bullet` | ARM FreeRTOS 内 APF/IK + 串口运动 + 全连杆检查 | 全路线 JSON PASS、逐段回位和 full-link queries |
| `windows-embedded-apf-first-leg-diagnostic` | Windows 真实 ARM 首段控制 | 运行及诊断步骤通过 |
| `genuine-qemu-uart-live-observer (ubuntu-latest)` | Linux QEMU UART 实测反馈/物理恢复 | 观察器和恢复测试退出码为 0 |
| `genuine-qemu-uart-live-observer (windows-latest)` | Windows 同上 | 观察器和恢复测试退出码为 0 |
| `windows-c-apf-bridge` | 生产 C APF/IK 的 Windows DLL | DLL 构建、导出符号和 CTest 通过 |
| `windows-controlled-obstacle-recovery` | Windows 障碍后保持与恢复 | C APF/UART 实体仿真日志 `passed=true` 与回位检查 |

**常见错误判读：** `build` 成功=生成了二进制；`unit test PASS`=局部输入与断言通过；`UART ACK`=固件已接受请求；`physical settled`=PyBullet 反馈满足稳定到位条件；`GUI acceptance PASS`=实际按脚本规定的动作、回位和完成退出都获独立校验。这些是五个不同层次。

**证据文件：** Stage5 运行脚本将 `result.json`、`worker.log` 等保存到给定 `-OutputDir`，部分任务会产出 ZIP；CI 原始作业日志/Artifacts 在 [Actions](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions)。每次使用**新的输出目录**，避免与上次结果混淆。失败优先查看第一个报错、`worker.log`、`result.json`，而不是删掉日志或放宽阈值。

## 7. 架构、调用方向、参数唯一来源

```text
UART IRQ -> Driver RingBuffer -> ProtocolRX -> Command Dispatch
                                           -> Motion Pipeline / Trajectory Task
Robot Model -> FK/IK -> Motion selector -> Trajectory reference
                             -> Control Task (PID + limiter)
                             -> virtual motor / UART -> PyBullet
PyBullet measured joints -> UART -> MotorState -> control/trajectory feedback
Monitor Task -> performance diagnostics
```

- `model/ur5_model.c` 是 **UR5 几何、坐标转换、关节限位** 的权威来源；别在 IK、Trajectory、GUI 中复制。
- `config/robot_defaults.json` 是 **固件默认 PID、任务优先级/栈/周期、速度/加速度和超时** 的唯一编辑点；`tools/generate_firmware_defaults.py` 生成 C 配置，`stage4_motion_config.c` 只做适配和派生阈值。
- `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json` 是当前静态障碍场景数据源；APF 参数由调用方传给生产 C 函数，测试夹具不构成生产配置。
- **依赖规则：** `algorithms/` 不依赖 FreeRTOS/UART/Motor Driver；`kinematics` 只解几何，`motion` 选解，`trajectory` 生成参考，`control` 闭环，`application` 调度与共享状态；`.h` 公开接口契约，`.c` 保留实现与并发原因。
- 角度单位：Canonical 角度为 0.01° 且属于 `[-180°,180°)`；连续位置也是 0.01°，可能跨圈；DH 长度/笛卡尔位置为 mm。碰撞检测以实际 URDF 和全连杆门禁为准。
- 变更参数之前查 [Stage5 参数登记](docs/part5/01_parameter_register.md)、[架构](docs/architecture.md) 和 [项目交接](HANDOFF_BOOK.md)；**不要在功能文件中新定义配置默认值**。

## 8. 仓库文件索引

以下索引覆盖仓库的代码、脚本、测试、工作流与文档。表格按**文件实际路径**生成；同目录下 `.c` 是实现，`.h` 是公共或内部接口；`*_test*` 是测试入口/夹具，不是生产功能。构建产物 `build/`、第三方克隆及本地 `.venv/` 不属于仓库。

### 根目录、构建与 CI（26 文件）

| 相对路径 | 职责/测试内容 |
|---|---|
| `.editorconfig` | C/H 等源文件缩进/换行约定 |
| `.github/workflows/native-motion-regression.yml` | GitHub Actions 自动测试工作流：native motion regression |
| `.github/workflows/stage4-closure-evidence.yml` | GitHub Actions 自动测试工作流：stage4 closure evidence |
| `.github/workflows/stage4-qemu-extended-soak.yml` | GitHub Actions 自动测试工作流：stage4 qemu extended soak |
| `.github/workflows/stage4-spec-acceptance.yml` | GitHub Actions 自动测试工作流：stage4 spec acceptance |
| `.github/workflows/stage4-trajectory-host.yml` | GitHub Actions 自动测试工作流：stage4 trajectory host |
| `.github/workflows/stage5-avoidance-uart.yml` | GitHub Actions 自动测试工作流：stage5 avoidance uart |
| `.github/workflows/stage5-cpu-trace.yml` | GitHub Actions 自动测试工作流：stage5 cpu trace |
| `.github/workflows/stage5-feedback-long-route.yml` | GitHub Actions 自动测试工作流：stage5 feedback long route |
| `.github/workflows/stage5-final-memory-comparison.yml` | GitHub Actions 自动测试工作流：stage5 final memory comparison |
| `.github/workflows/stage5-live-gui.yml` | GitHub Actions 自动测试工作流：stage5 live gui |
| `.github/workflows/stage5-memory-audit.yml` | GitHub Actions 自动测试工作流：stage5 memory audit |
| `.github/workflows/stage5-qemu-uart-pybullet.yml` | GitHub Actions 自动测试工作流：stage5 qemu uart pybullet |
| `.github/workflows/stage5-system-integration.yml` | GitHub Actions 自动测试工作流：stage5 system integration |
| `.github/workflows/stage5-visual-demo.yml` | GitHub Actions 自动测试工作流：stage5 visual demo |
| `.gitignore` | 忽略本地构建/日志/虚拟环境 |
| `CMakeLists.txt` | 当前目录 CMake 构建定义 |
| `CMakePresets.json` | ARM 和 Stage5 CMake 构建预设 |
| `HANDOFF_BOOK.md` | 历史开发接手与分支快照 |
| `README.md` | 当前目录说明/开发指导 |
| `cmake/arm-none-eabi-gcc.cmake` | ARM 交叉编译工具链定义 |
| `requirements-analysis.txt` | 可选 FK/IK 数值分析依赖 |
| `requirements.txt` | Stage5 NumPy/PyBullet 运行依赖 |
| `third_party/README.md` | 当前目录说明/开发指导 |
| `六轴工业机器人嵌入式运动控制固件开发与仿真验证.pdf` | 项目要求/设计原始 PDF |
| `评价.txt` | 项目评价记录 |

### 固件生产代码与配置（98 文件）

| 相对路径 | 职责/测试内容 |
|---|---|
| `firmware/README.md` | 当前目录说明/开发指导 |
| `firmware/freertos_demo/CMakeLists.txt` | 当前目录 CMake 构建定义 |
| `firmware/freertos_demo/FreeRTOSConfig.h` | RTOS 内核/中断配置 |
| `firmware/freertos_demo/README.md` | 当前目录说明/开发指导 |
| `firmware/freertos_demo/algorithms/avoidance/potential_field.c` | C APF 单步避障规划 |
| `firmware/freertos_demo/algorithms/avoidance/potential_field.h` | APF 基础配置/障碍输入接口 |
| `firmware/freertos_demo/algorithms/avoidance/potential_field_path.c` | 多步 APF 路线生成 |
| `firmware/freertos_demo/algorithms/avoidance/potential_field_path.h` | APF 路线接口 |
| `firmware/freertos_demo/algorithms/avoidance/stage5_apf_joint_path.c` | APF 航点转 IK 和梯形关节段 |
| `firmware/freertos_demo/algorithms/avoidance/stage5_apf_joint_path.h` | 关节路线转换接口 |
| `firmware/freertos_demo/algorithms/control/joint_command_limiter.c` | 位置/速度/加速度及量化门禁 |
| `firmware/freertos_demo/algorithms/control/joint_command_limiter.h` | 关节命令限幅接口 |
| `firmware/freertos_demo/algorithms/control/joint_controller.c` | 六轴关节闭环控制 |
| `firmware/freertos_demo/algorithms/control/joint_controller.h` | 控制器 API |
| `firmware/freertos_demo/algorithms/control/pid.c` | 增量式 PID 与抗饱和处理 |
| `firmware/freertos_demo/algorithms/control/pid.h` | PID API |
| `firmware/freertos_demo/algorithms/kinematics/kinematics.h` | FK/IK 公共接口和数值验收阈值 |
| `firmware/freertos_demo/algorithms/kinematics/kinematics_common.c` | 运动学共享换算函数 |
| `firmware/freertos_demo/algorithms/kinematics/kinematics_forward.c` | 正运动学 FK |
| `firmware/freertos_demo/algorithms/kinematics/kinematics_internal.h` | IK 内部计算接口 |
| `firmware/freertos_demo/algorithms/kinematics/kinematics_inverse.c` | 解析 IK 的组织、过滤、结果封装 |
| `firmware/freertos_demo/algorithms/kinematics/kinematics_private.h` | 私有运动学辅助声明 |
| `firmware/freertos_demo/algorithms/kinematics/ur5_analytic_ik.c` | UR5 解析逆解分支算法 |
| `firmware/freertos_demo/algorithms/math/matrix4.c` | 4×4 变换矩阵运算 |
| `firmware/freertos_demo/algorithms/math/matrix4.h` | 矩阵运算接口 |
| `firmware/freertos_demo/algorithms/motion/ik_solution_selector.c` | IK 候选解的运动策略筛选 |
| `firmware/freertos_demo/algorithms/motion/ik_solution_selector.h` | IK 选解接口 |
| `firmware/freertos_demo/algorithms/motion/motion_limits.c` | 连续角展开与关节限位 |
| `firmware/freertos_demo/algorithms/motion/motion_limits.h` | 运动限位接口 |
| `firmware/freertos_demo/algorithms/trajectory/cartesian_arc.c` | 笛卡尔圆弧插补 |
| `firmware/freertos_demo/algorithms/trajectory/cartesian_common.c` | 笛卡尔轨迹公共运算 |
| `firmware/freertos_demo/algorithms/trajectory/cartesian_internal.h` | 笛卡尔轨迹私有接口 |
| `firmware/freertos_demo/algorithms/trajectory/cartesian_line.c` | 笛卡尔直线插补 |
| `firmware/freertos_demo/algorithms/trajectory/cartesian_trajectory.h` | 笛卡尔轨迹公共接口 |
| `firmware/freertos_demo/algorithms/trajectory/cartesian_waypoint.c` | 笛卡尔航点与 IK 转换 |
| `firmware/freertos_demo/algorithms/trajectory/polynomial_trajectory.c` | 三/五次多项式插补 |
| `firmware/freertos_demo/algorithms/trajectory/trajectory.h` | 关节/多项式/梯形轨迹公共 API |
| `firmware/freertos_demo/algorithms/trajectory/trapezoidal_profile.c` | 梯形速度轨迹规划 |
| `firmware/freertos_demo/application/motor_state.c` | RTOS 互斥保护的测量/目标状态与时间戳 |
| `firmware/freertos_demo/application/motor_state.h` | 共享电机状态访问接口 |
| `firmware/freertos_demo/application/protocol_tasks.c` | ProtocolRX/ProtocolTX 任务与通知 |
| `firmware/freertos_demo/application/protocol_tasks.h` | 协议任务启动接口 |
| `firmware/freertos_demo/application/runtime_config.c` | 运行时可修改参数及其同步 |
| `firmware/freertos_demo/application/runtime_config.h` | 运行时配置接口 |
| `firmware/freertos_demo/application/services/motion_pipeline.c` | 运动请求队列、任务启动、诊断与故障记录 |
| `firmware/freertos_demo/application/services/motion_pipeline.h` | 运动服务对外请求/配置接口 |
| `firmware/freertos_demo/application/services/protocol_commands.c` | 协议业务命令分发与回复 |
| `firmware/freertos_demo/application/services/protocol_commands.h` | 协议命令服务接口 |
| `firmware/freertos_demo/application/services/stage5_apf_proposal.c` | 测量姿态到 APF/IK 关节提案 |
| `firmware/freertos_demo/application/services/stage5_apf_proposal.h` | APF 提案接口与契约 |
| `firmware/freertos_demo/application/services/trajectory_request_adapter.c` | 运动请求到轨迹参数适配 |
| `firmware/freertos_demo/application/services/trajectory_request_adapter.h` | 轨迹请求适配接口 |
| `firmware/freertos_demo/application/task_config.h` | 任务优先级/周期/栈的生成配置别名 |
| `firmware/freertos_demo/application/tasks/control_task.c` | FreeRTOS 控制任务、PID 和限幅驱动流程 |
| `firmware/freertos_demo/application/tasks/monitor_task.c` | FreeRTOS 监测任务 |
| `firmware/freertos_demo/application/tasks/motion_tasks_internal.h` | 运动任务共享的内部状态/队列契约 |
| `firmware/freertos_demo/application/tasks/stage4_timing_probe.c` | Stage4 周期及执行时长探针 |
| `firmware/freertos_demo/application/tasks/stage4_timing_probe.h` | 探针接口 |
| `firmware/freertos_demo/application/tasks/trajectory_task.c` | FreeRTOS 轨迹任务、请求更新与 APF 提案 |
| `firmware/freertos_demo/application/uart_tx_manager.c` | 串口完整帧发送互斥 |
| `firmware/freertos_demo/application/uart_tx_manager.h` | 串口帧发送接口 |
| `firmware/freertos_demo/board.c` | 板级外设初始化 |
| `firmware/freertos_demo/board.h` | 板级接口和时钟定义 |
| `firmware/freertos_demo/common/angle_utils.c` | 角度规范化/连续角转换实现 |
| `firmware/freertos_demo/common/angle_utils.h` | 角度转换接口 |
| `firmware/freertos_demo/common/error_code.h` | 统一状态码 |
| `firmware/freertos_demo/common/math_constants.h` | 公共数学常量 |
| `firmware/freertos_demo/common/robot_types.h` | 关节/姿态/变换矩阵公用类型及单位 |
| `firmware/freertos_demo/communications/protocol.c` | C UART 协议编解码、状态机和校验 |
| `firmware/freertos_demo/communications/protocol.h` | 协议帧/指令/字段接口 |
| `firmware/freertos_demo/communications/ring_buffer.c` | 收发环形缓冲操作 |
| `firmware/freertos_demo/communications/ring_buffer.h` | RingBuffer 接口 |
| `firmware/freertos_demo/config/robot_defaults.json` | robot defaults |
| `firmware/freertos_demo/config/stage4_motion_config.c` | 生成默认值与派生运动配置的适配 |
| `firmware/freertos_demo/config/stage4_motion_config.h` | 默认运动配置的只读访问接口 |
| `firmware/freertos_demo/diagnostics/performance_monitor.c` | 任务与协议运行性能监测 |
| `firmware/freertos_demo/diagnostics/performance_monitor.h` | 性能采样接口 |
| `firmware/freertos_demo/diagnostics/stage5_cpu_trace.c` | Stage5 CPU/任务跟踪采样 |
| `firmware/freertos_demo/diagnostics/stage5_cpu_trace.h` | CPU 跟踪接口 |
| `firmware/freertos_demo/drivers/gpio_driver.c` | GPIO 驱动/仿真影子寄存器 |
| `firmware/freertos_demo/drivers/gpio_driver.h` | GPIO 接口 |
| `firmware/freertos_demo/drivers/motor_driver.c` | 虚拟电机目标/反馈、角度状态底层管理 |
| `firmware/freertos_demo/drivers/motor_driver.h` | 电机状态接口 |
| `firmware/freertos_demo/drivers/timer_driver.c` | 定时器驱动 |
| `firmware/freertos_demo/drivers/timer_driver.h` | 定时器接口 |
| `firmware/freertos_demo/drivers/uart_driver.c` | UART MMIO、ISR 和字节收发 |
| `firmware/freertos_demo/drivers/uart_driver.h` | UART 驱动接口 |
| `firmware/freertos_demo/linker.ld` | 目标内存布局 |
| `firmware/freertos_demo/main.c` | 板级初始化、任务创建及启动调度器 |
| `firmware/freertos_demo/memory.c` | 无标准库运行环境内存例程 |
| `firmware/freertos_demo/model/robot_model.h` | 机器人静态模型访问接口 |
| `firmware/freertos_demo/model/ur5_model.c` | UR5 DH、坐标变换、关节限位数据 |
| `firmware/freertos_demo/platform/freestanding_errno.c` | 裸机 errno 兼容 |
| `firmware/freertos_demo/startup.s` | Cortex-M4 启动和向量表 |
| `firmware/hello/linker.ld` | 目标内存布局 |
| `firmware/hello/main.c` | 板级初始化、任务创建及启动调度器 |
| `firmware/hello/rd1.md` | rd1 |
| `firmware/hello/startup.s` | Cortex-M4 启动和向量表 |

### 固件 C/QEMU/Unity 测试（59 文件）

| 相对路径 | 职责/测试内容 |
|---|---|
| `firmware/freertos_demo/tests/avoidance/CMakeLists.txt` | 此测试套件的 CMake/CTest 目标注册 |
| `firmware/freertos_demo/tests/avoidance/potential_field_path_test_main.c` | 回归测试或夹具：potential field path |
| `firmware/freertos_demo/tests/avoidance/potential_field_test_main.c` | 回归测试或夹具：potential field |
| `firmware/freertos_demo/tests/avoidance/stage5_apf_joint_path_test_main.c` | 回归测试或夹具：stage5 apf joint path |
| `firmware/freertos_demo/tests/feedback_clock/CMakeLists.txt` | 此测试套件的 CMake/CTest 目标注册 |
| `firmware/freertos_demo/tests/feedback_clock/feedback_tick_order_test.c` | 回归测试或夹具：feedback tick order |
| `firmware/freertos_demo/tests/feedback_clock/stubs/FreeRTOS.h` | 测试共享声明/夹具：FreeRTOS |
| `firmware/freertos_demo/tests/feedback_clock/stubs/semphr.h` | 测试共享声明/夹具：semphr |
| `firmware/freertos_demo/tests/feedback_clock/stubs/task.h` | 测试共享声明/夹具：task |
| `firmware/freertos_demo/tests/host/CMakeLists.txt` | 此测试套件的 CMake/CTest 目标注册 |
| `firmware/freertos_demo/tests/host/README.md` | 回归测试或夹具：README |
| `firmware/freertos_demo/tests/host/motor_feedback_safety_test.c` | 回归测试或夹具：motor feedback safety |
| `firmware/freertos_demo/tests/host/protocol_stream_fault_test.c` | 回归测试或夹具：protocol stream fault |
| `firmware/freertos_demo/tests/host/stage5_apf_wire_test.c` | 回归测试或夹具：stage5 apf wire |
| `firmware/freertos_demo/tests/kinematics/ik_reference_test_limits.h` | 测试共享声明/夹具：ik reference test limits |
| `firmware/freertos_demo/tests/kinematics/kinematics_dedup_test.c` | 回归测试或夹具：kinematics dedup |
| `firmware/freertos_demo/tests/kinematics/kinematics_dedup_test.h` | 测试共享声明/夹具：kinematics dedup |
| `firmware/freertos_demo/tests/kinematics/kinematics_fk_test.c` | 回归测试或夹具：kinematics fk |
| `firmware/freertos_demo/tests/kinematics/kinematics_fk_test.h` | 测试共享声明/夹具：kinematics fk |
| `firmware/freertos_demo/tests/kinematics/kinematics_ik_solver_test.c` | 回归测试或夹具：kinematics ik solver |
| `firmware/freertos_demo/tests/kinematics/kinematics_ik_solver_test.h` | 测试共享声明/夹具：kinematics ik solver |
| `firmware/freertos_demo/tests/kinematics/kinematics_ik_test.c` | 回归测试或夹具：kinematics ik |
| `firmware/freertos_demo/tests/kinematics/kinematics_ik_test.h` | 测试共享声明/夹具：kinematics ik |
| `firmware/freertos_demo/tests/kinematics/kinematics_self_test.c` | 回归测试或夹具：kinematics self |
| `firmware/freertos_demo/tests/kinematics/kinematics_self_test.h` | 测试共享声明/夹具：kinematics self |
| `firmware/freertos_demo/tests/kinematics/kinematics_test_support.c` | 回归测试或夹具：kinematics test support |
| `firmware/freertos_demo/tests/kinematics/kinematics_test_support.h` | 测试共享声明/夹具：kinematics test support |
| `firmware/freertos_demo/tests/model/robot_model_limits_test.c` | 回归测试或夹具：robot model limits |
| `firmware/freertos_demo/tests/motion/ik_solution_selector_test_main.c` | 回归测试或夹具：ik solution selector |
| `firmware/freertos_demo/tests/motion/motion_limits_test_main.c` | 回归测试或夹具：motion limits |
| `firmware/freertos_demo/tests/qemu_target/fk_batch_test_main.c` | 回归测试或夹具：fk batch |
| `firmware/freertos_demo/tests/qemu_target/gpio_shadow_test_main.c` | 回归测试或夹具：gpio shadow |
| `firmware/freertos_demo/tests/qemu_target/ik_batch_test_main.c` | 回归测试或夹具：ik batch |
| `firmware/freertos_demo/tests/qemu_target/ik_near_singular_trace_main.c` | 回归测试或夹具：ik near singular trace main |
| `firmware/freertos_demo/tests/qemu_target/kinematics_perf_test_main.c` | 回归测试或夹具：kinematics perf |
| `firmware/freertos_demo/tests/qemu_target/motor_test_main.c` | 回归测试或夹具：motor |
| `firmware/freertos_demo/tests/qemu_target/protocol_test_main.c` | 回归测试或夹具：protocol |
| `firmware/freertos_demo/tests/qemu_target/stage4_rtos_selftest.c` | 回归测试或夹具：stage4 rtos selftest |
| `firmware/freertos_demo/tests/qemu_target/stage4_trajectory_test_main.c` | 回归测试或夹具：stage4 trajectory |
| `firmware/freertos_demo/tests/qemu_target/test_startup.s` | 目标端测试启动代码 |
| `firmware/freertos_demo/tests/qemu_target/uart_tx_boundary_main.c` | 回归测试或夹具：uart tx boundary main |
| `firmware/freertos_demo/tests/simulation/CMakeLists.txt` | 此测试套件的 CMake/CTest 目标注册 |
| `firmware/freertos_demo/tests/simulation/stage4_sim_bridge.c` | 回归测试或夹具：stage4 sim bridge |
| `firmware/freertos_demo/tests/simulation/stage4_sim_bridge_smoke.c` | 回归测试或夹具：stage4 sim bridge smoke |
| `firmware/freertos_demo/tests/trajectory/CMakeLists.txt` | 此测试套件的 CMake/CTest 目标注册 |
| `firmware/freertos_demo/tests/trajectory/cartesian_control_test_main.c` | 回归测试或夹具：cartesian control |
| `firmware/freertos_demo/tests/trajectory/cartesian_ik_test_main.c` | 回归测试或夹具：cartesian ik |
| `firmware/freertos_demo/tests/trajectory/command_limiter_test_main.c` | 回归测试或夹具：command limiter |
| `firmware/freertos_demo/tests/trajectory/ik_reference_regression_test_main.c` | 回归测试或夹具：ik reference regression |
| `firmware/freertos_demo/tests/trajectory/trajectory_adapter_test_main.c` | 回归测试或夹具：trajectory adapter |
| `firmware/freertos_demo/tests/trajectory/trajectory_test_main.c` | 回归测试或夹具：trajectory |
| `firmware/freertos_demo/tests/unity/CMakeLists.txt` | 此测试套件的 CMake/CTest 目标注册 |
| `firmware/freertos_demo/tests/unity/test_main.c` | 回归测试或夹具：test main |
| `firmware/freertos_demo/tests/unity/test_motor_driver.c` | 回归测试或夹具：test motor driver |
| `firmware/freertos_demo/tests/unity/test_protocol.c` | 回归测试或夹具：test protocol |
| `firmware/freertos_demo/tests/unity/test_ring_buffer.c` | 回归测试或夹具：test ring buffer |
| `firmware/freertos_demo/tests/unity/test_uart_driver.c` | 回归测试或夹具：test uart driver |
| `firmware/freertos_demo/tests/unity/uart_driver_test_backend.c` | 回归测试或夹具：uart driver test backend |
| `firmware/freertos_demo/tests/unity/uart_driver_test_backend.h` | 测试共享声明/夹具：uart driver test backend |

### PyBullet 与 UART 仿真（47 文件）

| 相对路径 | 职责/测试内容 |
|---|---|
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/base.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/forearm.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/shoulder.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/upperarm.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/wrist1.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/wrist2.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/collision/wrist3.stl` | UR5 碰撞连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/base.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/forearm.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/shoulder.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/upperarm.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/wrist1.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/wrist2.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/meshes/ur5/visual/wrist3.dae` | UR5 可视化连杆网格资源 |
| `simulation/pybullet_ur5/models/ur5/urdf/ur5.urdf` | UR5 连杆、关节和物理模型 |
| `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json` | Stage5 静态障碍场景及策略输入 |
| `simulation/pybullet_ur5/src/fk_cross_validation.py` | PyBullet FK 对照 |
| `simulation/pybullet_ur5/src/hello_pybullet.py` | PyBullet 环境冒烟测试 |
| `simulation/pybullet_ur5/src/qemu_clock_gate.py` | QEMU/物理时钟同步门禁 |
| `simulation/pybullet_ur5/src/qemu_uart_receiver.py` | QEMU UART 帧读取与诊断 |
| `simulation/pybullet_ur5/src/scene_demo.py` | 示例场景 |
| `simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py` | Stage4 动态闭环仿真 |
| `simulation/pybullet_ur5/src/stage5_avoidance_preflight.py` | APF 路线安全预检 |
| `simulation/pybullet_ur5/src/stage5_avoidance_uart_gate.py` | 避障路线到 UART 指令的门禁 |
| `simulation/pybullet_ur5/src/stage5_cartesian_shortcut.py` | 笛卡尔捷径与路线校验 |
| `simulation/pybullet_ur5/src/stage5_collision_validation.py` | PyBullet 连杆碰撞/距离验证 |
| `simulation/pybullet_ur5/src/stage5_obstacle_hold_recovery.py` | 障碍后 HOLD 恢复仿真 |
| `simulation/pybullet_ur5/src/stage5_qemu_uart_closed_loop.py` | Stage5 串口与物理闭环基础逻辑 |
| `simulation/pybullet_ur5/src/stage5_qemu_uart_live_gui.py` | Stage5 QEMU UART 物理闭环/原生 GUI 启动入口 |
| `simulation/pybullet_ur5/src/stage5_qemu_uart_trace_replay.py` | UART/物理追踪回放 |
| `simulation/pybullet_ur5/src/stage5_qemu_uart_visual_demo.py` | Stage5 可视化会话编排 |
| `simulation/pybullet_ur5/src/stage5_safe_gui_jog.py` | GUI 安全点动策略、动作/回位门禁 |
| `simulation/pybullet_ur5/src/stage5_static_obstacle_probe.py` | 静态障碍探测与 C APF 调用 |
| `simulation/pybullet_ur5/src/ur5_demo.py` | UR5 手动可视化示例 |
| `simulation/pybullet_ur5/tests/test_stage5_avoidance_preflight.py` | 回归测试或夹具：test stage5 avoidance preflight |
| `simulation/pybullet_ur5/tests/test_stage5_collision_validation.py` | 回归测试或夹具：test stage5 collision validation |
| `simulation/uart_bridge/angle_utils.py` | Python 角度转换 |
| `simulation/uart_bridge/burst_stress_test.py` | 仿真脚本：burst stress |
| `simulation/uart_bridge/communication_soak_test.py` | 仿真脚本：communication soak |
| `simulation/uart_bridge/communication_stress_test.py` | 仿真脚本：communication stress |
| `simulation/uart_bridge/internal_performance_metrics.py` | 仿真脚本：internal performance metrics |
| `simulation/uart_bridge/parameter_config_test.py` | 仿真脚本：parameter config |
| `simulation/uart_bridge/performance_baseline.py` | 仿真脚本：performance baseline |
| `simulation/uart_bridge/protocol_codec.py` | Python 协议编解码镜像 |
| `simulation/uart_bridge/protocol_fault_test.py` | 仿真脚本：protocol fault |
| `simulation/uart_bridge/uart_receiver.py` | 仿真脚本：uart receiver |
| `simulation/uart_bridge/ur5_uart_bridge.py` | 通用 UART↔PyBullet 桥接 |

### 测试/构建/诊断脚本（53 文件）

| 相对路径 | 职责/测试内容 |
|---|---|
| `tools/aggregate_stage5_perf_repeats.py` | 性能重复试验聚合 |
| `tools/analyze_fk_batch.py` | FK 批量结果对照 |
| `tools/analyze_ik_batch.py` | IK Robotics Toolbox 数值对照 |
| `tools/analyze_stage4_timing.py` | Stage4 任务时序统计 |
| `tools/analyze_stage5_cpu_trace.py` | Stage5 CPU Trace 分析 |
| `tools/analyze_stage5_task_timing.py` | Stage5 任务时序分析 |
| `tools/check_protocol_mirror.py` | C/Python 协议字段镜像一致性检查 |
| `tools/check_stage5_safe_gui_acceptance.py` | 从 result.json 独立裁决真实 GUI 验收 |
| `tools/ci_stage2_driver_qemu.sh` | 工具脚本：ci stage2 driver qemu |
| `tools/ci_stage2_protocol_qemu.sh` | 工具脚本：ci stage2 protocol qemu |
| `tools/ci_stage4_algorithm_qemu.sh` | 工具脚本：ci stage4 algorithm qemu |
| `tools/ci_stage4_rtos_qemu.sh` | 工具脚本：ci stage4 rtos qemu |
| `tools/ci_stage5_cpu_trace_qemu.sh` | 工具脚本：ci stage5 cpu trace qemu |
| `tools/compare_stage5_perf_ab.py` | Stage5 性能 A/B 对照 |
| `tools/generate_firmware_defaults.py` | JSON 参数校验并生成 C 头文件 |
| `tools/resolve_stage5_python.ps1` | 定位可用的 Python/PyBullet 解释器 |
| `tools/run_stage5_controlled_recovery_windows.ps1` | Windows 障碍恢复一键测试 |
| `tools/run_stage5_embedded_apf_windows.ps1` | Windows ARM 内 APF 全路线一键验收 |
| `tools/run_stage5_first_leg_native_bullet_windows.ps1` | 首段 C/PyBullet 控制复现 |
| `tools/run_stage5_live_windows.ps1` | Windows 原生 GUI 一键启动/验收 |
| `tools/run_stage5_perf_repeats.sh` | 工具脚本：run stage5 perf repeats |
| `tools/stage4_pid_gain_sweep.py` | PID 参数扫描研究 |
| `tools/stage4_pid_study.py` | PID 工况研究 |
| `tools/stage5_existing_python.py` | 已有 Windows Python ABI 导入兼容启动器 |
| `tools/stage5_memory_map.py` | 固件内存 map 解析 |
| `tools/test_stage4_closure.ps1` | Stage4 PID/QEMU 闭环扩展回归 |
| `tools/test_stage4_once.ps1` | Stage1–4 一键回归 |
| `tools/tests/run_stage5_controlled_apf_hold_recovery.py` | 受控避障/HOLD 回位测试入口 |
| `tools/tests/run_stage5_embedded_apf_uart.py` | ARM APF UART 全路线测试入口 |
| `tools/tests/run_stage5_first_leg_native_bullet.py` | 首段原生 C/Bullet 对照测试 |
| `tools/tests/test_firmware_defaults.py` | 回归测试或夹具：test firmware defaults |
| `tools/tests/test_readme_links.py` | 回归测试或夹具：test readme links |
| `tools/tests/test_repository_architecture.py` | 静态回归：算法层依赖方向、生成配置和 UART/FreeRTOS 共享 NVIC 优先级 |
| `tools/tests/test_stage4_pid_gain_sweep.py` | 回归测试或夹具：test stage4 pid gain sweep |
| `tools/tests/test_stage4_pid_study.py` | 回归测试或夹具：test stage4 pid study |
| `tools/tests/test_stage4_timing.py` | 回归测试或夹具：test stage4 timing |
| `tools/tests/test_stage5_auto_bypass_selection.py` | 回归测试或夹具：test stage5 auto bypass selection |
| `tools/tests/test_stage5_embedded_apf_runtime.py` | 回归测试或夹具：test stage5 embedded apf runtime |
| `tools/tests/test_stage5_existing_python.py` | 回归测试或夹具：test stage5 existing python |
| `tools/tests/test_stage5_gui_acceptance.py` | 回归测试或夹具：test stage5 gui acceptance |
| `tools/tests/test_stage5_gui_apf_integration.py` | 回归测试或夹具：test stage5 gui apf integration |
| `tools/tests/test_stage5_gui_avoidance_visibility.py` | 回归测试或夹具：test stage5 gui avoidance visibility |
| `tools/tests/test_stage5_gui_joint_travel.py` | 回归测试或夹具：test stage5 gui joint travel |
| `tools/tests/test_stage5_gui_safe_jog.py` | 回归测试或夹具：test stage5 gui safe jog |
| `tools/tests/test_stage5_live_gui.py` | 回归测试或夹具：test stage5 live gui |
| `tools/tests/test_stage5_memory_map.py` | 回归测试或夹具：test stage5 memory map |
| `tools/tests/test_stage5_native_joint_stops.py` | 回归测试或夹具：test stage5 native joint stops |
| `tools/tests/test_stage5_obstacle_hold_recovery.py` | 回归测试或夹具：test stage5 obstacle hold recovery |
| `tools/tests/test_stage5_perf_repeats.py` | 回归测试或夹具：test stage5 perf repeats |
| `tools/tests/test_stage5_scene_catalog.py` | 回归测试或夹具：test stage5 scene catalog |
| `tools/tests/test_stage5_task_timing.py` | 回归测试或夹具：test stage5 task timing |
| `tools/tests/test_stage5_trace_replay.py` | 回归测试或夹具：test stage5 trace replay |
| `tools/tests/test_stage5_visual_demo.py` | 回归测试或夹具：test stage5 visual demo |

### 设计说明、阶段报告与历史验收证据（62 文件）

| 相对路径 | 职责/测试内容 |
|---|---|
| `docs/architecture.md` | 设计/实验/验收文档：architecture |
| `docs/cortex_m4_notes.md` | 设计/实验/验收文档：cortex m4 notes |
| `docs/development.md` | 设计/实验/验收文档：development |
| `docs/part1/01_system_design.md` | 设计/实验/验收文档：01 system design |
| `docs/part1/04_env.md` | 设计/实验/验收文档：04 env |
| `docs/part2/02_driver_interface_and_module_description.md` | 设计/实验/验收文档：02 driver interface and module description |
| `docs/part2/03_unit_and_function_test_report.md` | 设计/实验/验收文档：03 unit and function test report |
| `docs/part3/01_kinematics_derivation.md` | 设计/实验/验收文档：01 kinematics derivation |
| `docs/part3/02_algorithm_validation_report.md` | 设计/实验/验收文档：02 algorithm validation report |
| `docs/part4/02_pid_tuning_and_trajectory_performance.md` | 设计/实验/验收文档：02 pid tuning and trajectory performance |
| `docs/part4/03_multitask_scheduling_and_realtime_verification.md` | 设计/实验/验收文档：03 multitask scheduling and realtime verification |
| `docs/part4/part4_archive/00_parameter_register.md` | 设计/实验/验收文档：00 parameter register |
| `docs/part4/part4_archive/01_joint_trajectory_interface.md` | 设计/实验/验收文档：01 joint trajectory interface |
| `docs/part4/part4_archive/02_stage4_full_implementation_guide.md` | 设计/实验/验收文档：02 stage4 full implementation guide |
| `docs/part4/part4_archive/03_pid_tuning_record.md` | 设计/实验/验收文档：03 pid tuning record |
| `docs/part4/part4_archive/04_realtime_verification.md` | 设计/实验/验收文档：04 realtime verification |
| `docs/part4/part4_archive/05_global_architecture_review_2026-10-26.md` | 设计/实验/验收文档：05 global architecture review 2026-10-26 |
| `docs/part4/part4_archive/06_stage4_requirements_verification_2026-10-27.md` | 设计/实验/验收文档：06 stage4 requirements verification 2026-10-27 |
| `docs/part4/part4_archive/07_stage4_reverification_2026-10-27.md` | 设计/实验/验收文档：07 stage4 reverification 2026-10-27 |
| `docs/part4/part4_archive/08_stage4_qemu_timing_and_stack_2026-10-27.md` | 设计/实验/验收文档：08 stage4 qemu timing and stack 2026-10-27 |
| `docs/part4/part4_archive/09_polynomial_preflight_and_json_config_2026-10-27.md` | 设计/实验/验收文档：09 polynomial preflight and json config 2026-10-27 |
| `docs/part4/part4_archive/10_stage4_pid_sensitivity_study.md` | 设计/实验/验收文档：10 stage4 pid sensitivity study |
| `docs/part4/part4_archive/11_stage4_pid_candidate_verification_2026-10-27.md` | 设计/实验/验收文档：11 stage4 pid candidate verification 2026-10-27 |
| `docs/part4/part4_archive/12_stage4_release_candidate_2026-10-28.md` | 设计/实验/验收文档：12 stage4 release candidate 2026-10-28 |
| `docs/part5/00_stage5_scope_and_acceptance.md` | 设计/实验/验收文档：00 stage5 scope and acceptance |
| `docs/part5/01_parameter_register.md` | 设计/实验/验收文档：01 parameter register |
| `docs/part5/02_integration_issue_log.md` | 设计/实验/验收文档：02 integration issue log |
| `docs/part5/03_linker_memory_audit.md` | 设计/实验/验收文档：03 linker memory audit |
| `docs/part5/03_uart_qemu_pybullet_contract.md` | 设计/实验/验收文档：03 uart qemu pybullet contract |
| `docs/part5/04_arm_link_memory_baseline_2026-10-28.md` | 设计/实验/验收文档：04 arm link memory baseline 2026-10-28 |
| `docs/part5/04_local_uart_remediation_2026-10-27.md` | 设计/实验/验收文档：04 local uart remediation 2026-10-27 |
| `docs/part5/05_apf_route_and_full_link_collision_gate.md` | 设计/实验/验收文档：05 apf route and full link collision gate |
| `docs/part5/05_qemu_pybullet_visual_demo.md` | 设计/实验/验收文档：05 qemu pybullet visual demo |
| `docs/part5/06_live_uart_gui.md` | 设计/实验/验收文档：06 live uart gui |
| `docs/part5/08_freeRTOS_cpu_trace_baseline.md` | 设计/实验/验收文档：08 freeRTOS cpu trace baseline |
| `docs/part5/09_apf_ik_joint_route_bridge.md` | 设计/实验/验收文档：09 apf ik joint route bridge |
| `docs/part5/10_c_apf_uart_live_execution.md` | 设计/实验/验收文档：10 c apf uart live execution |
| `docs/part5/13_safe_cartesian_route_shortcut.md` | 设计/实验/验收文档：13 safe cartesian route shortcut |
| `docs/part5/14_motion_feedback_timestamp_ordering.md` | 设计/实验/验收文档：14 motion feedback timestamp ordering |
| `docs/part5/15_stage5_exit_review_2026-10-30.md` | 设计/实验/验收文档：15 stage5 exit review 2026-10-30 |
| `docs/part5/16_restored_gui_baseline_integration_2026-10-30.md` | 设计/实验/验收文档：16 restored gui baseline integration 2026-10-30 |
| `docs/part5/16_safe_jog_click_review_2026-10-31.md` | 设计/实验/验收文档：16 safe jog click review 2026-10-31 |
| `docs/part5/17_stage5_final_acceptance_2026-10-31.md` | 设计/实验/验收文档：17 stage5 final acceptance 2026-10-31 |
| `docs/part5/18_stage5_windows_final_two_step_acceptance.md` | 设计/实验/验收文档：18 stage5 windows final two step acceptance |
| `docs/part5/19_stage5_original_requirements_evidence_review_2026-11-01.md` | 设计/实验/验收文档：19 stage5 original requirements evidence review 2026-11-01 |
| `docs/part5/20_stage5_5_3_repeated_performance_evidence_2026-11-01.md` | 设计/实验/验收文档：20 stage5 5 3 repeated performance evidence 2026-11-01 |
| `docs/part6/00_stage6_interface_and_acceptance_contract.md` | 设计/实验/验收文档：00 stage6 interface and acceptance contract |
| `docs/phase1-4_acceptance_status_2026-10-27.md` | 设计/实验/验收文档：phase1-4 acceptance status 2026-10-27 |
| `docs/project_plan_progress_audit_2026-10-31.md` | 设计/实验/验收文档：project plan progress audit 2026-10-31 |
| `docs/protocol.md` | 设计/实验/验收文档：protocol |
| `docs/system_design.md` | 设计/实验/验收文档：system design |
| `docs/testing/results/communication_soak_20261004_082640.json` | 设计/实验/验收文档：communication soak 20261004 082640 |
| `docs/testing/results/communication_soak_20261004_083759.json` | 设计/实验/验收文档：communication soak 20261004 083759 |
| `docs/testing/results/communication_soak_20261004_153439.json` | 设计/实验/验收文档：communication soak 20261004 153439 |
| `docs/testing/results/communication_soak_20261005_081607.json` | 设计/实验/验收文档：communication soak 20261005 081607 |
| `docs/testing/results/stage3_fk_validation.txt` | 设计/实验/验收文档：stage3 fk validation |
| `docs/testing/results/stage3_host_ctest.txt` | 设计/实验/验收文档：stage3 host ctest |
| `docs/testing/results/stage3_stack_usage.txt` | 设计/实验/验收文档：stage3 stack usage |
| `docs/weeklyReports/w1.pdf` | 设计/实验/验收文档：w1.pdf |
| `docs/weeklyReports/w2.pdf` | 设计/实验/验收文档：w2.pdf |
| `docs/weeklyReports/w3.pdf` | 设计/实验/验收文档：w3.pdf |
| `docs/weeklyReports/w4.pdf` | 设计/实验/验收文档：w4.pdf |

## 9. 贡献与审查约定

1. 先检查 `git status`、当前分支、HEAD 和 [参数登记](docs/part5/01_parameter_register.md)；在工作分支做小提交，不覆盖历史验收或备份。
2. 新功能先判断数据归属：**模型固有数据 / 算法中间量 / 运行时反馈 / 应用策略**。禁止重新定义同义参数；测试数据不得回写生产默认值。
3. 新公开接口在 `.h` 写 Doxygen `@brief/@param/@return/@note`、单位和调用语境；`.c` 注释解释非显而易见的算法、边界、RTOS 并发原因。删重复的“安全宣言”，不删真正的校验逻辑。
4. 控制、状态、UART、FreeRTOS 的调用方向不得倒置；禁止算法层直接读写硬件或全局电机运行态。
5. 至少重跑受影响的本机单测、目标构建和相应 QEMU/GUI 测试；**提交的 CI/证据只归属于该提交**。用户手动 GUI 验收另行记录。

完整设计背景参见 [架构说明](docs/architecture.md)、[参数登记簿](docs/part5/01_parameter_register.md)、[Stage5 人工 GUI 指南](docs/part5/18_stage5_windows_final_two_step_acceptance.md)。
