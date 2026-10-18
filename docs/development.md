# 开发、构建与文件管理规范

## 1. 单一构建入口

CMake 是固件及 QEMU 测试唯一正式构建定义，不再单独维护 VS Code
手写 GCC 源码清单。当前仓库提供 `CMakePresets.json`。

```powershell
cmake --preset arm-qemu
cmake --build --preset firmware
cmake --build --preset arm-qemu
```

切换启动自测时使用独立构建目录或显式重新配置；
`ROBOT_ENABLE_KINEMATICS_SELF_TEST` 和
`ROBOT_ENABLE_PARSER_BENCHMARK` 是 CMake 缓存选项，不是运行时开关。
默认开启以保留当前开发行为。

构建生成的头文件、ELF、运行日志等存入 `build/`；
不要覆盖配置源文件或手动编辑生成结果。

## 2. 新功能最小交付检查表

- [ ] 确认模块归属、唯一数据所有者及允许的依赖方向
- [ ] 定义输入输出、角度/速度单位、边界、错误码和生命周期
- [ ] 完成实现，不复制现有解析器、模型或算法
- [ ] 增加正常、边界、异常输入测试
- [ ] 验证对应 CMake 目标构建和 QEMU/仿真回归
- [ ] 更新公开接口或运行步骤相关文档
- [ ] 检查生成文件、临时结果、无关调试代码及 Git 差异

仅成功构建不能视为通信、控制或仿真功能验证通过。

## 3. 文件拆分准入规则

默认保持已有文件和目录不动。只有在以下情况才考虑拆分：

1. 有明确不同的责任、变化原因或生命周期；
2. 拆分后依赖方向更清晰，且接口成本合理；
3. 可以独立测试而非只是将代码搬到另一个文件；
4. 已确认所有调用点、状态所有权和链接目标均可一起迁移。

拆分前写明：旧职责、新职责、接口变化、保留行为、回归命令。
不按固定行数拆分，不为计划中的 Trajectory/Control/Planning
建立空壳，也不为了目录对称搬运已有文件。

## 4. 接口与并发

- `common/` 和纯算法对 FreeRTOS 解耦。
- 规范角与连续关节位置分别建模，不混用 `int16` 协议值与连续位置。
- 多任务共享目标和反馈之前必须约定单一写入方、读取快照、
  同步机制和时效性；`volatile` 本身不等于完整并发同步。
- Driver 与 Application 的状态所有权在控制任务接入时审查，
  未验证前不强行搬迁。
- 严格区分“编译进 ELF”“已经调用”“运行及仿真通过”三种状态。

## 5. 测试结果和历史交付

`docs/part1/`、`docs/part2/` 与周报记录各阶段当时的成果，
不应静默改写成现状。持续更新的架构以 `docs/architecture.md` 为准。

临时测试数据进入 `build/`，已有正式基线先保留；
后续整理时记录代码提交、工具环境、输入条件、指标与判定阈值。
对 Git 已跟踪文件，仅新增 `.gitignore` 不会自动取消跟踪。

当前 CMake 定义了独立 QEMU 测试固件目标，但**目标存在、构建成功、
测试实际运行**是三件事，必须分别验收。

## 6. Cleanup 回归门槛

工作分支的 GitHub Actions 现有两道独立检查：

- `native-regression`：在 Ubuntu 上运行三个 CTest 本机回归目标。
- `arm-firmware-build`：安装 ARM GCC 与 Ninja，克隆固定标签
  `FreeRTOS-Kernel V11.1.0`，执行 `cmake --preset arm-qemu` 和全目标构建。

通过 ARM ELF 构建**不等于**已经在 QEMU 执行测试固件，
也不等于 PyBullet/UART 端到端联调成功。真实闭环控制的故障处理
与物理硬件验证须在 Control Task 实装时独立验收。

当前唯一 Motor Driver 反馈写入任务是 ProtocolRX。
任务运行期间，反馈更新、快照及目标读写统一经过
`application/motor_state.c` 的同一 Mutex。
`motor_driver.c` 保持 RTOS 无关；初始化阶段、Scheduler 启动前，
`main.c` 可以直接初始化驱动和初始目标。
接入 Control Task 时不得绕过 `motor_state` 访问状态。

反馈超时判定函数只报告新鲜度，没有启停执行器的副作用。
为防止误用，未经超时阈值、停机策略和恢复测试，不允许将其
宣称为已实现的机器人运行安全机制。

## 7. Git 工作方式

重构期间保留原始备份分支，只在工作分支修改，`main` 不直接改。
每次提交尽量保持单一主题：`build`、`refactor`、`test`、`docs`、
`fix`、`feat`。改动前后检查：

```powershell
git status --short
git diff --check
git diff --stat
```

没有 ARM 构建或 QEMU 运行环境时，在提交说明中明确
“静态审查完成，运行验证待执行”，不得声称测试通过。
