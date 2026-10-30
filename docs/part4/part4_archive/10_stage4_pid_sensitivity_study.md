# 第四阶段收尾：多工况 PID / PyBullet 灵敏度试验

## 目的、版本与边界

- 工作分支：review/stage4-closure-evidence-20261027；第四阶段冻结分支 backup/stage4-verified-2026-10-27 **保持不动**。
- 本轮工作是补充第四阶段 PID 整定依据，而不是给 Stage4 添加新生产控制算法。
- 桥接库直接链接现有 C 轨迹、PID 和限速器及 UR5 模型。Python **没有重写 PID**。
- 每次实验用 PyBullet DIRECT 执行带关节状态反馈的物理仿真，保存 6 轴参考角、控制命令、实测角与轨迹误差。
- 本轮 **不运行 ARM QEMU / FreeRTOS**。这些既有验证见 08_stage4_qemu_timing_and_stack_2026-10-27.md；真实 UART ↔ QEMU ↔ PyBullet 联调属于第五阶段。

## 单一参数来源与测试夹具

| 类型 | 唯一来源 | 说明 |
|---|---|---|
| 默认 PID、积分及输出限幅、速度与加速度限制 | firmware/freertos_demo/config/robot_defaults.json | 生产策略不修改，仍由 C 装配器注入 |
| 增益试验注入 ABI | firmware/freertos_demo/tests/simulation/stage4_sim_bridge.c | [6][3] 数组，列顺序 Kp/Ki/Kd；仅作用于测试实例 |
| 运动起点、终点 | simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py | 试验初始姿态、目标构型，raw=0.01° |
| 负载、PI/PID试验增益 | tools/stage4_pid_study.py | PyBullet 位置执行器最大力矩试验夹具，不是实物额定负载 |
| 物理步长、原始验收门限 | stage4_dynamic_closed_loop.py | 测试政策，不属于 JSON 生产参数 |

测试矩阵为 3 种运动构型 × 2 档执行器最大力矩 × 3 个试验配置，共 18 组：

1. 基线：直接读 JSON 默认六轴 P/I/D；
2. PI 实验：保留 JSON 中的 P，各轴增加测试用非零 I；
3. PID 实验：PI 实验基础上增加测试用非零 D。

**仅基线 J2 正向移动与默认模拟输出力矩采用之前的正式验收门限作为 CI 阻断条件。** 其它组合为灵敏度研究，可能超出原小幅试验的 RMSE、超调或到位门限。保留真实指标，不将失败掩盖为 PASS，也不因结果好看便修改默认 PID 参数。运行时异常仍判失败。

## 运行

Windows PowerShell 在仓库根目录，要求 CMake、C 编译器、项目 .venv 和 PyBullet：

~~~powershell
cmake -S firmware/freertos_demo/tests/simulation -B build/stage4-closure-simulation
cmake --build build/stage4-closure-simulation --config Debug
ctest --test-dir build/stage4-closure-simulation -C Debug --output-on-failure

# MSVC + Debug 默认路径。Ninja 单配置库可能在父目录。
.\.venv\Scripts\python.exe tools/stage4_pid_study.py --library build/stage4-closure-simulation/Debug/stage4_sim_bridge.dll --output-dir build/stage4-pid-study
~~~

如使用 Ninja + MinGW 或 Linux，--library 应指向实际编译得到的 .dll 或 .so。不要把试验用 I/D 增益写回生产配置。

单元测试：

~~~powershell
.\.venv\Scripts\python.exe -m unittest discover -s tools/tests -p test_stage4_pid_study.py -v
~~~

GitHub Actions 使用 .github/workflows/stage4-closure-evidence.yml，重新运行既有 Host 算法套件、C 桥接 smoke 和本次研究试验。

## 输出与解释

- study_matrix.csv：18 组试验的执行状态、六轴最大最终误差、J2 RMSE/超调、六轴最大 RMSE、样本数。
- study_summary.json：构型、试验条件、完成数、门限状态及异常。
- 每组独立目录中的 stage4_pybullet_samples.csv 和 stage4_pybullet_metrics.json：逐帧复核。
- GitHub Actions 归档原始记录，不能只留摘要表。

**验收语义严格分开**：integration_executed 表示代码运行结束；baseline_acceptance_passed 表示是否满足原小幅试验门限；nominal_reference_passed 表示原正式基线工况通过。

CI 运行完成前不在文档中填写新的实测数字。结果应以对应 Actions run 及原始 artifact 为准。

## 尚未关闭的范围

本矩阵不证明实物执行器性能、全部限位工况、真实固件 UART 时序、多轴笛卡尔轨迹的物理执行、30 分钟满负载、独占 CPU 使用率或硬件 WCET。

第四阶段已有短时 QEMU P99、最大处理经过时间与栈水位数据，均来自其独立报告。不能将 PyBullet 宿主机计算时间或 QEMU 虚拟时间充作实物 WCET，也不能仅凭新测试便把第四阶段宣称为全范围正式结项。


## 2026-10-27 首次云端实测（含原始数据）

[GitHub Actions Run #40487601864](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40487601864) 已完成，source SHA 为 fa1774027b0d2fdac6c56847999cd8217aab9899。原 Stage4 Host 6/6、C 桥接 smoke 1/1、Python 配置单元测试通过，18/18 PyBullet DIRECT 试验没有运行异常，原基线 J2 正向150 N·m 场景通过。

**重要：18组中有12组满足原小步门限，6组未达到。** 它们均为新增六轴中等幅度场景，不等于6个程序崩溃。下表以六轴最终最大关节误差、J2轨迹RMSE、J2超调为依据，单位均为度。

| 中等幅度构型 / PyBullet 最大力矩 | 控制配置 | 最终最大误差 | J2 RMSE | J2超调 | 原小步门限 |
|---|---|---:|---:|---:|---|
| 150 N·m | 原默认 P | 0.02 | 0.2615 | 0.00 | 未通过：RMSE |
| 150 N·m | 测试 PI | 0.03 | 0.2568 | 0.00 | 未通过：RMSE |
| 150 N·m | 测试 PID | 0.02 | 0.2592 | 0.00 | 未通过：RMSE |
| 60 N·m | 原默认 P | 0.02 | 0.2726 | 0.71 | 未通过：RMSE、超调 |
| 60 N·m | 测试 PI | 0.03 | 0.3041 | 0.82 | 未通过：RMSE、超调 |
| 60 N·m | 测试 PID | 0.02 | 0.1953 | 0.45 | 未通过：超调 |

原仿真门限：最终最大误差不高于0.25°、J2轨迹RMSE不高于0.25°、J2超调不高于0.15°。**这套门限原本针对2°小幅试验；跨工况比较仅用于揭示跟踪灵敏度，不能直接推断系统正式验收失败或合格。** 六组最终位置误差均在0.02°～0.03°，但中途轨迹误差和超调仍值得继续评估；仅凭终点误差小不能判运动质量好。

上述测量来源为 GitHub Actions 归档 stage4-native-pid-sensitivity-traces 中的 study_matrix.csv，以及对应各工况 stage4_pybullet_metrics.json / stage4_pybullet_samples.csv。测试版 PI/PID 不应作为生产默认参数；这批结果不足以据此计算“最佳PID”。


## 2026-10-27 第二轮：分离筛选与保留工况复核

第一轮六轴5°中等幅度工况暴露了6组超出原2°小幅门限的问题：150 N·m 主要是跟踪 RMSE 略超标；60 N·m 还存在0.45°～0.82°的 J2 超调。当前尚不能因此认定生产 PID 算法出错。也不能仅凭最后到位误差很小就忽略整段超调。

为避免重复“拍一个 Kp/Ki/Kd 再盲试”，新增 tools/stage4_pid_gain_sweep.py。只通过测试用 C 桥接接口构造实验控制器：
- **校准集**：六轴5°、150和60 N·m；
- **验证集**：未用于排名的 J2 正向、反向移动，同样两档模拟最大力矩；
- **参数**：Kp 在 JSON 默认的倍率上变化，测试 I/D 有明确有限候选；不修改生产默认；
- **比较**：对已有0.25° J2 RMSE、0.15° J2超调、0.25°最大最终误差计算最坏超标比，保存每个候选的全部 CSV/JSON；
- **原则**：先只按校准集排名，再在独立保留工况复查前五名。即使某候选6组均满足门限，也仅代表当前软件夹具，不是最优PID或硬件标定。

运行一次 Windows 本机收尾测试（从仓库根目录）：

~~~powershell
git fetch origin
# 在工作区干净的前提下，切换到审查分支
git switch --track origin/review/stage4-closure-evidence-20261027
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
~~~

如果本机已有同名本地分支，改用 git switch review/stage4-closure-evidence-20261027 以及 git pull --ff-only。首条命令前需运行 git status --short 检查是否有未提交工作；禁止强制覆盖用户本机修改。

产物位于 build/stage4-closure-local/evidence/：
- pid-matrix：18组 Stage4 C/PyBullet 试验，包含逐帧关节运动曲线数据；
- gain-sweep：校准集和保留工况的所有试验目录、calibration.csv、validation.csv、sweep_summary.json。

参数改动、性能门限和硬件安全策略只有在完成验证且被单独审查后才能进入生产配置。**这批实验不能被当作 QEMU + PyBullet 的实时控制链路证据**；Stage4 QEMU 回归会通过原有 tools/test_stage4_once.ps1 独立执行。

云端测试状态不在运行开始时预填 PASS，必须链接实际完成的 Actions run。
