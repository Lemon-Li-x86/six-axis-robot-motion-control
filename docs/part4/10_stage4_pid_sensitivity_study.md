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
