# 第四阶段：PID参数调优记录与轨迹性能分析文档

**项目**：六轴工业机器人嵌入式运动控制固件开发与仿真验证  
**整理日期**：2026-10-28  
**交付类别**：原项目大纲第四阶段第二项正式文档交付物  
**源码基线**：`48e08c32b6a816014ff9e1b67462150f35d0a313`（Stage4 RC，经 PR #5 合入 `main`）。  
**实验平台**：ARM Cortex-M4 / FreeRTOS / QEMU、C 运动控制模块、UR5 / PyBullet 软件仿真。

## 1. 研究目标与模块实现

第四阶段实现六关节轨迹规划、笛卡尔直线与圆弧插补、增量式关节 PID，并基于 UR5 / PyBullet 反馈开展参数调优。本报告整理算法实现、测试方法、动态跟踪指标与参数选择过程。

| 功能 | 生产 C 代码 | 仿真验证路径 |
|---|---|---|
| 三次/五次多项式轨迹 | `firmware/freertos_demo/algorithms/trajectory/polynomial_trajectory.c` | Host 数值边界、ARM/QEMU 轨迹测试、多项式 FreeRTOS 发布前检查 |
| 梯形/三角加减速 | `algorithms/trajectory/trapezoidal_profile.c` | 六轴同步、限速/限加速度与 PyBullet 反馈场景 |
| 笛卡尔直线/圆弧 | `algorithms/trajectory/cartesian_{line,arc,waypoint}.c` | Host 插补、IK/Selector/FK 回代及规划任务离散航点 |
| 单关节增量式 PID | `algorithms/control/pid.c`、`joint_controller.c` | Host P/I/D 及启动历史误差回归；同源 C/PyBullet 动态实验 |
| 输出运动约束 | `algorithms/control/joint_command_limiter.c`、`application/tasks/trajectory_task.c` | 位置、速度、加速度门限及反馈失效拒绝 |

上表中 `algorithms/` 的相对路径均以 `firmware/freertos_demo/` 为根。关节连续位置采用 **0.01°/整数单位**，速度和加速度分别为 °/s、°/s²；笛卡尔位置采用 mm。轨迹规划和 PID 计算调用生产 C 模块，Python 负责 PyBullet 物理仿真与指标采集。

## 2. 关节与笛卡尔轨迹设计、质量检验

### 2.1 关节轨迹

- **三次插值**：在给定持续时间内匹配起点与终点的位置、速度。
- **五次插值**：在上述约束外同时匹配起点与终点加速度。
- **梯形/三角速度规划**：根据每关节给定速度和加速度约束计算共享归一化运动进度，同步六关节起停；短距离自动形成三角曲线。
- **整段发布前检查**：`trajectory_polynomial_validate_limits()` 检查整个归一化时间区间内位置、速度和加速度的端点及内部极值；规划通过后按20ms离散采样，运行期间继续进行逐点检查。越限计划在发布首个参考点前即被拒绝。

轨迹数值验证包括 **32 组边界工况 × 每组1001个采样点**的 Host 密采样对照、ARM/QEMU 轨迹算法 **20/20** 项检查，以及 FreeRTOS 多项式拒绝与恢复路径 **8/8** 项检查。

### 2.2 笛卡尔直线、圆弧与逆解航点

- 位置沿直线或三点圆弧采样，姿态使用四元数最短路径插值。
- 规划任务按配置的 **20ms** 轨迹周期取离散末端位姿，调用现有 IK 与选解器取得连续关节目标，并检查 FK 回代位置/姿态残差及关节速度连续性。
- 输入起点与当前反馈不一致、目标不可达、限位或近奇异分支被拒绝时，不发布无效航点。

笛卡尔轨迹已完成离散几何、IK、选解及 FK 回代的 Host/QEMU 回归。本阶段动态性能试验重点测量关节空间的跟踪误差；笛卡尔末端连续轨迹误差与连杆碰撞性能列为后续专项测试内容。

## 3. PID 结构与当前参数

控制器为六轴独立**增量式 PID**，支持各轴独立 Kp/Ki/Kd、积分限制、输出限制、死区和反积分饱和处理；微分项首样本误差历史的虚假脉冲问题已有针对性修复和回归。

生产参数来自 `firmware/freertos_demo/config/robot_defaults.json` 中的 `stage4_motion.pid[0..5]`，构建时生成 C 宏，并经配置装配器注入六轴控制器。

| 参数（六轴当前相同，均可按轴配置） | 当前仿真候选 |
|---|---:|
| Kp | **0.25** |
| Ki | **0** |
| Kd | **0.025** |
| 积分限幅 | 10 °·s |
| 输出修正限幅 | 3° |
| PID 死区 | 0.02° |
| 关节速度上限 | 15°/s |
| 关节加速度上限 | 30°/s² |
| Control / Trajectory 周期 | 10ms / 20ms |
| 反馈超时 | 3000ms |

历史基线先后使用 `Kp=0.8, Ki=0, Kd=0` 和 `Kp=0.2, Ki=0, Kd=0`；当前候选改为 `0.25 / 0 / 0.025`。

## 4. 动态跟踪性能与调参记录

### 4.1 小幅关节运动：历史基线

生产同源 C 轨迹/PID/限速器共享库与 PyBullet DIRECT 在 **Kp=0.2、Ki=0、Kd=0** 的历史基线上得到：

| 指标 | J2 单轴 +2° | 六轴小幅同步 |
|---|---:|---:|
| 采样点数 | 253 | 253 |
| J2 轨迹 RMSE | **0.1300456°** | **0.1306400°** |
| J2 最大超调 | **0°** | **0°** |
| 最大关节末端目标误差 | 0.02° | 0.02° |

这组数据对应历史 `Kp=0.2` 的两种小幅关节运动，用作后续调参的性能基线。

### 4.2 18组灵敏度矩阵与异常结果

扩展试验使用 **3种运动构型 × 2档 PyBullet 模拟执行器最大力矩（150/60 N·m）× 3类 PID 试验配置**，比较 J2 跟踪 RMSE、超调及六轴末端误差。力矩表示 PyBullet 中的执行器输出上限，用于比较驱动力变化对跟踪性能的影响。

2026-10-28 Windows 复验结果：**18/18 组执行完成，16/18 组满足探索性性能门限**。另外两组是六轴中等幅度、60 N·m 条件下的*试验 PI* 与*试验 PID*，J2 超调分别为 **0.55° / 0.38°**，高于 **0.15°** 的对照门限。两组均完成运行，差异体现在动态超调指标。

较早一轮参数配置的矩阵结果为 **12/18** 满足同一探索性门限；本轮为 **16/18**。两轮采用不同源码版本及参数配置，原始结果分别留档。

### 4.3 候选选择与独立验证

在原 P 基线基础上，试验按规定的候选增益倍率和有限 Ki/Kd 组合执行搜索：
- **60 组校准**，用于比较候选；
- **20 组保留验证**，用于独立评价筛选出的候选；
- 对选中候选检查六个软件试验条件，结果 **6/6 符合门限**；
- 最坏的“性能指标/相应测试门限”比值 **0.7673293597**（越小于1越满足本次门限）。

扫参记录 `kp_1_ki_0_kd_0.025` 中的 `kp_1` 表示基线 Kp 的 **1 倍**，对应生产配置 **Kp=0.25、Ki=0、Kd=0.025**。

本轮 UR5/PyBullet 试验采用最大末端目标误差 **≤0.25°**、J2 轨迹 RMSE **≤0.25°**、J2 超调 **≤0.15°** 作为比较门限。候选在所选六个工况中全部达标。

## 5. 不同验证层的证据

| 证据层 | 可复核结果 | 来源 |
|---|---|---|
| 本机 Windows Stage4 收尾 | Host CTest 6/6、C bridge 1/1、18/18 执行、16/18 探索门限、60/20 扫参、6/6 候选条件 | 本机源码 `6cedeff99fac67b6aa4a25ccaa18909722164ccc`；本机测试摘要、`build/` 日志 |
| 同一生产候选的 Linux/Windows CI | 原生 PID 矩阵、Windows Host、ARM/QEMU 回归3个 Job 均成功 | [Stage4 CI #40639281389](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40639281389)，HEAD `48e08c3` |
| QEMU/FreeRTOS ↔ UART ↔ PyBullet 额外集成测试 | 最新一次三段运动 **3/3 SETTLED**；该次最大片段最终关节误差约 **0.09°** | 同上 QEMU Job；Stage5 UART 集成专项回归 |
| 合入 `main` 后原生与 ARM 构建 | 2/2 Job 成功 | [main CI #40642030001](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40642030001)；合并后源码文件与 RC 完全一致 |

UART 集成回归曾在 [CI #40633371737](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40633371737) 出现首段超时；最新 [CI #40639281389](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40639281389) 三段均完成。两轮记录为后续稳定性分析提供依据。

## 6. 复现、数据与结论

Windows 仓库根目录，在已有工具链及 Python 环境下：

~~~powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
~~~

首条执行 Stage3/4 非驱动核心回归；第二条执行18组灵敏度矩阵、增益搜索及相关 QEMU 验证。输出见 `build/stage4-final-validation.log` 与 `build/stage4-closure-local/evidence/`，后者包含逐帧 CSV、JSON 指标和筛选摘要。云端 PID 与 UART 运行的原始附件保存在对应 GitHub Actions Artifacts 中。

第四阶段形成了可复现的关节/笛卡尔轨迹 C 算法、六轴增量式 PID 控制器和受控仿真调参结果。候选 `Kp/Ki/Kd=0.25/0/0.025` 在指定六工况中符合性能门限。后续性能研究将扩展笛卡尔末端全程误差、复杂轨迹和更多动力学条件。

### 历史证据位置

- [原 PID 基线记录](../part4_archive/03_pid_tuning_record.md)
- [原18组 PID 灵敏度研究](../part4_archive/10_stage4_pid_sensitivity_study.md)
- [候选参数与回归记录](../part4_archive/11_stage4_pid_candidate_verification_2026-10-27.md)
- [完整 RC 验收单及待归档事项](../part4_archive/12_stage4_release_candidate_2026-10-28.md)
- [多项式整段预检记录](../part4_archive/09_polynomial_preflight_and_json_config_2026-10-27.md)

历史资料按测试阶段保留，便于追溯参数演变与试验结果。