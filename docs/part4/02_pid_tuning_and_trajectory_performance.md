# 第四阶段：PID参数调优记录与轨迹性能分析文档

**项目**：六轴工业机器人嵌入式运动控制固件开发与仿真验证  
**整理日期**：2026-10-28  
**交付类别**：原项目大纲第四阶段第二项正式文档交付物  
**源码基线**：Stage4 仿真候选 `48e08c32b6a816014ff9e1b67462150f35d0a313`，已通过 PR #5 合入 `main`；后续只改文档的提交不改变这批测试的实际被测源码。  
**范围**：ARM Cortex-M4 / FreeRTOS / QEMU 和生产同源 C 控制算法 / UR5 / PyBullet 的**软件仿真**。

## 1. 目标与实现边界

根据任务书，第四阶段完成六关节空间轨迹、笛卡尔直线与圆弧轨迹、增量式关节 PID 和仿真反馈调参。本报告汇总**实际已有的轨迹算法测试、动态跟踪指标和参数选择依据**；不把系统集成、实体安全或未测工况作为已完成成果。

| 功能 | 生产 C 代码 | 仿真验证路径 |
|---|---|---|
| 三次/五次多项式轨迹 | `firmware/freertos_demo/algorithms/trajectory/polynomial_trajectory.c` | Host 数值边界、ARM/QEMU 轨迹测试、多项式 FreeRTOS 发布前检查 |
| 梯形/三角加减速 | `algorithms/trajectory/trapezoidal_profile.c` | 六轴同步、限速/限加速度与 PyBullet 反馈场景 |
| 笛卡尔直线/圆弧 | `algorithms/trajectory/cartesian_{line,arc,waypoint}.c` | Host 插补、IK/Selector/FK 回代及规划任务离散航点 |
| 单关节增量式 PID | `algorithms/control/pid.c`、`joint_controller.c` | Host P/I/D 及启动历史误差回归；同源 C/PyBullet 动态实验 |
| 输出运动约束 | `algorithms/control/joint_command_limiter.c`、`application/tasks/trajectory_task.c` | 位置、速度、加速度门限及反馈失效拒绝 |

上表中 `algorithms/` 的相对路径均以 `firmware/freertos_demo/` 为根。关节连续位置采用 **0.01°/整数单位**，速度和加速度分别为 °/s、°/s²；笛卡尔位置采用 mm。算法层不重写 Python PID，Python 仅作为 PyBullet 物理仿真及测试指标采集端。

## 2. 关节与笛卡尔轨迹设计、质量检验

### 2.1 关节轨迹

- **三次插值**：在给定持续时间内匹配起点与终点的位置、速度。
- **五次插值**：在上述约束外同时匹配起点与终点加速度。
- **梯形/三角速度规划**：根据每关节给定速度和加速度约束计算共享归一化运动进度，同步六关节起停；短距离自动形成三角曲线。
- **整段发布前检查**：`trajectory_polynomial_validate_limits()` 检查整个归一化时间区间的位置、速度和加速度端点及内部极值，而不只检查20ms采样点。非法计划在发布第一个运动参考值之前拒绝，运行期间还保留逐点检查。

已有数值证据：Host 对 **32 组边界 × 1001 个采样点**进行密采样对照；ARM/QEMU 轨迹算法 **20/20** 通过；FreeRTOS 多项式拒绝与恢复路径 **8/8** 通过。以上是**数值与接口正确性证据**，不是实际机械关节的最大速度/最大加速度示波曲线。

### 2.2 笛卡尔直线、圆弧与逆解航点

- 位置沿直线或三点圆弧采样，姿态使用四元数最短路径插值。
- 规划任务按配置的 **20ms** 轨迹周期取离散末端位姿，调用现有 IK 与选解器取得连续关节目标，并检查 FK 回代位置/姿态残差及关节速度连续性。
- 输入起点与当前反馈不一致、目标不可达、限位或近奇异分支被拒绝时，不发布无效航点。

目前有离散几何与 IK/Selector 的 Host/QEMU 回归，**没有可量化的整段笛卡尔末端物理轨迹 RMSE、全关节连续加速度或连杆碰撞指标**。因此不能将关节 J2 跟踪精度当成笛卡尔轨迹精度。这是本交付物的明确测量边界。

## 3. PID 结构与当前参数

控制器为六轴独立**增量式 PID**，支持各轴独立 Kp/Ki/Kd、积分限制、输出限制、死区和反积分饱和处理；微分项首样本误差历史的虚假脉冲问题已有针对性修复和回归。

生产参数唯一来源：`firmware/freertos_demo/config/robot_defaults.json` 中的 `stage4_motion.pid[0..5]`，由构建生成 C 宏后通过配置装配器注入，不在 Python 或 PID 生产源码里另存一份可修改默认增益。

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

早期的 `Kp=0.8,Ki=0,Kd=0` 和 `Kp=0.2,Ki=0,Kd=0` **只是已归档的历史对照版本**；不能在本次版本的“当前参数”栏再次使用。

## 4. 动态跟踪性能与调参记录

### 4.1 历史小幅运动基线（旧参数，不是现行候选）

生产同源 C 轨迹/PID/限速器共享库与 PyBullet DIRECT 在 **Kp=0.2、Ki=0、Kd=0** 的历史基线上得到：

| 指标 | J2 单轴 +2° | 六轴小幅同步 |
|---|---:|---:|
| 采样点数 | 253 | 253 |
| J2 轨迹 RMSE | **0.1300456°** | **0.1306400°** |
| J2 最大超调 | **0°** | **0°** |
| 最大关节末端目标误差 | 0.02° | 0.02° |

这些数值是当时小幅场景的测量结果，不能作为新 PID 候选的重新实测数据，也不能解释为机器人实体定位精度。

### 4.2 18组灵敏度矩阵与异常结果

扩展试验使用 **3种运动构型 × 2档 PyBullet 模拟执行器最大力矩（150/60 N·m）× 3类 PID 试验配置**，比较 J2 跟踪 RMSE、超调及六轴末端误差。力矩是仿真执行器夹具参数，**不是实物负载标定**。

在 2026-10-28 Windows 收尾验收中：**18/18 组无异常执行，但只有 16/18 组符合沿用小幅场景的探索性性能门限**。未达门限的两组均为六轴中等幅度、60 N·m 情况下的*试验 PI* 和 *试验 PID*：J2 超调分别 **0.55° / 0.38°**，高于对照门限 **0.15°**。不把它们写成程序崩溃，也不隐去超调。

历史较早的首次矩阵曾出现 **12/18** 达到同样的探索性门限，属于**不同 SHA 与不同默认参数的旧试验**。不同轮次的 12/18 和 16/18 不能混成一次测试。

### 4.3 候选选择与独立验证

在原 P 基线基础上，试验按规定的候选增益倍率和有限 Ki/Kd 组合执行搜索：
- **60 组校准**，用于比较候选；
- **20 组保留验证**，不用于搜索排序；
- 对选中候选检查六个软件试验条件，结果 **6/6 符合门限**；
- 最坏的“性能指标/相应测试门限”比值 **0.7673293597**（越小于1越满足本次门限）。

扫参结果文件名中的 `kp_1_ki_0_kd_0.025`，`kp_1` 表示相对于该轮基线的 **倍率 1**，并非生产 Kp 数值等于1。写入生产 JSON 的实际候选是 **Kp=0.25、Ki=0、Kd=0.025**。

本轮策略的比较门限为最大末端目标误差 ≤0.25°、J2 轨迹 RMSE ≤0.25°、J2 超调 ≤0.15°。它们是本项目 UR5/PyBullet 的**软件试验判据**，不是机器人产品性能指标；六工况通过不代表任意负载或全工作空间最优。

## 5. 不同验证层的证据

| 证据层 | 可复核结果 | 来源 |
|---|---|---|
| 本机 Windows Stage4 收尾 | Host CTest 6/6、C bridge 1/1、18/18 执行、16/18 探索门限、60/20 扫参、6/6 候选条件 | 本机运行源码 `6cedeff99fac67b6aa4a25ccaa18909722164ccc`，用户确认的测试摘要；完整本机原始 `build/` 尚待独立归档 |
| 同一生产候选的 Linux/Windows CI | 原生 PID 矩阵、Windows Host、ARM/QEMU 回归3个 Job 均成功 | [Stage4 CI #40639281389](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40639281389)，HEAD `48e08c3` |
| QEMU/FreeRTOS ↔ UART ↔ PyBullet 额外集成测试 | 最新一次三段运动 **3/3 SETTLED**；该次最大片段最终关节误差约 **0.09°** | 同上 QEMU Job；属于额外 Stage5 联调验证，不替代独立 C/PyBullet PID 调参数据 |
| 合入 `main` 后原生与 ARM 构建 | 2/2 Job 成功 | [main CI #40642030001](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40642030001)；合并后源码文件与 RC 完全一致 |

之前曾有一轮 RC QEMU UART 三段运动首段超时（[CI #40633371737](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40633371737)），应作为**历史稳定性问题**保留；一次后续成功不等于对所有偶发超时的统计证明。

## 6. 复现、数据与结论

Windows 仓库根目录，在已有工具链及 Python 环境下：

~~~powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_closure.ps1 -IncludeQemuBaseline
~~~

首条用于非驱动 Stage3/4 核心回归；第二条同时包含需要较长时间的18组矩阵与增益搜索。输出见 `build/stage4-final-validation.log` 与 `build/stage4-closure-local/evidence/`，后者包含逐帧 CSV、JSON 指标和筛选摘要。云端 PID 与 UART 运行的原始附件见上述 Actions Run 的 Artifacts；**目前并未声称完整本机证据文件已经提交进 Git 仓库**。

第四阶段的关节/笛卡尔路径**核心 C 算法**、PID 控制与受控软件仿真整定可作为本阶段成果提交。仍未测得独立的笛卡尔物理全程误差、全工作空间/各负载 PID 最优性，以及实体机器人运动安全。后续 Stage5 完整系统集成不与本阶段源代码、轨迹/PID 报告混淆。

### 历史证据位置

- [原 PID 基线记录](../part4_archive/03_pid_tuning_record.md)
- [原18组 PID 灵敏度研究](../part4_archive/10_stage4_pid_sensitivity_study.md)
- [候选参数与回归记录](../part4_archive/11_stage4_pid_candidate_verification_2026-10-27.md)
- [完整 RC 验收单及待归档事项](../part4_archive/12_stage4_release_candidate_2026-10-28.md)
- [多项式整段预检记录](../part4_archive/09_polynomial_preflight_and_json_config_2026-10-27.md)

以上历史文档保留其原始措辞与当时的参数状态；本报告的“当前”以开头注明的源码基线为准。