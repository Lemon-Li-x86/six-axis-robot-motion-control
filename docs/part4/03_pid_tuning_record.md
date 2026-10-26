# 第4阶段 PID 调优与轨迹性能记录

## 记录状态

**已有 GitHub Actions 原生 C/PyBullet 动态基线与受控扫参实测；尚无本机ARM/UART全链路、实体机器或正式增益标定证据。**

| 项目 | 计划/来源 | 实测 |
|---|---|---|
| 控制周期 | config/robot_defaults.json: 10ms | 待测 |
| 轨迹输出周期 | config/robot_defaults.json: 20ms | 待测 |
| 六轴仿真初始 Kp | `config/stage4_motion_config.c` 当前 0.2（原始基线0.8） | 有云端受控对照，仍非硬件标定 |
| 初始 Ki / Kd | 配置文件：均为 0 | 未整定 |
| P/I/D 抗饱和 | 增量式 PID + 积分限幅 + 条件积分冻结 | 待测 |
| 位置跟踪 MAE / RMSE | 记录各轴误差（deg） | 云端 J2 已测，其余轴详见各次 JSON/CSV |
| 最大超调 | deg / % | 旧 Kp0.8 时 J2 超调1.07°，低 Kp 对照见下节 |
| 稳态误差 | deg | 待测 |
| 控制指令限幅触发 | 计数/时间比例 | 待测 |
| 关节速度/加速度 | 与 max limits 比较 | 待测 |
| 实验环境与 SHA | QEMU、PyBullet、代码 SHA | 待填 |

## 建议测试姿态

初次只在纯仿真环境、小角度、不接近奇异点的关节步进上启用输出。依次测试 J1、J2、J3 单轴与六轴同步，分别用零 P 控制、仅 P、再引入 I 和 D 的小步改动；每次记录实际反馈、期望轨迹、命令轨迹以及采样时间，不得仅凭动画看起来流畅判断通过。

实际整定后更新 `config/stage4_motion_config.c`，并在此文档记录每次 Kp/Ki/Kd 更改的理由、测试时间、试验 SHA。PID 误差使用连续关节位置，不得将 ±180° 的规范角差直接作为误差。

暂不能据此填写“位置精度±0.1°”之类的工业机器人性能承诺。

## 动态闭环试验工具（新增，待运行）

新增可重复的**真实 C 算法 → PyBullet DIRECT** 仿真：
- `firmware/freertos_demo/tests/simulation/stage4_sim_bridge.c` 提供测试专用原生共享库。它直接链接生产代码 `trapezoidal_profile.c`、`pid.c`、`joint_controller.c`、`joint_command_limiter.c`、`motion_limits.c` 与 `ur5_model.c`，不重写一份 Python PID。
- PID 初值、速度与加速度限制全部通过 `stage4_motion_default_config()` 获取；控制周期经 `config/robot_defaults.json` 的唯一来源读取。运动构型仅在 Python 测试脚本中作为试验输入。
- `simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py` 运行 PyBullet DIRECT，实际按控制周期读取六轴关节状态、调用固件同源的C轨迹采样/PID/限速、执行位置控制，输出 `build/stage4-closed-loop/stage4_pybullet_samples.csv` 和 `stage4_pybullet_metrics.json`。
- 记录 J2 轨迹 MAE、RMSE、最大动态跟踪误差、最大超调、稳态目标误差、达到允许带的时间、采样数量和使用的控制周期。这些是**试验计算定义**，不替代任何工业机器人标准认证。
- 以测试夹具中 J2 的小角度、远离 J5/J3 奇异的关节构型开始，不会对实体机器人输出控制；`ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` 在非驱动 QEMU 验证中继续强制 OFF。

GitHub Actions `.github/workflows/stage4-trajectory-host.yml` 配置自动运行原生库构建、C API smoke 和 Python/PyBullet 仿真；Windows 可以通过 `tools/test_stage4_once.ps1` 在最终一次性验收中执行。

**状态更新**：以下章节已记录云端实际动态基线和8组受控扫参，因此 MAE、RMSE、超调均已有量化观测；这仍只是小幅运动的仿真默认增益对照，不能当成全工作空间、不同负载或实体机器的完成整定。FreeRTOS/UART 动态闭环不在 Native C DLL 覆盖范围。

## 云端首轮动态基线实测（GitHub Actions，2026-10-26）

测试来源：[Run #40320326975](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40320326975)，提交 `5872bb03`。经 GitHub Actions Job Log 实际记录确认：`stage3-stage4-host` 与 `stage4-native-pybullet` 均为 **success**；后者完成原生共享库严格编译、C Smoke 和 PyBullet DIRECT 动态测试。

| 项目 | 云端实测 |
|---|---:|
| 固件默认 PID 初始 Kp / Ki / Kd | 0.8 / 0 / 0（均由配置源注入） |
| 控制周期 / 物理步长 | 10 ms / 1 ms |
| J2 测试总目标移动 | +2.00°（由 -45° 到 -43°） |
| 梯形轨迹总时长 | 0.51639777 s |
| 物理仿真总时长 | 2.53 s |
| J2 最终目标误差 | 0.04° |
| J2 轨迹 MAE | 0.42423° |
| J2 轨迹 RMSE | 0.51217° |
| J2 最大超调 | **1.07°** |
| 首次进入并持续保持默认1°稳态容差带 | 0.76 s |
| 动态样本数 | 253 |
| 首轮测试 verdict | 通过 **1°最终位置粗门限**，不等同PID调优完成 |

**解释**：J2 仅移动2°却超调1.07°，幅值相当于本次目标移动的53.5%，明显需要改善。末端误差0.04°不能掩盖动态超调。此结果只表示当前测试环境（URDF、PyBullet Position Control和测试力矩参数）下的观测，不代表机器人额定动态性能或真实固件UART实时闭环。

下一步采用 `stage4_sim_create_with_kp_scale` **仅测试专用**的 Kp 因子扫描（0.25、0.50、0.75、1.00），其他生产控制常数不变。比对超调、RMSE、末端误差及稳定时间后，才决定是否修改 `config/stage4_motion_config.c` 中的唯一默认Kp。不允许直接把试验中的倍数当成新的第二份生产配置，也不允许将调参结果写成真实硬件标定。

## 八组云端对照与新的仿真初始 Kp（2026-10-26）

同一套生产 C 算法分别对 J2 +2°、六轴同步小幅移动，完成 `kp_scale = 0.25,0.5,0.75,1.0` 四档（**原基线 Kp=0.8**）的 **2×4 = 8 组 PyBullet DIRECT** 对照。[GitHub Actions Run #40321477702](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40321477702)，所有试验任务成功。以下指标来自 Job Logs，均为 J2，最终位置误差取六轴中最大值。

| Kp等效值 | J2单轴 RMSE | J2单轴超调 | 六轴同步 J2 RMSE | 六轴同步 J2超调 | 六轴最大最终误差 |
|---:|---:|---:|---:|---:|---:|
| 0.2 | 约0.130° | **0.00°** | 约0.131° | **0.00°** | 0.02° |
| 0.4 | 约0.117° | 0.35° | 约0.117° | 0.35° | 0.02° |
| 0.6 | 约0.356° | 0.88° | 约0.356° | 0.88° | 0.02° |
| 0.8 | 约0.512° | 1.07° | 约0.512° | 1.07° | 0.04° |

**选型依据**：从单次2°小幅运动，优先抑制超调并维持合理轨迹RMSE，因此将 `config/stage4_motion_config.c` 中唯一六轴Kp默认值从0.8改为**0.2**，Ki/Kd仍为0。0.4有略低RMSE但超调0.35°，对如此小位移过高。这是**仿真初始增益**，并非适用于所有负载/工作空间/真实硬件的PID整定结论。

新的CI与一键验收将只测试本次正式默认Kp、两种运动场景，并要求 J2 最大超调不超过0.15°、J2 RMSE不超过0.25°、全部关节末端误差不超过0.25°，三个测试门限属于**仿真验收政策**，不是机器人本体性能参数。最新修改后的再次实测结果仍需独立确认。
