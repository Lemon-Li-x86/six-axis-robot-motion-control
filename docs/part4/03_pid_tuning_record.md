# 第4阶段 PID 调优与轨迹性能记录

## 记录状态

**尚无本机 PyBullet 闭环调参实测数据，不能标注 PASS。**

| 项目 | 计划/来源 | 实测 |
|---|---|---|
| 控制周期 | config/robot_defaults.json: 10ms | 待测 |
| 轨迹输出周期 | config/robot_defaults.json: 20ms | 待测 |
| 六轴仿真初始 Kp | config/stage4_motion_config.c: 0.8 | 未整定 |
| 初始 Ki / Kd | 配置文件：均为 0 | 未整定 |
| P/I/D 抗饱和 | 增量式 PID + 积分限幅 + 条件积分冻结 | 待测 |
| 位置跟踪 MAE / RMSE | 记录各轴误差（deg） | 待测 |
| 最大超调 | deg / % | 待测 |
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

**状态声明**：截至源码提交，此次新增仿真尚未得到真实数据，因此不能填写上表中的超调、MAE、RMSE 或 PID 整定“通过”。本次为默认参数的**基线试验**，并未盲目自动调参；真正的 PID 增益需根据仿真曲线及运行稳定性有依据地修订单一配置。原有 FreeRTOS/UART 实机通信动态闭环仍不在此原生 DLL 的覆盖范围。
