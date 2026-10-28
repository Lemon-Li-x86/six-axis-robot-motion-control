# FreeRTOS 固件（ARM Cortex-M4 / QEMU）

当前应用包含：ProtocolRX/ProtocolTX、Motor State、Trajectory Task、Control Task、Monitor Task，及 UR5 FK/IK、轨迹规划、PID和运动限速器。

- `main.c` 负责初始化、启动自测和FreeRTOS任务创建。
- `config/robot_defaults.json` 是任务周期、优先级、栈深度与启动目标角唯一默认源；构建时生成头文件。
- `config/stage4_motion_config.c` 是第4阶段PID、位置/速度/加速度策略和超时初值的唯一来源。
- 纯计算置于 `algorithms/`，本体事实置于 `model/`，硬件设备在 `drivers/`，任务与调度在 `application/`。

**安全构建门控**：`ROBOT_ENABLE_STAGE4_TASKS` 与 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT` 均默认 OFF。测试专用 `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST` 仅在 TASKS=ON 且 OUTPUT=OFF 时启用。不要拿QEMU算法成功替代实物电机安全验证。

完整统一非驱动回归见仓库根目录 `tools/test_stage4_once.ps1`；架构、参数及剩余缺口参见 `docs/part4/05_global_architecture_review_2026-10-26.md`。
