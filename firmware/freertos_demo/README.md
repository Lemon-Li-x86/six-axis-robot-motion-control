# FreeRTOS 运动控制固件

本固件面向 Cortex-M4 的 QEMU MPS2-AN386，包含 ProtocolRX/ProtocolTX、Trajectory、Control、Monitor 任务，UR5 FK/IK、APF、轨迹规划、PID 与关节限幅。整体使用步骤、逐文件索引和测试结果判据见 [根 README](../../README.md)。

## 配置唯一来源

- `config/robot_defaults.json`：任务周期/优先级/栈、固件 PID、速度/加速度、超时和启动目标等默认值。
- `tools/generate_firmware_defaults.py`（仓库根目录）：由 JSON 生成 `firmware_defaults_generated.h`；生成文件不手动编辑。
- `application/task_config.h`：FreeRTOS TASK_* 别名；`config/stage4_motion_config.c`：生成参数及 FK/IK 容差的只读适配，**均非另一份默认参数表**。
- `model/ur5_model.c`：UR5 DH、坐标帧与关节位置限位。
- `simulation/pybullet_ur5/scenarios/stage5_static_obstacles.json`：仿真场景输入。

## 调用与模块职责

`main.c` 完成设备与任务初始化；`drivers/` 负责底层字节/硬件抽象；`communications/` 解析帧；`application/` 分发命令、维护同步状态并协调 FreeRTOS 任务；`algorithms/` 只做运算，不直接调用 UART/FreeRTOS。APF 只提出候选目标，实际关节运动另经全连杆门禁、轨迹和控制闭环。

普通目标默认关闭 Stage4 任务/输出；Stage5 UART 集成使用 `cmake --preset stage5-live-uart` 明确开启仿真任务门控。此项目未验证实体机器人电机安全。

## 常用命令（仓库根目录）

```powershell
cmake --preset arm-qemu
cmake --build --preset firmware
cmake -S firmware/freertos_demo/tests/host -B build/tests-host
cmake --build build/tests-host --config Debug
ctest --test-dir build/tests-host -C Debug --output-on-failure
```

Stage5 ARM C APF / PyBullet 自动验收运行 `tools/run_stage5_embedded_apf_windows.ps1`；真人 GUI 验收运行 `tools/run_stage5_live_windows.ps1`。参数、示例、PASS 字段及测试矩阵见 [根 README](../../README.md)。
