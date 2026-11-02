# Firmware / ARM Cortex-M4 + FreeRTOS

**最新构建、自动回归及 GUI 验收入口请从 [仓库根目录 README](../README.md) 开始。** 本目录保存固件源码、可独立构建的 C 测试以及固件构建所需的工具链文件。历史阶段日志见 [HANDOFF_BOOK](../HANDOFF_BOOK.md)，旧开发分支说明不是最新运行命令。

## 构建

在仓库根目录执行：

```powershell
cmake --preset arm-qemu
cmake --build --preset firmware

# 真实 QEMU UART + PyBullet Stage5 路径所需的固件
cmake --preset stage5-live-uart
cmake --build --preset stage5-live-uart
```

工具：CMake >=3.20、Ninja、ARM GNU Toolchain、Python 3.11，及 `third_party/FreeRTOS-Kernel`（使用上游标签 `V11.1.0`）。配置模板：`CMakePresets.json`；工具链：`cmake/arm-none-eabi-gcc.cmake`。

## 源码边界

| 目录 | 主要职责 |
|---|---|
| `freertos_demo/common/` | 类型、单位、角度转换、错误码 |
| `freertos_demo/model/` | UR5 DH、Frame、关节限位唯一模型 |
| `freertos_demo/config/` | JSON 默认参数、只读派生配置 |
| `freertos_demo/drivers/` | UART、Timer、GPIO、虚拟 Motor Driver |
| `freertos_demo/communications/` | UART 帧协议、环形缓冲 |
| `freertos_demo/algorithms/` | FK/IK、Motion、Trajectory、PID、APF |
| `freertos_demo/application/` | FreeRTOS Tasks、共享状态、命令服务 |
| `freertos_demo/diagnostics/` | 时序与 CPU 诊断 |
| `freertos_demo/tests/` | Host/ARM/QEMU/Unity 功能回归入口 |

**默认参数：** `freertos_demo/config/robot_defaults.json`（构建时生成头文件）；`stage4_motion_config.c` 只汇集生成配置与算法阈值，**不维护第二套 PID 默认值**。UR5 几何/限位由 `model/ur5_model.c` 维护。

## 测试

快速原生测试：

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/tests-host
cmake --build build/tests-host --config Debug
ctest --test-dir build/tests-host -C Debug --output-on-failure
```

其他 CTest 套件：`tests/trajectory`、`tests/avoidance`、`tests/simulation`、`tests/feedback_clock`、`tests/unity`。运行 ARM/QEMU 串口、物理避障和 Windows GUI 的完整命令、PASS 字段和逐文件测试索引统一维护在 [根 README](../README.md)。

截至合并基线 `6a25faab`，Stage5 CI [13/13 PASS](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41399964643)；Windows 真人 GUI 操作签收独立待做。Stage6 尚未完成。
