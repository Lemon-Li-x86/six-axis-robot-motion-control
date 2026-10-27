# UART完整闭环本机整改记录（2026-10-27）

## 版本与范围

- 项目目录：`D:\1_ToGo\Robo\code\six-axis-robot-motion-control`。
- 分支：`feat/stage5-system-integration-2026-10-27`。
- 基线：`927b3090a6d272c5d64cabffccc3e2336f6e4654`，本报告对应其上的本地未提交修改，不能标成该SHA原样通过。
- main及冻结备份未修改；没有提交、推送或触发云端工作流。
- 仅验证单次关节请求经ARM/FreeRTOS、QEMU UART、PyBullet反馈闭环，不包含Cartesian IK、APF全路径或长时压力。

## S5-010：启动演示目标被误当成控制输出

原脚本收到运动请求ACK后，立即执行所有0x01目标帧。ACK只代表入队，不能保证Control已经发布第一个输出。ProtocolTX此时仍可能发送JSON中的历史演示目标（J2=0°），而PyBullet初始J2=-45°。

本机原始运行：请求ACK=0，目标帧130，读取的规划/控制计数均0，最大J2指令变化45°，实测J2变化1.01°，总判定false。延长诊断运行时，旧判据甚至可在最大指令变化45°的情况下返回true，因此“动了”不足以验收。

修复：仅在Stage4任务和命令输出同时启用时，ProtocolTX在Control首次发布前发送已接受的实测姿态；未有反馈时不发送目标帧。先读取发布计数，再读取控制目标，避免反向读取顺序跨越首次发布时发送旧默认值。旧版独立通信演示配置沿用原行为。

## S5-011：单次过早诊断与提前结束

旧脚本只读取一轮诊断，在规划启动前得到0后不会刷新；帧数满足即结束，没有最终到位门禁。本机带符号GDB现场在RTOS tick=1401时实际看到规划43、控制83、故障0、请求队列已消费，证明一次早期0快照不能判断任务一直未运行。此现场只解释本机观察，不能直接代替旧GitHub失败运行的根因证据。

修复：仿真端主动发送第一组反馈；以后仍按收到目标帧推进物理仿真并返回反馈。每隔至少10个目标帧重新读取完整诊断轮次，要求至少130目标帧、规划与控制均推进、故障0、零坏校验，以及全部六轴距目标最大误差不超过0.10°并连续保持20个目标帧。请求前保持误差不得超过0.02°。这些是测试门限，不改PID、模型、轨迹或IK容差。

## 本机最终运行

工具链：ARM GCC 15.3.1；Windows本机QEMU MPS2-AN386；项目`.venv`内PyBullet。闭环固件为独立Debug构建目录，显式打开Stage4任务、命令输出及Stage5模拟UART集成，关闭RTOS自检。完整回归另外使用命令输出OFF配置。

| 项目 | 最终实测 |
|---|---:|
| 请求ACK | 0 |
| 目标帧 | 135 |
| 反馈发送 / ACK | 136 / 135 |
| 完整诊断轮数 | 11 |
| 规划 / 控制 | 65 / 130 |
| 规划 / 控制故障 | 0 / 0 |
| 最大J2指令变化 | 0.98° |
| 最终最大六轴误差 | 0.02° |
| 连续到位目标帧 | 93 |
| 请求前保持误差 | 0° |
| 坏校验 | 0 |
| 判定 | PASS |

结束时最后一组反馈ACK尚在途，因此反馈发送比ACK多1；本测试不把该差值称为丢包率，也不声称测得零丢包。

原始本机证据位于（build不随Git提交，应独立归档）：
- `build/stage5-local-uart-results/stage5_uart_pybullet_metrics.json`：修复前失败。
- `build/stage5-local-uart-diagnose/gdb.txt`：带符号任务/队列快照。
- `build/stage5-local-uart-fixed/stage5_uart_pybullet_metrics.json`、同目录trace：最终修复后通过。
- `build/stage4-final-validation.log`：统一8步回归全部通过；Host六组、QEMU轨迹14/14、FK→IK→Selector→FK 128/128且失败/跳过0、近奇异2/2、RTOS29项，以及两组原生C/PyBullet场景通过。
- `build/stage5-local-host/Testing/Temporary/LastTest.log`：五组Host通过，其中协议异常流26个断言、0失败。

## 复现

在项目根目录运行：

```powershell
cmake -S . -B build/stage5-local-uart -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake' -DCMAKE_BUILD_TYPE=Debug -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON -DROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=ON -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF
cmake --build build/stage5-local-uart --target freertos_demo
.\.venv\Scripts\python.exe simulation/pybullet_ur5/src/stage5_qemu_uart_closed_loop.py --elf build/stage5-local-uart/firmware/freertos_demo/freertos_demo.elf --output-dir build/stage5-local-uart-fixed
```

当前结论：本机完整串口关节闭环通过，云端仍待提交后独立复验。此前GitHub取消或失败记录不被本机结果改写。
