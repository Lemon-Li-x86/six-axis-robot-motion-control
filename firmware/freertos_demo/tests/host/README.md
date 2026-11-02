# Host C Test Suite

从仓库根目录运行，使用本机编译器，不依赖 ARM/QEMU/FreeRTOS 或 Unity：

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/tests-host
cmake --build build/tests-host --config Debug
ctest --test-dir build/tests-host -C Debug --output-on-failure
```

**PASS：** CTest 退出码 0，全部已注册测试 `Passed`（输出 `100% tests passed`）。

| CTest 名称 | 测试功能/夹具 |
|---|---|
| `motion_limits_host` | 生产关节范围、连续角边界 |
| `motor_driver_host` | 虚拟驱动基本操作和异常输入 |
| `motor_feedback_safety_host` | 反馈跨圈、溢出和失败原子性 |
| `ik_selector_host` | 候选 IK 解优选与限位 |
| `protocol_stream_fault_host` | 破损/部分 UART 帧的流解析 |
| `stage5_apf_wire_host` | Stage5 C APF 协议请求/响应 |

CTest 名单与源码绑定以 [CMakeLists.txt](CMakeLists.txt) 为准。此测试不证明 FreeRTOS 任务时序、QEMU UART 传输、PyBullet 全连杆门禁或真人 GUI；后续测试入口和文件索引见 [根 README](../../../../README.md)。旧文件所称“只有三个 CTest”已经过期。
