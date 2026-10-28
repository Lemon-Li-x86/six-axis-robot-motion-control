# Stage 5 · ARM 固件链接内存基线（任务 5.3）

状态：**ARM 固件真实链接基线已采集并保存 CI 原始证据**，见 `docs/part5/04_arm_link_memory_baseline_2026-10-28.md`；动态 FreeRTOS 任务 CPU/栈及优化前后实验仍待完成。

## 统计范围

tools/stage5_memory_map.py 读取 GNU ld 的 Memory Configuration 和顶层输出段，区分 FLASH、RAM 以及 RAM 初始化数据在 FLASH 内的加载副本。输入子段不重复计数，未装载的调试段不计入占用；内存区域不完整、分配越界和重叠时显式报错。

这是链接期静态分配，不是动态任务 CPU 占比、堆峰值、FreeRTOS 栈水位、WCET 或实体硬件内存测量。当前 linker.ld 定义的是 QEMU MPS2-AN386 虚拟存储布局。

默认构建不变。只有显式打开 ROBOT_ENABLE_STAGE5_LINK_MAP=ON，才额外生成 stage5_firmware.map。此开关与 Stage4 QEMU Motion Self-Test 专属 map 构建互斥。

## Windows PowerShell / 仓库根目录

~~~powershell
python -m unittest discover -s tools/tests -p test_stage5_memory_map.py -v
cmake --preset arm-qemu -DROBOT_ENABLE_STAGE5_LINK_MAP=ON -DROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF
cmake --build --preset firmware
python tools/stage5_memory_map.py build/cmake-arm/firmware/freertos_demo/stage5_firmware.map --output build/stage5-memory-baseline.json
~~~

记录源 commit SHA、链接脚本、ARM 工具链版本、优化选项、CMake 开关、ELF、原始 map 与 JSON；构建未成功时不得填写测量数字。

比较两个版本（路径以实际构建位置为准）：

~~~powershell
python tools/stage5_memory_map.py build/new/firmware/freertos_demo/stage5_firmware.map --before build/old/firmware/freertos_demo/stage5_firmware.map --output build/stage5-memory-delta.json
~~~

比较必须保持链接脚本、工具链、优化参数、任务开关一致。delta_bytes 是“新减旧”，负数表示减少。Stage4 专属自检 map 编译参数不同，不能直接和 Stage5 Live 内存数据比较。

## 后续门禁

- 单元测试只验证解析器与人造 fixture，不代表 ARM ELF 已构建；
- 完整 5.3 仍需 FreeRTOS 任务运行占比、控制周期 P99/最大值、栈余量及长时仿真测量；
- 只给出可复现数据，不把 QEMU 仿真资源当作实体机器人指标。
