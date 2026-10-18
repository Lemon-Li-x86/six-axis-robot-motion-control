# 第3阶段：算法精度与性能对比测试报告

## 1. 验证目的

本报告用于验证六轴工业机器人嵌入式运动控制项目第3阶段实现的 UR5 正运动学、解析逆运动学及 IK 候选解筛选策略。

根据项目任务书，本阶段验证目标包括：

- 使用 Python Robotics Toolbox 作为参考数学计算工具，设计不少于 100 组测试姿态；
- 比较 Cortex-M4 C 实现与参考工具的末端位置误差、姿态误差；
- 检查 IK 几何求解状态、返回候选数量与候选解有效性；
- 验证关节限位与逆解选择策略的典型边界行为；
- 测量 QEMU 仿真环境下 FK、IK 和选解器的耗时与成功率；
- 统计嵌入式 ELF 的静态代码与数据尺寸。

验证按**精度对照、选择器功能回归、性能计时、资源统计**四类分别开展，不能将某一类测试的 `PASS` 替代其他类别的证据。

---

## 2. 测试环境

本阶段继续沿用第1阶段环境验证报告与第2阶段驱动接口文档中的主机及目标平台。

| 类别 | 配置 |
|---|---|
| 主机操作系统 | Windows |
| 开发环境 | Visual Studio Code / PowerShell |
| 目标架构 | ARM Cortex-M4 |
| 仿真设备 | QEMU `mps2-an386` |
| 编译工具链 | ARM GNU Toolchain / `arm-none-eabi-gcc` |
| 构建工具 | CMake + Ninja，`arm-qemu` Preset |
| 数学实现 | C 语言、`float`、标准 C 数学库 `libm` |
| 基准工具 | Python Robotics Toolbox (`rtb.models.DH.UR5()`) |
| 外部数学计算 | NumPy、SciPy、SpatialMath |
| GDB 调试工具 | `arm-none-eabi-gdb` |
| 计时器 | CMSDK APB Timer0，名义 25 MHz |
| 性能测试编译选项 | `-O2`（专用测试目标，非对正式固件全体配置的断言） |

Robotics Toolbox 的经典 UR5 模型按照 `ur5_model.c` 对齐几何参数、基座与工具坐标系转换：`robot.links[0].d = 0.089159`，`robot.base = SE3.Rz(pi)`，`robot.tool` 使用工程指定的固定旋转矩阵。所有位置量统一换算为毫米后再比较。

---

## 3. 测试对象、程序及判定标准

| 测试类别 | Cortex-M4 入口 / 工具 | 核心判据 |
|---|---|---|
| FK 精度 | `qemu_fk_batch.elf` + `tools/analyze_fk_batch.py` | 128 组样本；位置和旋转最大误差在设定范围内 |
| IK 精度 | `qemu_ik_batch.elf` + `tools/analyze_ik_batch.py` | 128 组；所有返回解的位姿重建误差达标 |
| IK Selector | `ik_selector_host_test` / `qemu_kinematics_perf.elf` | 输入有效性、行程/限位/奇异处理及有效候选选择 |
| 计算性能 | `qemu_kinematics_perf.elf` | 128 组样本，每组重复 8 次 |
| 静态资源 | `arm-none-eabi-size` | `text/data/bss` 大小可测、可追溯 |

当前测试阈值：

| 测试 | 位置误差上限 | 姿态误差上限 |
|---|---:|---:|
| FK | 0.05 mm | 0.05° |
| IK 候选位姿重建 | 0.5 mm | 0.15° |

这两组阈值是本项目当前阶段的仿真验收条件，不应等同于工业机器人实物重复定位精度规范。

---

## 4. Forward Kinematics 精度验证

### 4.1 测试方法

`qemu_fk_batch.elf` 使用固定随机种子 `20261012` 生成 128 组输入，其中包括零位姿、单轴基准姿态、接近腕部奇异的姿态以及伪随机关节配置。每组由 `kinematics_forward()` 输出 4×4 矩阵。

二进制结果通过 GDB 从 `g_fk_samples` 导出到：

```text
build/fk_validation/fk_samples.bin
```

每条记录 80 字节，共 10,240 字节。`tools/analyze_fk_batch.py` 将 C 端矩阵与 RTB 对同一组输入的末端矩阵比较。

位置误差与旋转误差定义为：

$$
e_p=\|p_{\mathrm{C}}-p_{\mathrm{RTB}}\|_2\quad(\mathrm{mm})
$$

$$
e_R=\operatorname{angle}(R_{\mathrm{RTB}}^\mathsf{T}R_{\mathrm{C}})\quad(\mathrm{degree})
$$

### 4.2 验证结果

2026-10-18 在本机 `.venv` 中执行 `tools/analyze_fk_batch.py`，将 `build/fk_validation/fk_samples.bin` 中的 128 组输出与 Python Robotics Toolbox 参考矩阵比较。脚本返回：

```text
RESULT: PASS
```

| 指标 | 平均误差 | 最大误差 | RMSE |
|---|---:|---:|---:|
| 末端位置（mm） | 0.000074035 | 0.000193276 | 0.000083015 |
| 末端姿态（°） | 0.000010812 | 0.000030916 | 0.000012234 |

- 最大位置误差位于样本 21，输入关节角原始值 `[14618, 11915, -2512, 12431, -3594, 862]`；未截断的位置误差为 `0.000193276039207672 mm`。
- 最大姿态误差位于样本 102，输入关节角原始值 `[15702, 9497, -15443, 6991, -14188, 10226]`；未截断的姿态误差为 `3.0916052652878267e-05°`。
- 两项最大误差均小于第 3 节定义的本阶段阈值，128 组样本的 FK 精度验证通过。

原始终端输出归档为：

```text
docs/testing/results/stage3_fk_validation.txt
```

以上是数学模型对照结果，不代表机械臂实物定位精度。

---

## 5. Inverse Kinematics 精度验证

### 5.1 测试方法与样本分布

`qemu_ik_batch.elf` 共使用 128 组确定性姿态，按当前测试源码分为三组：

1. 19 组：`J3 = +60°`，J5 从 -5° 到 +5° 取特定值；
2. 19 组：`J3 = 0°`，J5 使用相同扫描值；
3. 90 组：固定随机种子生成的六轴姿态。

每组先由 FK 产生目标位姿，再调用解析 IK。导出的结构记录包括原始关节角、FK/IK 状态、解数量及最多 8 组候选关节角。记录大小为 120 字节，总量 15,360 字节。

```text
build/ik_validation/ik_samples.bin
```

分析脚本 `tools/analyze_ik_batch.py` 把每一组 C IK 候选代入 Robotics Toolbox 的 FK，比较它与参考目标的末端位姿。因此检验的是**逆解位姿重建误差**，不能对不同几何分支的关节角逐项相减来判断 IK 对错。

### 5.2 求解状态与候选解分布

| 指标 | 实测 |
|---|---:|
| 姿态总数 | 128 |
| 求解成功 | 128 |
| 求解失败 | 0 |
| `ik_status = 0` | 128 |
| 返回候选总数 | 883 |
| 超出精度限值的候选 | 0 |

各姿态返回候选数量分布：

| 候选解数/姿态 | 姿态组数 |
|---:|---:|
| 2 | 1 |
| 4 | 25 |
| 5 | 5 |
| 6 | 10 |
| 8 | 87 |
| **总计** | **128** |

### 5.3 位置误差与姿态误差

| 指标 | 平均误差 | 最大误差 | RMSE |
|---|---:|---:|---:|
| 位置（mm） | 0.030231 | 0.105016 | 0.037447 |
| 姿态（°） | 0.003830 | 0.017359 | 0.005550 |

逐姿态最小误差候选的均值：位置 `0.003313 mm`；姿态 `0.000047°`。这里最小误差候选由分析脚本分别取最小值，**不代表选解器从运动行程、限位、奇异风险选择的候选**。

本次完整分析结论：

```text
RESULT: ALL CASES WITHIN TOLERANCE
```

分析脚本输出的 `build/ik_validation/ik_results.csv` 可用作逐候选误差的归档证据。

---

## 6. IK 最优解筛选功能验证

### 6.1 验证对象

选解器 `ik_solution_selector_select()` 从 `kinematics_inverse()` 返回的候选中，执行规范角合法性验证、连续角展开、行程与限位评价、肘部/腕部近奇异风险筛选，并返回连续位置目标。

对应 Host 测试：

```text
firmware/freertos_demo/tests/motion/ik_solution_selector_test_main.c
```

关键边界包括：

- NULL Pointer、零候选以及超过 8 组候选；
- 输入角范围非法或当前连续关节位置越限；
- 穿越 ±180° 时选择正确的连续等效角；
- 比较总行程与靠近限位的罚分；
- q3 / q5 近奇异候选的拒绝和降级；
- 全部候选被拒时的状态码，以及失败时输出保持不变。

### 6.2 已获得结果

本地 Windows Host CTest 在 2026-10-18 的输出为：

```text
motion_limits_host                 Passed
motor_driver_host                 Passed
motor_feedback_safety_host        Passed
ik_selector_host                  Passed

100% tests passed out of 4
Total Test time (real) = 0.08 sec
```

该结果说明本轮四个 Host 测试目标全部通过，其中 `ik_selector_host` 覆盖了选解器的输入检查、连续角转换、边界与近奇异处理等人工构造的回归用例。测试证据保存路径：

```text
docs/testing/results/stage3_host_ctest.txt
```

此外，ARM/QEMU 性能测试中 `ik_solution_selector_select()` 的 128 组姿态全部成功：

```text
selection_success = 128
```

该结果证明当前这 128 组样本都有可接受候选，不代表所有关节极限和奇异位形都已覆盖。选解器目前使用 q3、q5 的 `|sin(q)|` 启发式，尚未独立覆盖肩部奇异或完整 Jacobian 奇异值指标。

---

## 7. QEMU 求解耗时验证

### 7.1 测量方法

`qemu_kinematics_perf.elf` 在 ARM Cortex-M4 的 `mps2-an386` 模型内运行，使用 CMSDK APB Timer0。输入 128 组样本，每组分别重复 FK、IK、选解器各 8 次，记录每组平均 Tick，再统计 128 组的最小、最大和平均值。

计时器名义频率：

```text
25,000,000 Hz
1 tick = 40 ns (QEMU emulated timer reference)
```

换算公式：

$$
t_{\mu s}=\frac{\mathrm{Timer\ ticks}}{25}
$$

测试目标单独设置 `-O2`；原有独立 QEMU 测试配置为 `-O0`。当前性能统计包含计时器读取开销，没有对结果做未经验证的补偿；编译器对重复调用的优化与仿真调度也可能影响测量。

### 7.2 Timer0 自检

| 项目 | 实测 |
|---|---:|
| `timer_frequency_hz` | 25,000,000 |
| `timer_probe_ticks` | 8510 |
| `timer_read_pair_ticks` | 242 |
| 连续两次计时器读取换算 | 9.68 μs |
| 均摊到每次算法调用的计时读取开销（估计） | 约 1.21 μs |

探针计数大于 0，说明 QEMU 模型中的 Timer0 正在推进。这里的 Tick **不等于已经标定的实际 Cortex-M4 CPU Cycle**。

### 7.3 计算性能结果

| 模块 | 成功/样本 | 最小 ticks | 平均 ticks | 最大 ticks | 最小 μs | 平均 μs | 最大 μs |
|---|---:|---:|---:|---:|---:|---:|---:|
| FK | 128/128 | 1109 | 1265.13 | 1827 | 44.36 | **50.61** | 73.08 |
| IK | 128/128 | 3511 | 7963.17 | 11112 | 140.44 | **318.53** | 444.48 |
| IK Selector | 128/128 | 394 | 1257.58 | 4326 | 15.76 | **50.30** | 173.04 |

原始 Tick 统计和：

```text
FK:          sum_ticks = 161937
IK:          sum_ticks = 1019286
IK Selector: sum_ticks = 160970
```

平均值按总 Tick 除以 128 组计算，时间除以 25 换算到微秒。IK 平均计时约为 FK 的 6.29 倍，主要与多分支解析过程及更多三角函数运算相符；这里不是基于真实硬件性能计数器的微架构结论。

各模块的“最大值”只代表**这次 128 组测试中的最大观测值**，不能直接称作 WCET（最坏情况执行时间）。

---

## 8. 内存与 ELF 资源统计

ARM GNU Toolchain 的 `arm-none-eabi-size` 输出：

| ELF | text (B) | data (B) | bss (B) | 合计 (B) |
|---|---:|---:|---:|---:|
| `qemu_kinematics_perf.elf` | 16,352 | 0 | 3,716 | 20,068 |
| `freertos_demo.elf` | 44,188 | 8 | 33,500 | 77,696 |

正式固件的 `text + data = 44,196 B ≈ 43.16 KiB`，`data + bss = 33,508 B ≈ 32.72 KiB`。

说明：

1. 这两组数据是**整 ELF** 而非单个运动学模块；两种 ELF 包含的启动与测试内容不同，不宜直接相减来估计算法开销；
2. `text/data/bss` 无法反映最大调用栈深度、任务栈水位或动态分配峰值；
3. 当前测试的 QEMU RAM 映射不是实际 STM32 的内存参数，不能以此断言目标实物一定有足够资源。

---

### 8.1 编译器静态栈帧分析

在 ARM 编译配置中临时开启 `-fstack-usage`，重新构建 `qemu_kinematics_perf` 后读取 `.su` 文件，获得以下函数级栈帧记录：

| 函数 | 栈帧大小（B） | 分析类型 |
|---|---:|---|
| `kinematics_forward` | 248 | static |
| `kinematics_inverse` | 312 | static |
| `ik_solution_selector_select` | 168 | static |
| `kinematics_public_to_dh_transform` | 80 | static |
| `matrix4_multiply` | 120 | static |
| `motion_limits_unwrap_nearest` | 88 | static |
| `ur5_analytic_ik_solve_q2` | 56 | static |
| `ur5_analytic_ik_solve_q4` | 48 | static |

`static` 是 GCC 栈分析报告给出的“静态可确定”分类，并不表示函数以 C 语言 `static` 关键字声明。

例如 `kinematics_inverse → kinematics_public_to_dh_transform → matrix4_multiply` 这条可见调用路径的已报告帧大小合计为 `312 + 80 + 120 = 512 B`，**但这一合计不是已验证的最大栈深度**：其他调用链、优化后的实际调用关系、`libm` 内部调用、ISR 和 FreeRTOS 任务栈均未完成整体上界分析。不能据此声称不存在栈溢出风险。

栈分析记录保存于：

```text
docs/testing/results/stage3_stack_usage.txt
```

这批数值来自专用性能测试目标的编译配置，不能直接替代正式运行中 FreeRTOS 任务栈水位测量。

---

## 9. 构建与回归验证

### 9.1 Windows Host 测试

执行命令：

```powershell
cmake -S firmware/freertos_demo/tests/host -B build/motion-host
cmake --build build/motion-host --config Debug
ctest --test-dir build/motion-host -C Debug --output-on-failure
```

现有目标包含原有 3 项 Host Test 和新增 `ik_selector_host`。2026-10-18 的本地终端记录显示 **4/4 Passed**，已保存至 `docs/testing/results/stage3_host_ctest.txt`。

### 9.2 ARM Cortex-M4 构建

```powershell
cmake --preset arm-qemu
cmake --build --preset arm-qemu
```

本地已反馈完成构建；性能测试对应 `qemu_kinematics_perf.elf` 也已成功生成并在 GDB 中运行完毕。

### 9.3 FK/IK 精度数据复现

在 QEMU/GDB 中导出相应二进制内存记录后：

```powershell
.\.venv\Scripts\python.exe tools\analyze_fk_batch.py
.\.venv\Scripts\python.exe tools\analyze_ik_batch.py
```

FK 控制台全文已保存至 `docs/testing/results/stage3_fk_validation.txt`。为保证可复现性，还应保留 IK 控制台输出、`build/ik_validation/ik_results.csv` 与样本生成器固定随机种子的记录；`build/` 目录中的产物不一定随 Git 提交。

### 9.4 QEMU 性能数据复现

启动 `qemu_kinematics_perf.elf`，通过 GDB 设置 `break qemu_kinematics_perf_done`，在断点执行：

```gdb
p g_kin_perf_finished
p g_kin_perf_report
```

本次记录 `g_kin_perf_finished = 1`、`timer_probe_ticks = 8510`，表示整个测试程序执行到完成断点。

### 9.5 静态资源分析

```powershell
arm-none-eabi-size .\build\cmake-arm\firmware\freertos_demo\qemu_kinematics_perf.elf
arm-none-eabi-size .\build\cmake-arm\firmware\freertos_demo\freertos_demo.elf
```

建议将上面所有原始终端输出保存到 `docs/testing/results/`，并记录代码提交 SHA 和工具链版本，保证第三阶段数据可复现。

---

## 10. 测试结果汇总

| 测试项 | 当前记录 | 结论 |
|---|---|---|
| FK 128 组 RTB 对照 | 128/128；最大位置误差 0.000193276 mm，最大姿态误差 0.000030916° | 通过；详细数据已归档 |
| IK 128 组 RTB 对照 | 128/128，883 个候选，0 个超差 | 通过 |
| IK Selector 本地功能实现 | Windows CTest 4/4 Passed | 通过 Host 回归 |
| ARM Cortex-M4 构建 | 本地构建成功 | 通过 |
| FK/IK/Selector 性能基准 | 均 128/128 成功，已记录 Tick 与 μs | 通过 QEMU 仿真基线 |
| ELF 静态尺寸 | 两个 ELF 均有 `text/data/bss` 输出 | 已统计 |
| ARM GCC 静态栈帧 | FK 248 B、IK 312 B、Selector 168 B（入口函数，不含调用链总深度） | 已记录；完整最大栈深度未验证 |
| 实物 MCU 最坏执行时间 | 无测量 | 未验证 |
| 实物 MCU 最大栈使用量 | 无测量 | 未验证 |
| 肩部奇异性全覆盖 | 当前选择器没有独立分析 | 未完成 |

---

## 11. 当前验证边界

本阶段已经建立算法实现与数学精度对照关系，但以下内容不属于已验证结果：

1. 全可达工作空间穷尽性验证和所有病态/不可达姿态的完整覆盖；
2. 完整 Jacobian 奇异性指标及肩部奇异性的独立识别；
3. 真实 Cortex-M4 芯片执行时间、任务争用场景 WCET 与控制周期适配；
4. 完整调用链最大栈占用、FreeRTOS 任务栈水位与动态/静态堆资源峰值（已记录核心函数的静态栈帧）；
5. 关节整条运动过程的碰撞、速度、加速度、动力学和执行器安全策略；
6. 选解器输出的选中目标，在独立批量测试中进行 FK 回代精度核验（当前 IK 候选的 RTB 回代检验与选解器功能测试分别完成）。

特别是选解器返回**某一组合法的连续目标**，不代表机器人可以不经轨迹规划就安全到达此目标。

---

## 12. 验证结论

第3阶段完成了经典 UR5 FK 与解析 IK 的 C 语言实现，并建立了 IK 多候选选择策略。现有数值对照表明，在所使用的 128 组测试姿态内：

- FK 128 组通过；最大位置误差 `0.000193276 mm`，最大姿态误差 `0.000030916°`，平均值与 RMSE 已记录；
- IK 128/128 求解成功，883 个返回候选解全部处于当前精度阈值内；
- FK、IK 与 IK Selector 的 QEMU 性能测试均为 128/128 调用成功；
- 已取得 QEMU 仿真计时、对应 ELF 静态段大小以及核心函数的 GCC 静态栈帧数据；
- Windows Host CTest 四项测试全部通过。

因此可以支持第3阶段**当前仿真与算法功能范围内**的验收结论，但不可据此声称实物实时控制、安全性、最坏栈使用或完整奇异性分析已经完成。

本阶段交付物由嵌入式 FK/IK 与选解器源码、`01_kinematics_derivation.md` 和本报告共同组成，作为第4阶段关节空间轨迹、笛卡尔插补及闭环控制研发的基础。
