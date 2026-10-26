# 第3阶段老师评语整改记录（2026-10-26）

## 范围与基线

- 基线：`main` 的 `f17f014b7d929eb022e7ead80da75e483b81d670`。
- 冻结备份：`backup/pre-teacher-review-2026-10-26`，本次**不得修改**。
- 工作分支：`fix/ik-review-2026-10-26`。
- 仅修改解析 IK 数值链和 QEMU 运动学测试，不改 Robot Model 几何参数、通信协议、运行态或实际执行器接口。

## 评语 1：J3 余弦越界和近奇异分支完整性

### 当前数学链

`T_base_ee` → `kinematics_public_to_dh_transform` → 肩部 `q1`
→ 腕部 `q5` → `q6` 的姿态矩阵投影 → `p13_x,p13_y`
→ `cos(q3) = (p13_x²+p13_y²-a2²-a3²)/(2a2a3)` → 两条肘部分支。

源码对应 `algorithms/kinematics/ur5_analytic_ik.c`，其中 `ur5_analytic_ik_compute_p13` 同时供 q3、q2 求解调用。

### 实施的数值修改

1. `q6` 的三角计算和 `atan2` 的旋转矩阵投影中间值改为双精度，在输出边界再转回公开接口规定的 `robot_real_t`。
2. `p13` 的加减、乘积与结果在求解过程中使用双精度，避免在数百毫米的平方和相减处过早舍入。
3. `cos(q3)` 的平方和、分母和 `sqrt(1-cos²)` 使用双精度；**没有增大**现有 `KINEMATICS_TRIG_DOMAIN_EPSILON=1e-5`。
4. q2 使用同一个双精度 `p13` 辅助结果，防止 q3 和 q2 使用不一致的几何中间值。

这是一项有针对性的数值稳健性改动，**并非已证实全部丢分支问题均由 float 精度造成**。公开位姿矩阵仍为 float，近 q5=0 时 q6 不可唯一识别；因此仍需差分证据，不得声称已穷尽修复或8解必然存在。

### 本地复测所需证据

- 编译前后同一批 near-singular 输入下的 q1、q5、q6、p13_x/y、cos_q3 和候选分支数，记录发生第一次显著偏移的步骤。
- 单独校对 `q6`：正负 q5 分支的投影、atan2 输入和 q6 值。
- 独立参考（例如 double 精度 DH / Python Robotics Toolbox）计算 `p13`，区分姿态矩阵输入误差与几何链累积误差。
- 特别覆盖 `q3≈0/±180°`、`q5≈0°`、可达边界与不可达输入。真实奇异点不能机械地要求返回8个互异解。
- 原来128组 IK 对照必须重新运行并归档误差分布，避免因修改 libm 路径引入精度或资源回归。

## 评语 2：QEMU 中 IK Selector 的端到端验证

原有 `qemu_kinematics_perf` 已连通 FK → IK → Selector，但选中目标没有 FK 回代。本次直接在**原有** QEMU 测试入口扩充：

1. 生成输入六轴关节角；FK 得到目标位姿；IK 求候选；Selector 从连续关节起点选出目标。
2. 把已选中的连续关节角规约成 canonical `[-180°,180°)`（单位仍为 0.01°），调用**同一份生产 FK 函数**回代。
3. 记录平移欧氏距离（mm）及旋转矩阵左上3×3最大元素差。门限为位置误差 ≤0.5 mm 与矩阵元素最大差 ≤0.003；**矩阵元素门限不是角度误差门限**。
4. 输出 `g_kin_perf_report.roundtrip_passed/failed/skipped`、最坏误差和 `g_kin_perf_samples[i]` 逐样本数据。被选择策略拒绝的构型单独计入 skipped，不误记为 FK 回代失败。

需要额外检查：`selection_success == roundtrip_passed + roundtrip_failed` 且 `roundtrip_failed == 0`；`roundtrip_skipped` 的原因应逐项核对，不能只看总体通过率。

### 本地运行（仓库根目录 / PowerShell）

```powershell
cmake --preset arm-qemu
cmake --build --preset arm-qemu
cmake -S firmware/freertos_demo/tests/host -B build/motion-host
cmake --build build/motion-host
ctest --test-dir build/motion-host --output-on-failure
```

运行 QEMU：

```powershell
qemu-system-arm `
  -M mps2-an386 `
  -kernel .\build\cmake-arm\firmware\freertos_demo\qemu_kinematics_perf.elf `
  -S -gdb tcp::1234 -serial null -monitor none -display none
```

另开一个终端运行 `arm-none-eabi-gdb .\build\cmake-arm\firmware\freertos_demo\qemu_kinematics_perf.elf`：

```gdb
set pagination off
target remote localhost:1234
break qemu_kinematics_perf_done
continue
p g_kin_perf_finished
p g_kin_perf_report
p g_kin_perf_samples
```

还应重新导出 QEMU FK/IK 批量样本并执行 `tools/analyze_fk_batch.py` 和 `tools/analyze_ik_batch.py`，对照原第3阶段精度阈值。

## 验收状态

截至本整改记录初稿，**尚无这批改动在用户本机 ARM 交叉编译、QEMU 执行、Host CTest 及 Python Robotics Toolbox 对照的实际运行记录**。因此状态是「源码已提交、待本地验证」，而不是「已完全验收」。尤其老师要求的逐级 p13 误差根因证据，尚需记录可复现的失败样本和差分追踪结果，不能以简单切换 double 代替根因分析。

后续正式验收前补齐：测试时间、提交 SHA、输入样本、p13 逐级误差表、解数变化、FK 回代误差、QEMU/Host 测试输出和编译器版本。


## 用户本机实测更新（2026-10-26；第4阶段开发分支的第3阶段测试目标）

用户从 Windows ARM GNU Toolchain / QEMU MPS2-AN386 环境运行 `qemu_kinematics_perf.elf`，于 `qemu_kinematics_perf_done` GDB 断点读取到以下**真实执行输出**（尚未附本机 `git rev-parse HEAD` 提交 SHA）：

| GDB 变量 | 数值 |
|---|---:|
| `g_kin_perf_finished` | 1 |
| `sample_count` | 128 |
| `repetitions` | 8 |
| `timer_frequency_hz` | 25000000 |
| `fk_success` | 128 |
| `ik_success` | 128 |
| `selection_success` | 128 |
| `roundtrip_passed` | 128 |
| `roundtrip_failed` | 0 |
| `roundtrip_skipped` | 0 |
| `worst_position_error_mm` | 0.089311257 |
| `worst_rotation_max_element_error` | 0.000167578459 |

**评语2状态：该128组 QEMU FK→IK→Selector→FK回代验证已通过**（位置门限0.5mm、旋转矩阵元素差门限0.003）；全部已选中的样本均完成并通过回代。上表记录的旋转指标是**矩阵元素差**，不是角度误差。

**评语1状态：未关闭。** 128/128 IK 成功只证明每组至少有一组可用候选，并未证明所有有效分支被正确保留。需继续做 q6 → p13 → cos(q3) 逐级差分、近奇异候选完整性和独立精度参考对照；不能将第二条通过冒充第一条也通过。

另外，此轮实测未涉及完整 PyBullet PID 闭环或 FreeRTOS 满负载周期/栈/抖动验证。


## 用户本机近奇异样本复查：2026-10-26

同一 `qemu_kinematics_perf` 运行终点处，用户在 GDB 读取了下列原始报告：

| 样本号 | J3 | J5 | 有效 IK 解数 | 最终选解 FK 回代位置误差 (mm) | 旋转矩阵最大元素差 |
|---:|---:|---:|---:|---:|---:|
| 8 | 60° | -0.01° | 8 | 0.0603962317 | 0.000167578459 |
| 9 | 60° | 0° | 4 | 0.064938955 | 0.000148564577 |
| 10 | 60° | +0.01° | 8 | 0.021943666 | 0.0000274181366 |
| 27 | 0° | -0.01° | 6 | 0.0592272617 | 0.0000225305557 |
| 28 | 0° | 0° | 2 | 0.0484741069 | 0.00000011920929 |
| 29 | 0° | +0.01° | 4 | 0.0442805737 | 0.0000222325325 |

六组 `fk_status=ik_status=selection_status=roundtrip_status=0`。样本9/28位于 q5=0° 真实腕部奇异点；样本27/28/29也取 q3=0°，接近肘部奇异。完整解数不必恰好8，且最终 FK 回代成功**不能证明所有有效 IK 分支没有遗失**。

**后续差分重点**：在相同 J3=0° 情况下，J5=-0.01/0/+0.01° 对应 `6/2/4` 个候选，出现不对称现象。仅凭数量不能判定是 q6 近奇异值、q3 余弦越界还是0.01°量化去重造成。因此新增**独立诊断目标** `qemu_ik_near_singular_trace`，仅这个目标编译 `ROBOT_KINEMATICS_ELBOW_TRACE=1`；正式固件编译行为、原有求解阈值和分支数学不变。

该诊断直接复用生产 IK 的 `ur5_analytic_ik_compute_p13()`，将6组样本拆分成两个 shoulder 分支和各自两个 wrist 分支，记录：
- q1、q5 的角度和求解状态；
- q6 的输出角度及其是否因 `KINEMATICS_SINGULARITY_EPSILON` 拒绝；
- 生产公式计算的 `p13_x_mm`、`p13_y_mm` 与**裁剪前** `raw_cos_q3`；
- q3 求解状态及两条 q3 候选弧度；最终公共 IK 解数。

GDB 操作（在仓库根目录，先关闭原 QEMU 以释放 1234 端口）：

```powershell
git fetch origin
git switch feat/stage4-trajectory-2026-10-26
git merge --ff-only origin/feat/stage4-trajectory-2026-10-26
cmake --preset arm-qemu -DROBOT_ENABLE_STAGE4_TASKS=ON -DROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF
cmake --build --preset arm-qemu --target qemu_ik_near_singular_trace

qemu-system-arm `
  -M mps2-an386 `
  -kernel .\build\cmake-arm\firmware\freertos_demo\qemu_ik_near_singular_trace.elf `
  -S -gdb tcp::1234 -serial null -monitor none -display none
```

在另一个 PowerShell 中：

```powershell
arm-none-eabi-gdb .\build\cmake-arm\firmware\freertos_demo\qemu_ik_near_singular_trace.elf
```

```gdb
set pagination off
target remote localhost:1234
break qemu_ik_trace_done
continue
p g_ik_trace_done
p g_ik_trace[3]
p g_ik_trace[4]
p g_ik_trace[5]
```

`g_ik_trace[0..5]` 分别映射原样本 `8,9,10,27,28,29`。优先查看3、4、5，若要比较 J3=60° 的对照组再读0、1、2。

**必须保留待验收状态**：未有上述诊断的实际值前，不判定 6/2/4 是程序缺陷或真实退化；上述诊断记录尚不是独立 double 高精度参考实现，最终还要对已定位的可疑分支与独立几何解进行比较。
