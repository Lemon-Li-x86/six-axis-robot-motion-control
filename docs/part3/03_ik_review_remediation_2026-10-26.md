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

截至本次远程修改，**尚无这批改动在用户本机 ARM 交叉编译、QEMU 执行、Host CTest 及 Python Robotics Toolbox 对照的实际运行记录**。因此状态是「源码已提交、待本地验证」，而不是「已完全验收」。尤其老师要求的逐级 p13 误差根因证据，尚需记录可复现的失败样本和差分追踪结果，不能以简单切换 double 代替根因分析。

后续正式验收前补齐：测试时间、提交 SHA、输入样本、p13 逐级误差表、解数变化、FK 回代误差、QEMU/Host 测试输出和编译器版本。
