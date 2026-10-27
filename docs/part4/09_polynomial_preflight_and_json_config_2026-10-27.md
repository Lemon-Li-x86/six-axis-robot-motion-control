# 阶段4程序补强：整段轨迹预检与JSON策略配置

本机工作树记录，2026-10-27。所有测试输出保存在`build/`；本报告不代表改动已提交或远端CI已经重跑。

## 1. 多项式轨迹的执行前检查

旧路径仅在20ms任务采样时检查位置/速度/加速度，可能漏掉相邻采样点之间的峰值。现在`trajectory_polynomial_validate_limits()`在整段归一化时间`u∈[0,1]`上检查位置、速度和加速度的端点与内部极值。内部极值通过导数根定位：按更高阶导数根划分单调区间，在区间内执行有界二分。最大多项式次数为5，根定位递归最高次数4、二分最多64步；无动态内存或全局工作缓存。

输入范围从同一Robot Model及运行配置注入：连续位置为0.01°整数，速度为°/s，加速度为°/s²。还检查系数与记录的起终点一致、系数有限、时间有效及中间计算可表示。浮点比较仅保留double舍入余量，不加入可配置运动容差。接口只验证给定多项式和运动边界，不验证碰撞或动力学。

Trajectory任务先确认计划起点与最新反馈一致，再调用整段预检。失败时写入规划故障并保持参考禁用；原有运行中采样检查继续保留。计算在较大的规划任务栈执行。`submit_polynomial()`返回成功仍只表示请求已入队，实际执行或拒绝应通过诊断确认。

## 2. 参数唯一来源

`firmware/freertos_demo/config/robot_defaults.json`新增`stage4_motion`：

| 字段 | 单位与内容 |
|---|---|
| `pid[0..5]` | 每轴独立的kp/ki/kd、积分限幅°·s、输出限幅°、死区° |
| `max_velocity_deg_s[0..5]` | 每轴正速度上限°/s |
| `max_acceleration_deg_s2[0..5]` | 每轴正加速度上限°/s² |
| `feedback_timeout_ms` | 反馈超时ms |
| `reference_timeout_trajectory_periods` | 参考超时相对于规划周期的倍数 |
| `cartesian_start_tolerance_mm` | Cartesian请求起点位置容差mm |
| `cartesian_start_rotation_matrix_tolerance` | 起点旋转矩阵最大元素差容差 |

当前数值保持原有仿真默认值：Kp/Ki/Kd=0.2/0/0，速度15°/s、加速度30°/s²、反馈超时3000ms；可按轴独立修改。构建时由`generate_firmware_defaults.py`生成C头文件，`stage4_motion_config.c`只装配生成结果。UR5几何与行程仍属于Robot Model；IK回代容差从算法接口派生，未复制到JSON。

生成器拒绝未知/缺失字段、非六轴列表、布尔或字符串数值、NaN/Inf、负增益、零输出限幅、非正速率及非法超时。实数策略参数范围0至1e6；速率/输出限幅/起点容差必须大于0，旋转矩阵差容差不超过2。反馈超时为1..60000ms，参考周期倍数为1..1000。修改JSON后必须重新构建，不在Cortex-M4运行时解析JSON。

## 3. 已运行验证

| 验证 | 结果与范围 |
|---|---|
| JSON生成器 | 10个unittest通过，含多组非法输入及各轴独立值生成 |
| Stage3/4 Host | 6个CTest通过；轨迹用例含32组边界条件×1001采样点与极值预检的对照 |
| ARM/QEMU轨迹算法 | 20项通过，含内部速度/加速度峰值、位置超界和端点元数据不一致 |
| 原有FreeRTOS任务自检 | 29项通过，健康窗口及反馈失联检查保留 |
| 新增FreeRTOS多项式路径 | 独立8项通过：越限计划发布首个参考前拒绝；随后安全计划启动；规划任务实测最小剩余**467 words**，判据不少于64 words |
| Native C/PyBullet | 单轴和六轴回归通过；JSON迁移保持原默认策略与接口 |

入口：`powershell -ExecutionPolicy Bypass -File .\tools\test_stage4_once.ps1`。原始统一日志：`build/stage4-final-validation.log`；各QEMU/GDB输出位于`build/gdb_stage4_*`。测试用自检任务栈改用规划任务的配置栈深，以容纳新增计划夹具；生产任务栈配置不变。

## 4. 尚存验证边界

全程多项式界限不等于Cartesian路径完整加速度重定时。完整工作空间的奇异穿越、碰撞、真实负载PID标定、30分钟高任务/UART负载和实物硬实时验证仍需独立实验。历史计时与内存报告中的数值属于对应测试镜像；新增测试夹具后的ELF尺寸不能沿用旧数字。
