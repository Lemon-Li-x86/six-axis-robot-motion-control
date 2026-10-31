# 项目原始大纲逐项进度审计与 Stage5 收尾边界（2026-10-31）

> **审计基准**：用户提供的四页 PDF《六轴工业机器人嵌入式运动控制固件开发与仿真验证 实习项目大纲》；同一仓库的可访问源代码、测试报告与 GitHub Actions 证据。**项目实际执行范围是 ARM Cortex-M4/QEMU + FreeRTOS + 虚拟 UART + PyBullet UR5，不存在实体电机/六轴机器人。**
>
> **判定词**：`通过（仿真范围）` 表示有对应的软件功能与测试证据；`部分` 表示只有核心功能或缺少原 PDF 明确要求的一环；`未验收` 不等于代码不存在，而是缺乏足以签收的证据。不要以 GitHub 绿灯替代逐项要求。

## 1. 原 PDF 的 24 个核心任务逐项对照

| 原 PDF 条目 | 任务要求（忠实归纳） | 源码/验证与缺口 | 判定 |
|---|---|---|---|
| **1.1** | 调研分层架构并形成 Markdown 接口与技术方案 | `docs/part1/01_system_design.md`、`docs/architecture.md` 和 `firmware/freertos_demo/` 分层 | 通过（仿真范围） |
| **1.2** | Cortex-M4、arm-none-eabi-gcc、CMake、QEMU、FreeRTOS v10.x、调度/时钟/GPIO | QEMU `mps2-an386` 工程和任务调度已有本机/CI 回归；不等同于实体 STM32 | 通过（仿真范围） |
| **1.3** | PyBullet UR5、视角/环境、单关节点动基础演示 | URDF 和 PyBullet 物理基础演示，以及 Stage4 DIRECT 闭环 | 通过（仿真范围） |
| **1.4** | 虚拟 UART 双向二进制帧通信与连通性 | 真实 QEMU 串口、C 解析及 Python 对端双向通信；异常帧回归 | 通过（仿真范围） |
| **2.1** | STM32 HAL/C 驱动：UART、Timer、GPIO、中断与标准接口 | QEMU CMSDK APB UART/Timer 为寄存器级驱动，GPIO 采用 Shadow Backend；**没有 STM32 HAL 板级端口** | **部分：明确平台替代** |
| **2.2** | 指令/参数/状态协议解析与实时上传 | `communications/`、`protocol_tasks`、虚拟串口联调及 C/ARM 回归 | 通过（仿真范围） |
| **2.3** | 六关节伺服抽象、目标/编码器/速度接口 | `drivers/motor_driver.[ch]` 为六轴模拟电机/反馈接口；非真实编码器/伺服 | 通过（模拟 Backend） |
| **2.4** | Unity 对正常、边界、异常驱动/通信功能回归 | Stage1–4 复核记录含 Unity **25/25**、ARM/QEMU UART/电机/GPIO 回归 | 通过（已覆盖样本） |
| **3.1** | UR5 DH/FK/IK 推导及多解析解、限位 | `docs/part3/`、`model/`、生产 C 解析 IK 与选解器 | 通过（已覆盖模型） |
| **3.2** | C FK/IK、矩阵/数学运算、内存资源约束 | `algorithms/kinematics/`、ARM/QEMU 验证和静态资源测量 | 通过（已覆盖模块） |
| **3.3** | IK 最优筛选：行程、关节约束和奇异边界报错 | `ik_solution_selector`、`motion_limits` 和奇异性回归；未证明全工作空间及所有奇异工况 | 通过（指定工况） |
| **3.4** | Python Robotics Toolbox **≥100 组**误差与性能对比 | Stage1–4 验收报告记录 RTB **128 组**，含 FK/IK、QEMU 回代 | 通过（已覆盖样本） |
| **4.1** | 三/五次多项式、梯形速度、速度/加速度约束 | `algorithms/trajectory/`、ARM/Host 单测和整段边界预检 | 通过（指定工况） |
| **4.2** | 笛卡尔直线/圆弧、IK 转换、周期插补 | C Cartesian 规划与 FreeRTOS 周期执行已测 | 通过（指定工况） |
| **4.3** | 单关节增量式 PID、限幅/死区、仿真反馈整定 | 六轴 `pid`、`joint_command_limiter` 和 PyBullet 闭环；Stage4 参数研究 **18/18 可执行**，并非所有扩展负载/超调工况都满足门限 | 核心通过，非最坏条件证明 |
| **4.4** | FreeRTOS Trajectory/Control/Communication/Monitor、多任务同步 | `application/tasks/`、队列、周期任务回归 | 核心通过 |
| **5.1** | 驱动→协议→运动学→规划→PID→反馈；Git 问题可追溯 | Cortex-M4/QEMU 生产 C 轨迹/PID、UART 虚拟传动、PyBullet 实际反馈；历史静态 C APF **4/4** 与 **28/28** | 核心通过 |
| **5.2** | PyBullet 静态长方体；**C APF 嵌入固件路径规划模块**；运动中避障 | 生产 C APF 与 IK/FK 经**主机同源 C 动态库**规划并验证全臂离散碰撞，航点交给 FreeRTOS UART 执行。**缺少 FreeRTOS 运行时接收障碍输入并自主调用 APF 编排**，不符合原文全部条件 | **部分（架构偏离，不能无条件签收）** |
| **5.3** | 实测任务 CPU、优化高耗时路径、map 内存裁剪、IRQ/调度抖动优化 | 已有 QEMU/GDB 短时 CPU/P99/栈水位、同配置 map A/B；APF 路径 28→4 有几何压缩证据。但 **FLASH +48B、RAM 未减少**；没有证据表明 IRQ 优先级调整/全系统 CPU 优化取得目标效果、也没有满载 WCET | **部分：测量完成，优化成效未完整证实** |
| **5.4** | GDB+QEMU/UART 定位时序、丢包、溢出、死机并归档 | `docs/part5/02_integration_issue_log.md`、已修正串口帧恢复/时间戳竞态；Windows 实际 SAFE JOG J6+0.5°、真实回位 1/1；仍有 [S5-010 #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34) 和未确定根因的历史 GUI 无响应 | 核心通过，问题未全部归零 |
| **6.1** | 原点→抓取→避障→放置→返回，多场景 | `docs/part6/00_stage6_interface_and_acceptance_contract.md` 已有接口规划，**尚无完整可验收应用任务** | 未完成 |
| **6.2** | 功能/边界/异常/长时维度和成功率、误差统计 | 前阶段积累大量组件测试，但没有 Stage6 完整搬运任务的全量成功率和连续运行报告 | 未完成 |
| **6.3** | 开发手册、系统测试报告、用户使用手册、标准注释 | 阶段文档已有，最终面向用户的完整交付手册尚未验收 | 部分 |
| **6.4** | 固件/仿真/文档/视频交付包及总结 | 尚未形成经过签收的结构化 Stage6 最终交付包和演示视频 | 未完成 |

## 2. Stage5 实际功能已验证的范围

1. **C APF/IK/几何**：具名静态障碍的整臂离散预检；同源 C APF 经宿主规划，缩短路径 4/4 与原始 28/28 都有独立逐段 UART/物理到位证据。并不构成连续扫掠碰撞证明。
2. **ARM 固件闭环**：FreeRTOS Control/Trajectory/Protocol 任务、生产 C PID 和真实 QEMU 串口反馈；不是 Python 伪造的 PID 轨迹，也没有连接真实电机。
3. **本地 GUI**：用户上传的 Windows SAFE JOG 证据中 J6 +0.5°、C/UART 完整回位 1/1、最大 HOLD 误差 0.02°，零固件故障；后续 Windows 原生鼠标与 FINISH 再次 PASS。见 `docs/part5/16_safe_jog_click_review_2026-10-31.md`。
4. **跨平台 CI**：代码版 `47b4acecfadf10cb09c16c0f3cd0885a71e98d56` 对应 [Actions #41206909042](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41206909042)，**8/9 任务 PASS**。Linux scripted native GUI C APF/J6 PASS，Windows C DLL、C 回位、Windows/Linux UART、固件与单元回归也 PASS。
5. **性能测量**：2026-10-30 Stage5 结项报告有 QEMU CPU/Timer0/栈水位和同配置 map 的原始数据、时间窗口及限制，不能直接外推硬件。APF 28→4 是可复核的航点压缩，不代表整个 CPU 占用或中断时延改善。

## 3. 尚未达到“原 PDF Stage5 全部完成”的三条门禁

| 门禁 | 当前事实 | 不可采取的伪修复 | 解除条件 |
|---|---|---|---|
| **G5-A：固件端 C APF 编排** | 同源 C 算法被编译/链接，但当前静态场景的 APF/IK 整路规划由宿主执行 | 在演示说明中将宿主规划描述为固件自主避障 | 明确批准**宿主 C 规划+固件闭环执行**作为需求变更，或者完成固件端运行时 APF 请求/生成/拒绝、资源/时序和串口端到端验收 |
| **G5-B：优化成效** | CPU/P99/栈水位/map 有观测，FLASH 同条件 +48B、RAM 0B；缺任务书所述全部优化成效 | 把“测过”写成“已降低 CPU/内存/抖动” | 固定同一构建/场景 A/B 采样，区分航点压缩收益与 ISR/CPU/内存收益；无法减小的指标写事实并决定范围 |
| **G5-C：legacy GUI `six_axis`** | [Issue #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34) 的 Linux Xvfb 首段 **0/3**，最终指令/仿真实际皆在请求值外约 0.24°，严格到位门限仍为 0.15° | 放宽容差、延长超时、删除红色测试、伪造回位或盲调 PID | 原 0.15° 门限下真 C/UART/物理收敛 **3/3** 且全 CI 通过，并查清命令平台误差；或者在明确批准的有界范围发布中继续公开保留 Issue |

**G5-C 的新定位信息**：通过失败 CI 工件 `stage5-gui-nodrag-xvfb` 的 `joint_trace.csv`，可见末端 `command=servo_target=actual` 均停在 J1 38.24° 等相应角度，第一段请求目标为 J1 38.00°。这说明 PyBullet 确实追到了**固件发布的最终电机命令**，但该电机命令未收敛到 **请求的段目标**；并非图形窗口冻结的证据。PID/限幅/参考时间链尚需分离测量后再确定根因。

## 4. 结项、冻结与后续开发决策

- **可以归档**：Stage5 的静态场景软件仿真核心、Windows 受控点动和对应实测分析，维持 **“有保留技术结项”** 的措辞；这不等于原 PDF 的无保留全项验收。
- **不能自动宣称**：固件端 APF 自主规划已完成、CPU/FLASH/IRQ 全面优化、所有 GUI 模式通过、实体机器人安全认证，以及 Stage6 搬运任务已完成。
- **PR/版本边界**：`fix/stage5-safe-jog-small-step-hold-20261031` 的 [PR #33](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/33) 仍 Draft、未合并；原 Stage4 备份与 `main` 保持不动。无条件 Release/Tag 不能在红灯和明确需求偏离时创建。
- **Stage6 设计可先行**：以已验证的 QEMU C/UART/物理运动执行 API 设计应用层任务状态机，要求虚拟抓取/放置可观测；但若用户坚持原 PDF 精确范围，G5-A/G5-B/G5-C 是 Stage5 正式封版的未清门禁，不得偷换成 Stage6 新任务。

## 5. 证据导航

- [Stage 1–4 汇总](phase1-4_acceptance_status_2026-10-27.md)
- [Stage 3 Robotics Toolbox 误差报告](part3/02_algorithm_validation_report.md)
- [Stage 4 Release Candidate](part4/part4_archive/12_stage4_release_candidate_2026-10-28.md)
- [Stage 5 5.1–5.4 性能/故障/静态避障证据](part5/15_stage5_exit_review_2026-10-30.md)
- [Stage 5 有保留结项与 Stage6 交接](part5/17_stage5_final_acceptance_2026-10-31.md)
- [Windows SAFE JOG 鼠标验证](part5/16_safe_jog_click_review_2026-10-31.md)
- [Stage 6 接口与验收契约](part6/00_stage6_interface_and_acceptance_contract.md)

**审核结论**：第一至第三阶段仿真核心有明确验收、第四阶段核心通过但封版 RC，**第五阶段不是“全部结束”，是核心已实现的有保留结项**，第六阶段任务型应用、全量系统测试与交付未完成。此结论同时符合源 PDF 的技术条款与现有仓库证据，不靠假设把缺口补成 PASS。
