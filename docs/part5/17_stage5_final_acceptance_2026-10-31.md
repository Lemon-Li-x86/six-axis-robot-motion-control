# Stage 5 系统集成与功能扩展：有保留结项与 Stage 6 交接（2026-10-31）

> **签收级别：仿真软件核心功能已验证，Stage 5 5.1–5.4 范围有保留结项；不是所有 GUI 回归全绿，也不是无条件 release。原 PDF 要求的固件端 APF 运行时编排和系统性优化效果尚缺严格验收，详见 [24 项原大纲逐项审计](../project_plan_progress_audit_2026-10-31.md)。**
>
> 该项目交付对象为 ARM Cortex-M4 QEMU / FreeRTOS / 模拟 UART / PyBullet UR5。项目无实体机器人，禁止把本记录写成真机电机、安全联锁或工业认证结论。旧评审 `15_stage5_exit_review_2026-10-30.md` 保留原貌作为历史证据，本页是更新后的阶段状态。

## 1. 阶段范围和签收矩阵

| 原任务书 | 已实测内容 | 结论 / 限制 |
|---|---|---|
| 5.1 全模块集成 | 生产 C 运动学、轨迹及 PID 运行在真实 QEMU ARM/FreeRTOS；UART 双向收发，PyBullet 作为唯一权威虚拟物理体；每段独立核对 ACK 与反馈稳定到位 | **核心通过**；模拟执行器不等于电机驱动 |
| 5.2 静态长方体避障 | PyBullet AABB → 生产 C APF / C IK 选解 / FK → 关节与全连杆离散净距预检 → UART 执行；压缩路径 4/4、未压缩 28/28 均有独立证据 | **指定静态场景通过**；APF/IK 由宿主同源 C 库编排，固件并未自主接收障碍输入并运行完整 APF；不保证连续扫掠、动态障碍、所有奇异位姿或局部极小 |
| 5.3 CPU/内存/实时性 | QEMU/GDB 定时窗口与任务 CPU、周期 P99/最大、栈水位；同 ARM 配置 `.map` 前后比对 | **短窗量测与审计完成**；非满载 WCET 证明，FLASH +48 B、RAM 0 B，不能称减小内存 |
| 5.4 全链路联调/排障 | C 字节流分片/缺字节/坏校验恢复，QEMU UART 陈旧反馈/时序问题回归；Windows 原生 GUI 操作、拒绝/取消、按实际物理回位完成判断 | **核心故障场景通过**；保留遗留原生 GUI legacy 三段收敛与偶发 GUI 无响应问题 |

以上具体 CPU/轨迹/ARM map 数字、历史 GitHub CI 依据见 [15_stage5_exit_review_2026-10-30.md](15_stage5_exit_review_2026-10-30.md)。不重复硬编码 C/物理模型中的阈值和参数。

## 2. 最新原生 GUI 验收证据

- **Windows 操作员实测**：`stage5-j6-manual-evidence.zip` 的独立验收 PASS；原始 C APF 4/4，J6 +0.5° 一次，C/UART 实际回位 1/1，最终 HOLD 最大角误差 **0.02°**，固件故障计数 0。随后 `build/stage5-caption-j6-verified` 的实际鼠标点动、Windows 标题栏 BUSY/RETURN COMPLETE、FINISH 正常结束也通过。细节见 [16_safe_jog_click_review_2026-10-31.md](16_safe_jog_click_review_2026-10-31.md)。
- **Linux 原生 `p.GUI` + Xvfb + QEMU 自动集成**：[Actions #41206909042](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41206909042)，`native-gui-scripted-j6-xvfb` PASS。工件 `stage5-gui-scripted-j6-xvfb`：`result.json` 中初始 4/4、`safe_gui_jog_applied=1`、`safe_gui_jog_completed=1`、`hold_recoveries=1`、最终误差 **0.02°**、时钟对齐、运行时全连杆无碰撞，零坏校验/ACK/固件故障。脚本输入复用正式碰撞/C 预检/物理回位，**并未**代替真实鼠标事件验证。
- **其他同版本门禁**：[Actions #41206909042](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/41206909042)，Windows/Linux GUI 单测、Windows C APF/IK DLL 构建及导出、Windows C 避障回位、Windows/Linux QEMU UART live observer、C 固件回归均 PASS。
- **旧原生 GUI 场景仍失败**：同次 Actions `native-gui-no-drag-xvfb` FAIL，`six_axis` 仅请求第 1 段，60s 后 `0/3`，没有最终验收。该用例**不可**被 J6 新用例的成功自动覆盖。

## 3. 有证据的遗留问题与性质

**S5-010 / [Issue #34](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/34)：legacy 原生 GUI `six_axis` 第一段不收敛，保持 OPEN。**

- CI `stage5-gui-nodrag-xvfb` 工件：`requests_accepted=1`、`segments_settled=0/3`、`target_frames=2115`、`feedback_acks=2113`，三个固件故障计数为零。末段关节读数约为起点 +8.24°，实际请求为 +8.00°，残余约 0.24°，超过现有权威脚本 `SETTLE_TOLERANCE_DEG=0.15`。**原因尚未证实**。不能放宽验收、伪造完成或删除失败测试。
- 过去 Windows 有偶发操作无响应或图形断开；最新受控 Windows 鼠标点动两次验证均成功，但没有复现确定的窗口死锁根因。现在分开记录 `native_gui_health`、点击 receipt、按钮状态、UART/子进程异常，保证将来能判定是未收点击、规划阻塞、窗口消息循环异常还是人为退出。**成功不等于证明绝无偶发问题。**
- 仅在 Stage5 的**已证明 C APF 静态闭环和受控 SAFE JOG 交付范围**认可有保留签收；原生 GUI legacy `six_axis` 的全面兼容不签收。Issue #34 必须留到后续修复，不以 CI 红灯冒充全绿。

## 4. 工程配置与冻结约束

1. Stage4 已验收快照、旧备份分支与 `main` 不改动。Stage5 修复位于 `fix/stage5-safe-jog-small-step-hold-20261031`，PR [#33](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/pull/33) 仍 **Draft/未合并**，基础分支为 `feat/stage5-gui-preserved-integration-20261030`。不将此有保留结项误解为“已进入 main”。
2. UR5 几何与限位以生产 model/URDF 为准；FreeRTOS 周期、轨迹、PID、UART 格式以既有配置与 wire 模块为准；Stage5 APF 净距、碰撞预算和障碍输入按场景 JSON 与生产 C 参数接口，详细唯一来源见 [01_parameter_register.md](01_parameter_register.md) 与 [Stage6 接口契约](../part6/00_stage6_interface_and_acceptance_contract.md)。
3. `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF` 仍保持；未开启任何实体驱动。任何 Stage6 重构先审查参数登记簿，禁止硬编码同义配置、第二套 APF/IK 或删改旧验收阈值。
4. 不满足 Issue #34 验收时，不把 PR #33 标为 Ready 或合并；不通过移除现有 CI 的方式获得表面全绿。

## 5. Stage6 可开始的工作与边界

以**已验收 Stage5 C APF / IK / UART / PyBullet 虚拟闭环接口**作为 Stage6 上层输入，不依赖原生 GUI 的人手点动。Stage6 的新工作是应用层编排：

`原点 → 抓取点 → 避障中间点 → 放置点 → 返回原点`。

每个步骤仍必须依序完成真实 UART ACK、物理稳定到位和全连杆碰撞门禁；虚拟抓取/放置必须有独立物体状态，不能用“末端到了”冒充抓取。场景参数外置，故障场景 fail closed；其余多场景成功率、异常矩阵、连续长时间运行、视频及完整交付按 [Stage6 接口与验收](../part6/00_stage6_interface_and_acceptance_contract.md) 独立计量。

**阶段签收结论：Stage5 5.1–5.4 的静态虚拟机器人闭环主体完成，有保留技术结项；固件运行时自主 APF、原大纲所述完整性能优化成效、原生 GUI 全模式、偶发断连根因和 PR 合并均不作无保留签收。** Issue #34 需在既有阈值下达到 3/3；如要求严格符合原 PDF，还须完成固件端 APF 编排或经用户明确批准的正式需求变更，以及可追溯的性能优化 A/B 验证。
