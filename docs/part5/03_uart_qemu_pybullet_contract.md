# Stage5 QEMU UART↔FreeRTOS↔PyBullet 集成接口与验收说明

2026-10-27：这是软件模拟端到端测试设计；**CI真正运行完成前，不标记整条链路为通过**。

## 数据流与权限边界

```text
PyBullet/软件模拟主机
  ├─ CMD_JOINT_STATE (0x81), 6×int16 centideg ───→ UART0 RX
  │   → Production C Parser → ProtocolRX → Motor State(反馈和时间戳)
  ├─ CMD_REQUEST_JOINT_MOVE (0x03)
  │   payload: [seq:u8] + [6×int16 canonical centideg] ─→ ProtocolRX
  │   → 范围校验 → 反馈新鲜度 → unwrap_nearest → motion_pipeline_submit_joint
  │   ← CMD_JOINT_MOVE_ACK (0x86), [seq:u8,status:i8]（仅表示请求入队）
  ├─ ← CMD_JOINT_STATE_ACK (0x82) （回显被接受的真实反馈）
  └─ ← CMD_SET_JOINT_TARGETS (0x01) （任务Control/PID输出位置目标）
          → PyBullet POSITION_CONTROL → 实际测量反馈循环
```

所有帧统一沿用 `AA 55 | command(1) | len(1) | payload | checksum(1)`；校验为 `(command + length + sum(payload)) & 0xff`。C端格式只由 `communications/protocol.h` 定义，编码/校验使用 `protocol.c`；Python属于串口对端，镜像常量由 `tools/check_protocol_mirror.py` 逐项核对，不在仿真端重写任何PID或轨迹算法。将来协议变动宜转为单一wire schema生成。

## 固件安全构建门控

正常 `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=OFF`、`ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=OFF`。要在QEMU UART上观察真实控制目标，测试专用CMake必须**显式**设置：

- `ROBOT_ENABLE_STAGE4_TASKS=ON`
- `ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT=ON`
- `ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION=ON`
- `ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST=OFF`

Stage5模拟入口仅在最后的QEMU集成开关打开时编译，禁止没有实体紧停/联锁证明就用于实际机械臂。这里开启OUTPUT只会让QEMU中固件**虚拟UART目标更新**，不是实际电机驱动。

## 命令语义与早期边界

- `CMD_SET_JOINT_TARGETS` 是**固件→PyBullet**，绝不与新的**PyBullet→固件**运动请求混同。
- 新请求序号用于异步ACK关联。当前仅支持一个序号待确认请求，**尚无重发去重/幂等/完成通知**；一个ACK成功表示成功提交单槽队列，不等于运动到位。
- `robot_joint_angles_t` 是[-180°,180°)的规范化六轴角。应用层从反馈连续位置按现有 `motion_limits_unwrap_nearest()` 转为受模型限位约束的最近连续目标。只在**反馈新鲜**且Stage4任务已启动时入队。
- 命令0x01由已有ProtocolTX按周期发送；仿真端从UART读出后交给PyBullet，反馈由物理仿真步生成再通过0x81传入。控制算法只存在于真实ARM FreeRTOS镜像中。
- 出错请求ACK携带负数 `ROBOT_STATUS_*`，无运动命令影响；部分截断帧无法可靠确定序号，因此由帧解析器丢弃而不发ACK。

## 本轮测试证据门禁

`stage5-qemu-uart-pybullet.yml` 以真实QEMU MPS2-AN386 Cortex-M4 `freertos_demo.elf`启动、将UART0映射为本机回环TCP接口、用PyBullet DIRECT加载UR5，发反馈、等待反馈ACK、发送1次移动请求、等待运动请求ACK，检测目标帧数量及关节真实物理运动。JSON包括ACK、帧计数、关节运动和错误校验数。

**仍未完成的独立验证**：模拟串口带宽饱和、重复序号去重、请求超时、双向流半帧、30min高负载稳定性、复杂Cartesian轨迹与完整连杆避障，以及硬件安全认证（本项目没有实体设备，不将最后一项列为仿真交付阻断）。

原第四阶段冻结备份与main均不更改。

## 本机整改更新（2026-10-27）

本地开发分支已完成启动目标保持及诊断刷新/最终到位判据修复。ARM/QEMU UART + FreeRTOS + PyBullet单次关节运动闭环本机PASS，最终最大误差0.02°，规划/控制65/130且故障0。该结论对应`927b3090`上的未提交修改，尚未推送，不能写成GitHub CI已经通过。原始复现、具体判据和回归记录见 [本机整改报告](04_local_uart_remediation_2026-10-27.md)。
