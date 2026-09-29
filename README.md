# six-axis-robot-motion-control

六轴工业机器人嵌入式运动控制固件开发与仿真验证

## Project Goal

基于 Cortex-M4、FreeRTOS 和 PyBullet，
逐步完成六轴工业机器人的嵌入式运动控制固件与仿真验证。

## Week 1 Goals

- [x] 理解项目总体架构与软件分层
- [x] 搭建 ARM Cortex-M4 / QEMU 交叉编译与仿真环境
- [x] 搭建 FreeRTOS 基础工程并验证任务调度
- [x] 建立 CMake + ARM GNU Toolchain 构建系统
- [x] 建立基础时钟与 GPIO 初始化工程模板
- [x] 搭建 PyBullet / UR5 仿真环境
- [x] 实现 UR5 关节控制、环境物体添加与视角切换
- [x] 设计 UART 二进制通信协议
- [x] 实现 Cortex-M4 与 Python 仿真端双向通信
- [x] 完成 Cortex-M4 → PyBullet UR5 控制与状态反馈闭环验证
- [ ] 完成第一周工作记录

## Week 1 Progress

第一阶段已完成 Cortex-M4 嵌入式开发环境、FreeRTOS 基础工程、
PyBullet / UR5 仿真环境以及 MCU 与仿真端之间的双向通信链路搭建。

当前系统基本链路如下：

```text
Cortex-M4 / FreeRTOS
        │
        │ UART 二进制控制帧
        ▼
       QEMU
        │
        │ TCP Serial Backend
        ▼
      Python
        │
        ▼
   PyBullet / UR5
        │
        │ 关节状态反馈
        ▼
      Python
        │
        ▼
Cortex-M4 / FreeRTOS