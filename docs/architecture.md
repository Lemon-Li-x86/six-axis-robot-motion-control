
# 六轴机器人运动控制系统架构

## 1. 项目总体结构

系统由三个主要部分组成：

1. Cortex-M4 / FreeRTOS 嵌入式固件
2. QEMU ARM 硬件仿真环境
3. Python / PyBullet UR5 机器人仿真环境

主要通信链路：

Cortex-M4 Firmware
    -> UART
    -> QEMU TCP
    -> Python Bridge
    -> PyBullet UR5

PyBullet 通过相同通信链路返回关节状态。

## 2. 固件模块职责

### common/
公共数据类型、错误码和单位约定。

### model/
UR5 静态模型、机器人参数和关节限位。

### drivers/
UART、GPIO、Timer、Motor 等底层接口。

### communications/
通信协议编解码、数据缓冲和协议相关逻辑。

### algorithms/math/
矩阵和通用数学运算。

### algorithms/kinematics/
正运动学、逆运动学和候选解计算。

### algorithms/motion/
角度展开、关节限位检查以及后续最优逆解选择。

### algorithms/trajectory/
未来负责轨迹插值、速度和加速度规划。

当前尚未实现，不提前建立空目录。

### algorithms/control/
未来负责 PID 等闭环控制算法。

当前尚未实现，不提前建立空目录。

### application/
FreeRTOS 任务、命令处理和系统运行协调。

### diagnostics/
性能监测、诊断和运行统计。

### tests/
固件测试、QEMU 测试和测试辅助代码。

## 3. 模块依赖原则

- Application 可以调用算法、通信和驱动接口。
- Algorithms 不依赖 FreeRTOS 任务。
- Drivers 不包含运动学、规划和控制策略。
- Communications 不决定机器人如何运动。
- Model 不保存运行时关节反馈。
- 测试代码不应成为正式业务逻辑的依赖。

## 4. 当前实现状态

已具备：

- Cortex-M4 / FreeRTOS 基础运行
- UART 双向通信
- PyBullet UR5 仿真桥接
- UR5 正运动学与解析逆运动学
- UR5 机器人参数与关节限位
- Motion Limits 算法及独立测试目标

尚未完成：

- Motion Limits 与实际运动命令链路的完整集成
- 最优逆解选择
- 时间参数化轨迹规划
- PID 闭环控制
- 独立 Control / Trajectory Task

注意：存在配置字段不代表对应 Task 已实现。

## 5. 后续开发原则

新功能按照职责进入对应模块。

不因为文件变长就盲目拆分。
不为未来功能预先创建空目录。
不在 main.c 中实现具体运动算法。
不重复维护机器人模型参数或协议定义。

重大结构变更需先通过原有构建与回归测试。
