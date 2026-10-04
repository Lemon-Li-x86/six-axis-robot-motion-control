# 通信与 FreeRTOS 验证基线

## 1. 测试目的

本文档用于记录当前六轴工业机器人嵌入式运动控制固件在通信、协议健壮性、FreeRTOS 实时调度和长时间稳定性方面的验证结果。

当前固件运行环境为：

- ARM Cortex-M4
- QEMU MPS2-AN386
- FreeRTOS
- CMSDK APB UART
- QEMU UART TCP 仿真接口

本文中的所有性能数据均属于：

**QEMU + TCP + FreeRTOS 仿真环境基线**

因此，本文中的通信时延、协议解析时间和任务唤醒时间不能直接作为真实 Cortex-M4 MCU 上的物理 UART 性能或最坏执行时间。

---

## 2. 当前通信架构

当前 UART 接收数据路径为：

UART RX Interrupt  
→ UART Driver Ring Buffer  
→ FreeRTOS Task Notification  
→ ProtocolRX Task  
→ Protocol 字节流状态机  
→ Application / Motor Driver

当前 UART 发送路径为：

ProtocolTX Task / ProtocolRX Task  
→ UART TX Manager  
→ FreeRTOS Mutex  
→ UART Driver  
→ UART0

UART TX Manager 使用 Mutex 对完整协议帧的发送过程进行串行化，防止多个 FreeRTOS Task 同时发送时发生协议帧字节交叉。

当前通信协议支持：

- 固定双字节帧头；
- 可变长度 Payload；
- Checksum 校验；
- 字节流状态机解析；
- 错误帧丢弃；
- 数据流重新同步；
- 未知 Command 拒绝；
- Diagnostics 查询；
- 六轴关节目标命令；
- 六轴关节状态反馈；
- ACK 响应。

---

## 3. 协议异常与恢复测试

### 3.1 测试目的

验证协议状态机在异常输入下能否：

- 正确拒绝非法数据；
- 不产生错误业务行为；
- 从错误数据中重新同步；
- 在后续合法帧到达后继续正常工作。

### 3.2 测试内容

当前异常协议测试覆盖：

1. 正常合法协议帧；
2. 帧头前存在随机垃圾数据；
3. Checksum 错误；
4. Payload 不完整或长度异常；
5. 异常数据后的协议恢复；
6. 未知 Command。

### 3.3 测试结果

**6 / 6 PASS**

所有异常输入均能够被正确处理。

协议 Parser 在发生错误输入后能够重新同步，并继续正确解析后续合法协议帧。

---

## 4. UART 通信 RTT 性能基线

### 4.1 测试条件

- 计划样本数：200
- 成功样本数：200
- Timeout：0

当前测试包含完整的模拟通信路径：

Python Test  
→ TCP  
→ QEMU UART  
→ UART RX ISR  
→ FreeRTOS Task Notification  
→ ProtocolRX Task  
→ Protocol Parser  
→ ACK  
→ UART TX  
→ TCP  
→ Python Test

### 4.2 测试结果

| 指标 | 结果 |
|---|---:|
| Minimum | 0.883 ms |
| Mean | 1.911 ms |
| Median | 1.877 ms |
| P95 | 2.479 ms |
| P99 | 4.692 ms |
| Maximum | 9.910 ms |

### 4.3 结果说明

200 次通信全部成功，没有发生 Timeout。

平均 RTT 约为：

**1.911 ms**

P95 为：

**2.479 ms**

P99 为：

**4.692 ms**

测试中观察到少量长尾延迟，其中最大值为：

**9.910 ms**

该最大值受到 QEMU 仿真调度、Windows 宿主机调度以及 TCP 通信等因素影响，因此不能直接视为真实 Cortex-M4 的最坏通信延迟。

当前 RTT 数据用于建立仿真环境下的通信性能基线。

---

## 5. 单帧协议 Parser 执行时间

### 5.1 测试方法

使用 QEMU MPS2-AN386 的 CMSDK APB Timer0 作为内部高分辨率计时器。

Timer 输入时钟：

**25 MHz**

因此：

**1 Timer Tick = 40 ns**

测试对象为一个完整的 17 Byte 六轴关节协议帧。

连续执行：

**1000 次完整协议帧解析**

并记录 Minimum、Average 和 Maximum。

### 5.2 测试结果

| 指标 | Timer Tick | 时间 |
|---|---:|---:|
| Samples | 1000 | - |
| Minimum | 27 ticks | 1.080 us |
| Average | 48 ticks | 1.920 us |
| Maximum | 7858 ticks | 314.320 us |

### 5.3 结果说明

当前 QEMU 仿真环境下，一个完整 17 Byte 协议帧的典型 Parser 执行时间约为：

**1.92 us**

最低测量结果为：

**1.08 us**

测试中观察到一次明显长尾值：

**314.32 us**

由于平均值仍保持在约 2 us，因此该最大值主要反映 QEMU 和宿主机调度抖动，而不是协议解析算法的正常执行时间。

该测试证明当前字节流状态机的正常协议解析过程在仿真环境下具有微秒级执行开销。

---

## 6. UART ISR 到 ProtocolRX Task 唤醒延迟

### 6.1 测试定义

本测试测量：

UART RX ISR  
→ `vTaskNotifyGiveFromISR()`  
→ FreeRTOS Scheduler  
→ ProtocolRX Task 恢复运行

之间的时间。

因此，本指标准确描述为：

**UART RX ISR → ProtocolRX Task Wake-up Latency**

该指标并不是单独的 FreeRTOS Context Switch 指令执行时间。

它包含：

- UART 中断处理；
- FreeRTOS FromISR Notification；
- Scheduler 调度；
- Task 上下文切换；
- QEMU 仿真调度；
- 宿主机操作系统调度抖动。

### 6.2 测试结果

| 指标 | Timer Tick | 时间 |
|---|---:|---:|
| Samples | 2085 | - |
| Minimum | 70 ticks | 2.800 us |
| Average | 1372 ticks | 54.880 us |
| Maximum | 38077 ticks | 1523.080 us |

### 6.3 结果说明

最低 ISR → Task 唤醒延迟为：

**2.8 us**

平均延迟为：

**54.88 us**

说明在正常调度条件下，当前事件驱动 UART 接收架构能够实现微秒级至几十微秒级的任务唤醒。

测试中最大值为：

**1.523 ms**

该长尾结果同样受到 QEMU 和宿主机调度影响，因此不能直接作为真实 Cortex-M4 上的最坏 Task Wake-up Latency。

---

## 7. UART Burst 压力测试

### 7.1 测试方法

Host 将大量完整 `JOINT_STATE` 协议帧拼接为连续数据，并通过 TCP 尽可能快速发送到 QEMU UART。

固件需要对每个合法 `JOINT_STATE`：

1. 完成协议解析；
2. 更新 Motor Driver 状态；
3. 返回对应 `JOINT_STATE_ACK`。

测试同时读取 UART Driver 的 RX Drop Counter，以检查 Ring Buffer 是否发生溢出。

### 7.2 测试结果

| Burst Frames | Burst Bytes | ACK | Lost | ACK Success | RX Drop Delta |
|---:|---:|---:|---:|---:|---:|
| 10 | 170 | 10 | 0 | 100.00% | 0 B |
| 100 | 1700 | 100 | 0 | 100.00% | 0 B |
| 500 | 8500 | 500 | 0 | 100.00% | 0 B |
| 1000 | 17000 | 1000 | 0 | 100.00% | 0 B |
| 5000 | 85000 | 5000 | 0 | 100.00% | 0 B |

最大 Burst 测试为：

- 5000 个完整协议帧；
- 85000 Byte 连续数据；
- 5000 / 5000 ACK；
- 0 ACK Lost；
- 0 RX Drop。

### 7.3 结果说明

在当前 QEMU + TCP + FreeRTOS 仿真环境下，没有观察到：

- Application 层协议帧丢失；
- UART Driver Ring Buffer 溢出；
- 协议解析失步；
- 多 Task UART TX 帧交叉。

当前 UART 接收和协议处理架构能够通过已有 Burst 压力测试。

---

## 8. 长时间 Soak 稳定性测试

### 8.1 测试目的

验证系统在持续通信条件下是否会出现：

- ACK 丢失；
- 通信 Timeout；
- UART RX Ring Buffer 溢出；
- Protocol Parser 长时间运行失步；
- 通信链路随运行时间增长而逐渐异常。

### 8.2 测试条件

- 目标运行时间：600 s
- 目标发送频率：100 Hz
- ACK Timeout：50 ms
- 连续发送合法 `JOINT_STATE`
- 测试前后读取 UART RX Drop Counter

### 8.3 测试结果

| 指标 | 结果 |
|---|---:|
| Actual Duration | 600.002 s |
| Target Rate | 100.000 Hz |
| Actual Rate | 100.001 Hz |
| Sent | 60001 |
| ACK | 60001 |
| Lost | 0 |
| Timeout | 0 |
| ACK Success | 100.000000% |
| RX Drop Before | 0 B |
| RX Drop After | 0 B |
| RX Drop Delta | 0 B |

### 8.4 结果说明

系统连续运行约：

**10 分钟**

以约：

**100 Hz**

的频率持续执行通信。

实际完成：

**60001 次协议事务**

其中：

- ACK 成功：60001
- ACK 丢失：0
- Timeout：0
- RX Drop：0

ACK 成功率：

**100%**

整个测试期间没有观察到 UART RX Ring Buffer 溢出、协议失步或通信稳定性下降。

因此，当前通信架构在本次仿真测试条件下能够保持长时间稳定运行。

---

## 9. 当前验证结论

当前通信与 FreeRTOS 基线已经完成以下验证：

- UART RX 中断接收；
- Ring Buffer 接收缓存；
- FreeRTOS Task Notification 事件驱动；
- 可变长度协议状态机；
- Checksum 校验；
- 异常协议帧处理；
- 错误数据后的重新同步；
- 多 Task UART TX 串行化；
- 正常功能闭环测试；
- 异常输入测试；
- RTT 性能测量；
- 单帧 Parser 执行时间测量；
- UART ISR 到 Task 唤醒延迟测量；
- Burst 压力测试；
- 长时间 Soak 稳定性测试。

在已有测试条件下：

- 未观察到协议帧丢失；
- 未观察到 UART RX Ring Buffer Drop；
- 未观察到通信 Timeout；
- 未观察到长时间运行失稳。

因此，当前通信和 FreeRTOS 基础架构可以作为后续机器人运动控制算法开发的稳定基线。

---

## 10. 第一阶段导师反馈整改情况

### 10.1 通信协议与 UART 驱动扩展性

导师建议：

- 使用通用字节流状态机；
- 支持可变长度协议；
- 使用 UART Interrupt；
- 引入 Ring Buffer；
- 增加错误和丢帧恢复能力；
- 降低 UART Driver 与 Protocol 的耦合。

当前状态：

**已完成。**

当前实现已经采用 UART RX Interrupt、Ring Buffer、通用 Variable-Length Protocol FSM，并验证了错误帧后的协议重新同步能力。

---

### 10.2 软件分层和跨层接口规范

导师建议：

- 固定六轴机器人核心数据结构；
- 明确数据单位和精度；
- 固定 Driver / Algorithm 对外接口；
- 统一错误码；
- 上层只能通过接口访问下层；
- 为后续轨迹规划、PID 和 FreeRTOS 调度预留结构。

当前状态：

**已完成当前阶段整改。**

当前工程已经建立 Common、Driver、Communication、Algorithm、Application 和 Diagnostics 等模块，并统一了机器人核心数据类型、错误码及跨层接口。

后续轨迹规划和 PID 等功能将在已有结构基础上继续实现。

---

### 10.3 异常测试、性能指标和稳定性验证

导师建议增加：

- 错误帧头；
- Checksum 错误；
- 不完整数据；
- 未知 Command；
- 异常后的协议恢复；
- 通信链路延迟；
- Task Wake-up / 调度延迟；
- 单帧协议解析时间；
- 压力测试；
- 长时间稳定性测试。

当前状态：

**已完成。**

已经完成：

- 6 项协议异常测试；
- 200 次 RTT 性能测试；
- 1000 次 Protocol Parser Benchmark；
- UART ISR → ProtocolRX Task Wake-up 测量；
- 最大 5000 帧 Burst Stress Test；
- 10 分钟、100 Hz、60001 帧 Soak Test。

在当前仿真测试中：

- ACK Lost = 0
- Timeout = 0
- RX Drop = 0

---

### 10.4 注释和接口说明规范

导师建议：

- 标明函数用途；
- 标明输入参数；
- 标明输出参数；
- 标明返回值；
- 对关键变量和重要程序节点添加必要说明。

当前状态：

**已完成当前阶段整改。**

当前公共接口、主要 Driver、Protocol、Application 和 Diagnostics 模块已经统一补充文件说明、函数说明、参数说明、返回值说明和关键流程注释。

---

## 11. 第一阶段整改结论

针对第一阶段导师提出的四类意见：

| 导师意见 | 当前状态 |
|---|---|
| 通信协议和 UART 驱动扩展性 | ✅ 已完成 |
| 软件分层和跨层接口规范 | ✅ 已完成 |
| 异常、性能和稳定性验证 | ✅ 已完成 |
| 注释和接口说明规范 | ✅ 已完成 |

因此，本阶段导师反馈的整改工作到此结束。

当前通信、驱动、FreeRTOS 和协议相关代码作为后续开发的冻结基线。

除非后续开发发现明确的阻断性缺陷，否则不再对该基线进行额外结构性修改。

下一阶段开发重点转向：

**UR5 六轴工业机器人运动学与运动控制算法。**