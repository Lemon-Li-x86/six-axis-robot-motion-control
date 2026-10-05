# 第2阶段：驱动接口规范与模块说明文档

## 1. 文档目的

本文档用于说明六轴工业机器人嵌入式运动控制项目第2阶段完成的驱动与通信模块，包括：

- UART Driver；
- Timer Driver；
- GPIO Driver；
- Motor Driver；
- Ring Buffer；
- Communication Protocol。

主要明确各模块功能、核心接口、数据格式、错误返回值及平台实现方式。

---

## 2. 模块总览

| 模块 | 文件 | 主要功能 |
|---|---|---|
| UART Driver | `drivers/uart_driver.c/h` | UART 中断收发与数据缓存 |
| Timer Driver | `drivers/timer_driver.c/h` | 自由运行计时与周期中断 |
| GPIO Driver | `drivers/gpio_driver.c/h` | GPIO 输出控制 |
| Motor Driver | `drivers/motor_driver.c/h` | 六轴关节电机统一抽象 |
| Ring Buffer | `communications/ring_buffer.c/h` | UART RX / TX 字节缓存 |
| Protocol | `communications/protocol.c/h` | 数据帧构造、解析与校验 |

公共类型和错误码定义于：

```text
common/robot_types.h
common/error_code.h
```

主要调用关系：

```text
Application
    ↓
Protocol / Motor Driver
    ↓
UART Driver
    ↓
Ring Buffer / Hardware
```

---

## 3. 公共数据类型与错误码

机器人关节数量：

```c
#define ROBOT_JOINT_COUNT 6U
```

关节角类型：

```c
typedef int16_t robot_joint_angle_t;
```

单位：

```text
0.01°
```

Canonical Angle 范围：

```text
[-180°, 180°)
```

六轴关节角使用：

```c
robot_joint_angles_t
```

连续关节位置使用：

```c
robot_joint_positions_t
```

底层为 `int32_t`，单位同样为 `0.01°`，用于保存不经过 ±180° 回绕的连续角度。

关节速度使用：

```c
robot_joint_velocities_t
```

单位：

```text
degree / second
```

各模块统一返回：

```c
robot_status_t
```

主要错误码：

| 返回值 | 含义 |
|---|---|
| `ROBOT_STATUS_OK` | 成功 |
| `ROBOT_STATUS_ERROR_NULL_POINTER` | 空指针 |
| `ROBOT_STATUS_ERROR_INVALID_ARGUMENT` | 参数非法 |
| `ROBOT_STATUS_ERROR_INVALID_COMMAND` | Command 非法 |
| `ROBOT_STATUS_ERROR_INVALID_LENGTH` | 数据长度非法 |
| `ROBOT_STATUS_ERROR_BUFFER_FULL` | 缓冲区已满 |
| `ROBOT_STATUS_ERROR_NOT_READY` | 模块尚未准备完成 |
| `ROBOT_STATUS_ERROR_OUT_OF_RANGE` | 参数超出范围 |

统一约定：

```text
0      = Success
负数   = Error
```

---

## 4. UART Driver

### 4.1 模块功能

UART Driver 实现：

- UART0 初始化；
- RX Interrupt；
- TX Interrupt；
- RX / TX Ring Buffer；
- RX Drop 统计；
- RX Event Callback；
- TX Busy 状态管理。

当前 QEMU 平台使用：

```text
CMSDK APB UART0
Base Address = 0x40004000
```

中断：

```text
IRQ0 = UART0 RX
IRQ1 = UART0 TX
```

### 4.2 核心接口

初始化：

```c
robot_status_t uart_driver_init(void);
```

开启 RX Interrupt：

```c
robot_status_t uart_driver_enable_rx_interrupt(void);
```

注册接收事件：

```c
void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
);
```

发送数据：

```c
robot_status_t uart_driver_write(
    const uint8_t *data,
    uint32_t length
);
```

数据首先写入 TX Ring Buffer，实际发送由 TX Interrupt 完成。

读取一个接收 Byte：

```c
uint8_t uart_driver_read_byte(
    uint8_t *data
);
```

返回：

```text
1 = 读取成功
0 = 当前无数据或参数无效
```

查询 RX Drop：

```c
uint32_t uart_driver_get_rx_drop_count(void);
```

查询 TX 状态：

```c
uint8_t uart_driver_is_tx_busy(void);
```

UART 数据路径：

```text
RX:
UART IRQ
→ RX Ring Buffer
→ ProtocolRX Task

TX:
Application
→ TX Ring Buffer
→ UART TX IRQ
```

---

## 5. Timer Driver

### 5.1 模块功能

Timer Driver 使用 CMSDK APB Timer。

当前配置：

```text
Timer0 = Free-running Performance Counter
Timer1 = Periodic Interrupt Timer
```

Timer Clock：

```text
25 MHz
```

即：

```text
1 Tick = 40 ns
```

### 5.2 核心接口

初始化：

```c
void timer_driver_init(void);
```

读取 Timer0：

```c
uint32_t timer_driver_get_counter(void);
```

计算经过 Tick：

```c
uint32_t timer_driver_elapsed_ticks(
    uint32_t start_counter,
    uint32_t end_counter
);
```

获取 Timer Frequency：

```c
uint32_t timer_driver_get_frequency_hz(void);
```

启动 Timer1 周期中断：

```c
robot_status_t timer_driver_start_periodic(
    uint32_t frequency_hz,
    timer_driver_periodic_callback_t callback
);
```

停止：

```c
void timer_driver_stop_periodic(void);
```

状态查询：

```c
uint32_t timer_driver_get_periodic_frequency_hz(void);

uint32_t timer_driver_get_periodic_irq_count(void);

uint8_t timer_driver_is_periodic_running(void);
```

Timer Callback 在 ISR Context 中执行，应保持短小并避免阻塞操作。

---

## 6. GPIO Driver

### 6.1 模块功能

GPIO Driver 提供统一数字输出控制接口。

当前支持：

```text
32 GPIO Pins
```

电平定义：

```c
GPIO_DRIVER_LEVEL_LOW
GPIO_DRIVER_LEVEL_HIGH
```

### 6.2 核心接口

初始化：

```c
robot_status_t gpio_driver_init(void);
```

配置输出：

```c
robot_status_t gpio_driver_configure_output(
    uint8_t pin
);
```

设置电平：

```c
robot_status_t gpio_driver_write(
    uint8_t pin,
    gpio_driver_level_t level
);
```

翻转：

```c
robot_status_t gpio_driver_toggle(
    uint8_t pin
);
```

读取当前输出状态：

```c
robot_status_t gpio_driver_get_output_level(
    uint8_t pin,
    gpio_driver_level_t *level
);
```

查询 Backend：

```c
uint8_t gpio_driver_is_mmio_backend(void);
```

当前 QEMU `mps2-an386` 未提供项目所需 GPIO 实际行为，因此默认使用：

```text
Shadow Backend
```

该 Backend 在软件中保存 Output Enable 和 Output Level。

---

## 7. Motor Driver

### 7.1 模块功能

Motor Driver 为六轴机器人提供统一关节驱动接口。

当前实现为 Simulation Backend，位置反馈主要来自 Python / PyBullet。

提供：

- Target Position；
- Position Feedback；
- Continuous Position；
- Velocity；
- Angle Wrap Correction。

### 7.2 核心接口

初始化：

```c
robot_status_t motor_driver_init(void);
```

设置目标位置：

```c
robot_status_t motor_driver_set_target_positions(
    const robot_joint_angles_t *targets
);
```

获取目标位置：

```c
robot_status_t motor_driver_get_target_positions(
    robot_joint_angles_t *targets
);
```

更新位置反馈：

```c
robot_status_t motor_driver_update_feedback(
    const robot_joint_angles_t *feedback,
    robot_real_t delta_time_s
);
```

获取 Canonical Position：

```c
robot_status_t motor_driver_get_positions(
    robot_joint_angles_t *positions
);
```

获取 Continuous Position：

```c
robot_status_t motor_driver_get_unwrapped_positions(
    robot_joint_positions_t *positions
);
```

获取速度：

```c
robot_status_t motor_driver_get_velocities(
    robot_joint_velocities_t *velocities
);
```

查询反馈状态：

```c
uint8_t motor_driver_has_feedback(void);
```

反馈处理流程：

```text
Feedback
   ↓
Canonical Normalize
   ↓
Angle Wrap Correction
   ↓
Continuous Position
   ↓
Velocity
```

例如：

```text
+179° → -179°
```

实际被识别为：

```text
+2°
```

连续位置为：

```text
179° → 181°
```

---

## 8. Ring Buffer

### 8.1 模块功能

Ring Buffer 为 UART RX / TX 提供字节缓存。

采用：

```text
Single Producer / Single Consumer
```

结构。

物理容量：

```text
128 Byte
```

实际有效容量：

```text
127 Byte
```

### 8.2 核心接口

初始化：

```c
void ring_buffer_init(
    ring_buffer_t *ring_buffer
);
```

写入：

```c
uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
);
```

读取：

```c
uint8_t ring_buffer_read(
    ring_buffer_t *ring_buffer,
    uint8_t *data
);
```

查询状态：

```c
uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
);

uint8_t ring_buffer_is_full(
    const ring_buffer_t *ring_buffer
);
```

UART RX：

```text
Producer = UART RX ISR
Consumer = ProtocolRX Task
```

UART TX：

```text
Producer = UART Driver Write
Consumer = UART TX ISR
```

---

## 9. Communication Protocol

### 9.1 模块功能

Protocol 模块实现：

- 数据帧构造；
- Header 同步；
- Command 解析；
- Payload Length 检查；
- Little Endian 编解码；
- Checksum；
- 字节流状态机；
- 异常输入后的重新同步。

### 9.2 Frame 格式

协议格式：

```text
AA 55 | Command | Length | Payload | Checksum
```

| 字段 | 长度 |
|---|---:|
| Header | 2 Byte |
| Command | 1 Byte |
| Length | 1 Byte |
| Payload | 0 ~ 64 Byte |
| Checksum | 1 Byte |

多字节数据采用：

```text
Little Endian
```

Checksum 计算：

```text
Command + Length + Payload
```

取低 8 bit。

### 9.3 Parser Interface

初始化：

```c
void protocol_parser_init(
    protocol_parser_t *parser
);
```

输入 Byte：

```c
uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
);
```

返回：

```text
1 = 已解析完整合法 Frame
0 = 当前未形成完整合法 Frame
```

Parser 状态：

```text
WAIT_HEADER_0
→ WAIT_HEADER_1
→ READ_COMMAND
→ READ_LENGTH
→ READ_PAYLOAD
→ READ_CHECKSUM
```

### 9.4 Command

| Command | 名称 | 功能 |
|---:|---|---|
| `0x01` | `SET_JOINT_TARGETS` | 设置六轴目标位置 |
| `0x02` | `SET_PARAMETER` | 设置运行参数 |
| `0x81` | `JOINT_STATE` | 上传关节状态 |
| `0x82` | `JOINT_STATE_ACK` | 状态接收确认 |
| `0x83` | `GET_DIAGNOSTICS` | 查询诊断信息 |
| `0x84` | `DIAGNOSTICS_RESPONSE` | 返回诊断信息 |
| `0x85` | `PARAMETER_ACK` | 返回参数配置结果 |

六轴关节 Payload：

```text
6 × int16_t = 12 Byte
```

---

## 10. 模块调用关系

完整通信接收链路：

```text
UART Hardware
    ↓
UART RX IRQ
    ↓
RX Ring Buffer
    ↓
ProtocolRX Task
    ↓
Protocol Parser
    ↓
Motor Driver / Runtime Configuration
```

发送链路：

```text
Application
    ↓
Protocol Frame
    ↓
UART TX Manager
    ↓
UART Driver
    ↓
TX Ring Buffer
    ↓
UART TX IRQ
```

机器人状态链路：

```text
Python / PyBullet
    ↓
JOINT_STATE
    ↓
Protocol
    ↓
Motor Driver
    ↓
Position / Velocity
    ↓
Algorithm / Control
```

---

## 11. 平台实现说明

当前项目运行于：

```text
QEMU MPS2-AN386
ARM Cortex-M4
```

UART 与 Timer 使用 QEMU 提供的：

```text
CMSDK APB UART
CMSDK APB Timer
```

完成寄存器级驱动。

GPIO 当前采用：

```text
Shadow Backend
```

Motor Driver 当前采用：

```text
Simulation Backend
```

整个驱动结构采用：

```text
Upper Layer
    ↓
Standard Driver API
    ↓
Platform Backend
```

因此后续迁移至实际 Cortex-M4 / STM32 平台时，可替换底层 Backend，同时保持上层调用接口稳定。

---

## 12. 结论

第2阶段已经完成：

```text
UART Driver
Timer Driver
GPIO Driver
Motor Driver
Ring Buffer
Communication Protocol
```

等模块的标准化接口设计。

UART 支持 RX / TX Interrupt 和双向 Ring Buffer；Timer 支持高分辨率计时与周期中断；GPIO 提供统一输出控制接口；Motor Driver 提供六轴目标位置、位置反馈、连续位置和速度接口；Protocol 提供完整的数据帧构造与字节流解析功能。

各模块通过公共数据类型和统一错误码进行交互，并通过标准 Driver API 隔离平台相关实现。

这些接口为后续运动学、轨迹规划及闭环控制模块提供基础软件接口。