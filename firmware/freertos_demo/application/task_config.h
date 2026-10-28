
/*
 * 文件：task_config.h
 *
 * 用途：
 * 定义机器人固件的 FreeRTOS Task 配置接口。
 *
 * 配置来源：
 *
 * config/robot_defaults.json
 *        |
 *        v
 * generate_firmware_defaults.py
 *        |
 *        v
 * firmware_defaults_generated.h
 *        |
 *        v
 * task_config.h
 *
 * 设计原则：
 *
 * 1. 默认参数只在 JSON 中维护；
 * 2. 本文件不重复写死默认数值；
 * 3. 保持现有 TASK_* 宏名称；
 * 4. 避免上层 Task 直接依赖配置生成细节；
 * 5. 新的 Control、Trajectory、Monitor Task
 *    可以继续使用本接口。
 *
 * 注意：
 *
 * ProtocolRX / ProtocolTX 仍照常创建；Stage4的 Control、
 * Trajectory 和 Monitor 仅在 ROBOT_ENABLE_STAGE4_TASKS=ON
 * 时由 motion_pipeline_start() 创建。
 * 所有任务堆栈和周期仍只有同一份JSON默认配置。
 */

#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H


/* =========================================================
 * Generated Configuration
 * ========================================================= */

/*
 * 此头文件由 CMake 在编译前自动生成。
 *
 * 开发者不应手动修改生成结果。
 */
#include "firmware_defaults_generated.h"


/* =========================================================
 * Task Priority
 * ========================================================= */

/*
 * FreeRTOS 任务优先级：
 *
 * 数值越大，任务优先级越高。
 *
 * 当前 configMAX_PRIORITIES = 5，
 * 因此优先级有效范围为 0~4。
 *
 * 参数范围在 Python 生成阶段验证。
 */

/*
 * Control：
 * Stage4 PID / Motion Control Task（按编译门控启用）。
 */
#define TASK_PRIORITY_CONTROL \
    FW_DEFAULT_TASK_PRIORITY_CONTROL


/*
 * ProtocolRX：
 * UART 接收与协议解析。
 */
#define TASK_PRIORITY_PROTOCOL_RX \
    FW_DEFAULT_TASK_PRIORITY_PROTOCOL_RX


/*
 * Trajectory：
 * 轨迹规划任务。
 */
#define TASK_PRIORITY_TRAJECTORY \
    FW_DEFAULT_TASK_PRIORITY_TRAJECTORY


/*
 * ProtocolTX：
 * 状态和目标发送任务。
 */
#define TASK_PRIORITY_PROTOCOL_TX \
    FW_DEFAULT_TASK_PRIORITY_PROTOCOL_TX


/*
 * Monitor：
 * 低优先级诊断和状态监测。
 */
#define TASK_PRIORITY_MONITOR \
    FW_DEFAULT_TASK_PRIORITY_MONITOR


/* =========================================================
 * Task Period
 * ========================================================= */

/*
 * 单位：
 * millisecond。
 *
 * 对周期任务而言，实际调度精度还受到
 * FreeRTOS Tick、优先级和系统负载影响。
 */

/*
 * PID Control 周期。
 */
#define TASK_PERIOD_CONTROL_MS \
    FW_DEFAULT_TASK_PERIOD_CONTROL_MS


/*
 * Trajectory Planning 周期。
 */
#define TASK_PERIOD_TRAJECTORY_MS \
    FW_DEFAULT_TASK_PERIOD_TRAJECTORY_MS


/*
 * Monitor 周期。
 */
#define TASK_PERIOD_MONITOR_MS \
    FW_DEFAULT_TASK_PERIOD_MONITOR_MS


/*
 * ProtocolTX 默认周期。
 *
 * 注意：
 *
 * 这是启动默认值。
 *
 * 固件运行后仍然允许通过
 * Runtime Config 和通信协议修改。
 */
#define TASK_PERIOD_PROTOCOL_TX_MS \
    FW_DEFAULT_TASK_PERIOD_PROTOCOL_TX_MS

/* Maximum idle interval between bytes in a partial protocol frame. */
#define PROTOCOL_FRAME_TIMEOUT_MS \
    FW_DEFAULT_PROTOCOL_FRAME_TIMEOUT_MS


/* =========================================================
 * Task Stack Depth
 * ========================================================= */

/*
 * xTaskCreate() 的 Stack Depth
 * 单位是 StackType_t 元素数量，
 * 不是 Byte。
 *
 * 当前 ARM Cortex-M4 使用 32-bit StackType_t，
 * 因此 256 words 通常对应 1024 bytes。
 *
 * JSON 中保存的是 words。
 *
 * 这些值是JSON声明的静态栈分配。
 * 阶段4 QEMU诊断通过 uxTaskGetStackHighWaterMark()
 * 记录高水位，但短时数据不是最坏情况下的栈上界。
 */

/*
 * Control Task Stack。
 */
#define TASK_STACK_DEPTH_CONTROL \
    FW_DEFAULT_TASK_STACK_CONTROL_WORDS


/*
 * ProtocolRX Task Stack。
 */
#define TASK_STACK_DEPTH_PROTOCOL_RX \
    FW_DEFAULT_TASK_STACK_PROTOCOL_RX_WORDS


/*
 * Trajectory Task Stack。
 */
#define TASK_STACK_DEPTH_TRAJECTORY \
    FW_DEFAULT_TASK_STACK_TRAJECTORY_WORDS


/*
 * ProtocolTX Task Stack。
 */
#define TASK_STACK_DEPTH_PROTOCOL_TX \
    FW_DEFAULT_TASK_STACK_PROTOCOL_TX_WORDS


/*
 * Monitor Task Stack。
 */
#define TASK_STACK_DEPTH_MONITOR \
    FW_DEFAULT_TASK_STACK_MONITOR_WORDS


#endif /* TASK_CONFIG_H */
