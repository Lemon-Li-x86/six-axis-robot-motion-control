/*
 * 文件：task_config.h
 *
 * 用途：
 * 集中定义机器人固件 FreeRTOS Task
 * 的优先级、周期和栈深度规划。
 *
 * 当前只有通信任务已经运行。
 *
 * Control、Trajectory、Monitor Task
 * 在对应模块实现后再创建，
 * 但调度位置和优先级关系现在提前冻结。
 *
 * FreeRTOS 任务优先级规则：
 *
 * 数值越大，
 * Task 优先级越高。
 */

#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H


/* =========================================================
 * Task Priority
 * ========================================================= */

/*
 * PID / Motion Control：
 *
 * 需要固定周期运行，
 * 因此设置为最高业务任务优先级。
 */
#define TASK_PRIORITY_CONTROL 4U


/*
 * UART RX / Protocol：
 *
 * 需要及时消费 Ring Buffer，
 * 防止接收数据堆积。
 */
#define TASK_PRIORITY_PROTOCOL_RX 3U


/*
 * Trajectory Planning：
 *
 * 负责生成未来目标点，
 * 实时性低于闭环控制。
 */
#define TASK_PRIORITY_TRAJECTORY 2U


/*
 * Protocol TX：
 *
 * 状态或目标通信发送。
 */
#define TASK_PRIORITY_PROTOCOL_TX 1U


/*
 * 状态监测与低优先级诊断。
 */
#define TASK_PRIORITY_MONITOR 1U


/* =========================================================
 * Task Period
 * ========================================================= */

/*
 * PID Control：
 *
 * 当前预留 10 ms。
 *
 * 后续可根据仿真与控制性能
 * 调整到更高控制频率。
 */
#define TASK_PERIOD_CONTROL_MS 10U


/*
 * Trajectory Planning：
 *
 * 当前预留 20 ms。
 */
#define TASK_PERIOD_TRAJECTORY_MS 20U


/*
 * Monitor：
 *
 * 当前预留 100 ms。
 */
#define TASK_PERIOD_MONITOR_MS 100U


/*
 * 当前 Protocol TX
 * 保持原有 1000 ms 测试周期。
 */
#define TASK_PERIOD_PROTOCOL_TX_MS 1000U


/* =========================================================
 * Task Stack Depth
 * ========================================================= */

/*
 * xTaskCreate() 的 Stack Depth
 * 单位不是 Byte，
 * 而是 StackType_t 元素数量。
 *
 * 当前只进行初始预留，
 * 后续使用 FreeRTOS Stack High Water Mark
 * 实测后再调整。
 */

#define TASK_STACK_DEPTH_CONTROL 256U

#define TASK_STACK_DEPTH_PROTOCOL_RX 256U

#define TASK_STACK_DEPTH_TRAJECTORY 256U

#define TASK_STACK_DEPTH_PROTOCOL_TX 256U

#define TASK_STACK_DEPTH_MONITOR 256U


#endif