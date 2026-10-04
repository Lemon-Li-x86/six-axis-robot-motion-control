/*
 * 文件：performance_monitor.h
 *
 * 用途：
 * 定义固件内部性能基线测量接口。
 *
 * 当前测量：
 *
 * 1. 单个完整协议帧的 Parser 执行时间；
 * 2. UART RX ISR 到 ProtocolRX Task 被唤醒的时间。
 *
 * 所有时间结果均以 Timer Tick 保存。
 */

#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#include <stdint.h>

#include "error_code.h"


typedef struct
{
    uint32_t parser_sample_count;

    uint32_t parser_min_ticks;

    uint32_t parser_average_ticks;

    uint32_t parser_max_ticks;


    uint32_t task_wakeup_sample_count;

    uint32_t task_wakeup_min_ticks;

    uint32_t task_wakeup_average_ticks;

    uint32_t task_wakeup_max_ticks;

} performance_metrics_t;


/**
 * @brief 初始化性能监测模块。
 */
void performance_monitor_init(void);


/**
 * @brief 执行 Protocol Parser 内部性能基准测试。
 *
 * @param[in] sample_count
 * 需要执行的完整协议帧解析次数。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 测试成功。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * sample_count 为 0。
 *
 * 其他：
 * Protocol Frame 构造失败。
 */
robot_status_t performance_monitor_run_parser_benchmark(
    uint32_t sample_count
);


/**
 * @brief 准备测量下一次 UART ISR -> Task 唤醒时间。
 *
 * 应由 ProtocolRX Task
 * 在进入阻塞等待之前调用。
 */
void performance_monitor_arm_task_wakeup(void);


/**
 * @brief 记录 UART ISR 通知 Task 之前的时间点。
 *
 * 本函数在 UART ISR 上下文中调用。
 */
void performance_monitor_mark_rx_isr(void);


/**
 * @brief 记录 ProtocolRX Task 被唤醒后的时间点。
 *
 * 如果之前存在有效 ISR 起始时间，
 * 则生成一个 Task Wakeup Latency Sample。
 */
void performance_monitor_record_task_wakeup(void);


/**
 * @brief 获取当前性能统计结果。
 *
 * @param[out] metrics
 * 输出当前性能指标。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 获取成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * metrics 为空。
 */
robot_status_t performance_monitor_get_metrics(
    performance_metrics_t *metrics
);


#endif