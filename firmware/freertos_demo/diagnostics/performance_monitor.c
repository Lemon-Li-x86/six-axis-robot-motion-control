/*
 * 文件：performance_monitor.c
 *
 * 用途：
 * 实现固件内部性能基线统计。
 *
 * 当前包括：
 *
 * 1. Protocol Parser 单帧执行时间；
 * 2. UART RX ISR -> ProtocolRX Task 唤醒时间。
 *
 * 测量结果统一使用 QEMU CMSDK APB Timer0 Tick。
 */

#include <stddef.h>
#include <stdint.h>

#include "performance_monitor.h"
#include "timer_driver.h"
#include "protocol.h"


/* =========================================================
 * Parser Benchmark Statistics
 * ========================================================= */

static uint32_t parser_sample_count = 0U;
static uint32_t parser_min_ticks = 0U;
static uint32_t parser_max_ticks = 0U;
static uint64_t parser_total_ticks = 0U;


/* =========================================================
 * Task Wakeup Statistics
 * ========================================================= */

static uint32_t task_wakeup_sample_count = 0U;
static uint32_t task_wakeup_min_ticks = 0U;
static uint32_t task_wakeup_max_ticks = 0U;
static uint64_t task_wakeup_total_ticks = 0U;


/*
 * 由 ProtocolRX Task 在进入阻塞等待前置 1。
 *
 * 为 1 时，下一次 UART RX ISR
 * 会记录 Task Wakeup Latency 的起始时间。
 */
static volatile uint8_t task_wakeup_measurement_armed = 0U;


/*
 * 表示已经记录 ISR 起始时间，
 * 正在等待 ProtocolRX Task 被唤醒。
 */
static volatile uint8_t task_wakeup_measurement_pending = 0U;


/*
 * UART RX ISR 记录的 Timer0 Counter。
 */
static volatile uint32_t task_wakeup_start_counter = 0U;


/* =========================================================
 * Statistics Helper
 * ========================================================= */

static void performance_monitor_update_statistics(
    uint32_t sample_ticks,
    uint32_t *sample_count,
    uint32_t *minimum_ticks,
    uint32_t *maximum_ticks,
    uint64_t *total_ticks
)
{
    if (*sample_count == 0U)
    {
        /*
         * 第一组 Sample 同时作为
         * Minimum 和 Maximum 初始值。
         */
        *minimum_ticks = sample_ticks;
        *maximum_ticks = sample_ticks;
    }
    else
    {
        if (sample_ticks < *minimum_ticks)
        {
            *minimum_ticks = sample_ticks;
        }

        if (sample_ticks > *maximum_ticks)
        {
            *maximum_ticks = sample_ticks;
        }
    }

    (*sample_count)++;
    *total_ticks += (uint64_t)sample_ticks;
}


/* =========================================================
 * Initialization
 * ========================================================= */

void performance_monitor_init(void)
{
    parser_sample_count = 0U;
    parser_min_ticks = 0U;
    parser_max_ticks = 0U;
    parser_total_ticks = 0U;

    task_wakeup_sample_count = 0U;
    task_wakeup_min_ticks = 0U;
    task_wakeup_max_ticks = 0U;
    task_wakeup_total_ticks = 0U;

    task_wakeup_measurement_armed = 0U;
    task_wakeup_measurement_pending = 0U;
    task_wakeup_start_counter = 0U;
}


/* =========================================================
 * Parser Benchmark
 * ========================================================= */

robot_status_t performance_monitor_run_parser_benchmark(
    uint32_t sample_count
)
{
    protocol_parser_t parser;
    protocol_frame_t output_frame;

    /*
     * 固定 Benchmark 输入。
     *
     * 关节角单位为 0.01 degree：
     *
     * 100 -> 1.00°
     * 600 -> 6.00°
     *
     * Benchmark 只测量 Parser 路径，
     * 具体关节值本身不影响测试目的。
     */
    robot_joint_angles_t sample_joints =
    {
        .value =
        {
            100,
            200,
            300,
            400,
            500,
            600
        }
    };

    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];

    uint32_t sample_index;

    robot_status_t status;

    if (sample_count == 0U)
    {
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }

    status = protocol_build_joint_target_frame(
        &sample_joints,
        frame
    );

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    protocol_parser_init(
        &parser
    );

    for (
        sample_index = 0U;
        sample_index < sample_count;
        sample_index++
    )
    {
        uint32_t byte_index;
        uint32_t start_counter;
        uint32_t end_counter;
        uint32_t elapsed_ticks;

        uint8_t frame_complete = 0U;

        /*
         * Timer0 为自由运行性能计数器。
         *
         * 计时范围覆盖一个完整 Frame
         * 的逐 Byte Parser 调用过程。
         */
        start_counter =
            timer_driver_get_counter();

        for (
            byte_index = 0U;
            byte_index < PROTOCOL_JOINT_FRAME_LEN;
            byte_index++
        )
        {
            if (
                protocol_parser_process_byte(
                    &parser,
                    frame[byte_index],
                    &output_frame
                )
            )
            {
                frame_complete = 1U;
            }
        }

        end_counter =
            timer_driver_get_counter();

        /*
         * 只有 Parser 真正完成完整 Frame
         * 才计入性能统计。
         */
        if (frame_complete)
        {
            elapsed_ticks =
                timer_driver_elapsed_ticks(
                    start_counter,
                    end_counter
                );

            performance_monitor_update_statistics(
                elapsed_ticks,
                &parser_sample_count,
                &parser_min_ticks,
                &parser_max_ticks,
                &parser_total_ticks
            );
        }
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * UART ISR -> ProtocolRX Task Wakeup Measurement
 * ========================================================= */

void performance_monitor_arm_task_wakeup(void)
{
    /*
     * 前一组测量仍未完成时，
     * 不覆盖已有起始时间。
     */
    if (!task_wakeup_measurement_pending)
    {
        task_wakeup_measurement_armed = 1U;
    }
}


void performance_monitor_mark_rx_isr(void)
{
    if (
        task_wakeup_measurement_armed
        && !task_wakeup_measurement_pending
    )
    {
        /*
         * 起点定义为：
         *
         * UART RX ISR 已经进入，
         * 准备通知 ProtocolRX Task 的时间点。
         */
        task_wakeup_start_counter =
            timer_driver_get_counter();

        task_wakeup_measurement_pending = 1U;
        task_wakeup_measurement_armed = 0U;
    }
}


void performance_monitor_record_task_wakeup(void)
{
    if (task_wakeup_measurement_pending)
    {
        uint32_t end_counter;
        uint32_t elapsed_ticks;

        /*
         * 终点定义为：
         *
         * ProtocolRX Task 从阻塞状态恢复后
         * 首次执行到本函数的时间点。
         */
        end_counter =
            timer_driver_get_counter();

        elapsed_ticks =
            timer_driver_elapsed_ticks(
                task_wakeup_start_counter,
                end_counter
            );

        task_wakeup_measurement_pending = 0U;

        performance_monitor_update_statistics(
            elapsed_ticks,
            &task_wakeup_sample_count,
            &task_wakeup_min_ticks,
            &task_wakeup_max_ticks,
            &task_wakeup_total_ticks
        );
    }
}


/* =========================================================
 * Metrics Query
 * ========================================================= */

robot_status_t performance_monitor_get_metrics(
    performance_metrics_t *metrics
)
{
    if (metrics == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    metrics->parser_sample_count =
        parser_sample_count;

    metrics->parser_min_ticks =
        parser_min_ticks;

    metrics->parser_max_ticks =
        parser_max_ticks;

    if (parser_sample_count > 0U)
    {
        metrics->parser_average_ticks =
            (uint32_t)(
                parser_total_ticks
                / parser_sample_count
            );
    }
    else
    {
        metrics->parser_average_ticks = 0U;
    }

    metrics->task_wakeup_sample_count =
        task_wakeup_sample_count;

    metrics->task_wakeup_min_ticks =
        task_wakeup_min_ticks;

    metrics->task_wakeup_max_ticks =
        task_wakeup_max_ticks;

    if (task_wakeup_sample_count > 0U)
    {
        metrics->task_wakeup_average_ticks =
            (uint32_t)(
                task_wakeup_total_ticks
                / task_wakeup_sample_count
            );
    }
    else
    {
        metrics->task_wakeup_average_ticks = 0U;
    }

    return ROBOT_STATUS_OK;
}