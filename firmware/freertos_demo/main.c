/*
 * 文件：main.c
 *
 * 用途：
 * 六轴机器人 Cortex-M4 / FreeRTOS 固件入口。
 *
 * 当前负责：
 *
 * 1. Board 初始化；
 * 2. UART Driver 初始化；
 * 3. Timer Driver 初始化；
 * 4. Performance Monitor 初始化；
 * 5. Motor Driver 初始化；
 * 6. 创建 ProtocolTX Task；
 * 7. 创建 ProtocolRX Task；
 * 8. 使用 Task Notification 实现 UART RX 事件驱动；
 * 9. 将通信数据交给 Motor Driver；
 * 10. 提供通信与内部性能 Diagnostics；
 * 11. 启动 FreeRTOS Scheduler。
 *
 * 核心机器人数据类型由 robot_types.h 统一定义。
 * Task 调度参数由 task_config.h 统一定义。
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "uart_driver.h"
#include "motor_driver.h"
#include "timer_driver.h"

#include "protocol.h"

#include "robot_types.h"
#include "error_code.h"

#include "task_config.h"
#include "performance_monitor.h"


/* =========================================================
 * ProtocolRX Task Handle
 * ========================================================= */

static TaskHandle_t
    protocol_rx_task_handle = NULL;


/* =========================================================
 * UART RX ISR Callback
 * ========================================================= */

/**
 * @brief UART RX 中断事件回调。
 *
 * 当前数据路径：
 *
 * UART RX IRQ
 * ->
 * UART Driver Ring Buffer
 * ->
 * 本 Callback
 * ->
 * Task Notification
 * ->
 * ProtocolRX Task
 *
 * 本函数同时记录 ISR 时间点，
 * 用于测量：
 *
 * UART ISR
 * ->
 * ProtocolRX Task Wakeup
 *
 * 的内部延迟。
 *
 * @note
 * 本函数运行在 ISR 上下文，
 * 不允许阻塞。
 *
 * 如果调用 FreeRTOS API，
 * 必须使用 FromISR 版本。
 */
static void uart_rx_event_from_isr(void)
{
    BaseType_t
        higher_priority_task_woken =
            pdFALSE;


    /*
     * 如果 ProtocolRX Task
     * 已经为下一次 Wakeup Measurement 做好准备，
     * 这里记录 ISR 起始时间。
     */
    performance_monitor_mark_rx_isr();


    if (
        protocol_rx_task_handle
        != NULL
    )
    {
        vTaskNotifyGiveFromISR(
            protocol_rx_task_handle,
            &higher_priority_task_woken
        );


        portYIELD_FROM_ISR(
            higher_priority_task_woken
        );
    }
}


/* =========================================================
 * ProtocolTX Task
 * ========================================================= */

/**
 * @brief 周期发送当前六轴目标关节位置。
 *
 * 数据路径：
 *
 * Motor Driver
 * ->
 * Protocol
 * ->
 * UART
 * ->
 * Python / PyBullet
 *
 * @param[in] parameters
 * FreeRTOS Task 参数。
 *
 * 当前未使用，应传入 NULL。
 */
static void task_protocol_tx(
    void *parameters
)
{
    robot_joint_angles_t
        joint_targets;


    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    (void)parameters;


    for (;;)
    {
        /*
         * 上层不直接维护 Joint Target。
         *
         * 当前目标位置统一从
         * Motor Driver 获取。
         */
        if (
            motor_driver_get_target_positions(
                &joint_targets
            )
            == ROBOT_STATUS_OK
        )
        {
            if (
                protocol_build_joint_target_frame(
                    &joint_targets,
                    frame
                )
                == ROBOT_STATUS_OK
            )
            {
                uart_driver_write(
                    frame,
                    PROTOCOL_JOINT_FRAME_LEN
                );
            }
        }


        /*
         * 当前按照
         * TASK_PERIOD_PROTOCOL_TX_MS
         * 周期发送目标位置。
         */
        vTaskDelay(
            pdMS_TO_TICKS(
                TASK_PERIOD_PROTOCOL_TX_MS
            )
        );
    }
}


/* =========================================================
 * ProtocolRX Task
 * ========================================================= */

/**
 * @brief 处理 UART 接收字节流和协议帧。
 *
 * 数据路径：
 *
 * UART ISR
 * ->
 * Ring Buffer
 * ->
 * Protocol FSM
 * ->
 * Application Dispatch
 *
 * 当前支持：
 *
 * 1. JOINT_STATE；
 * 2. GET_DIAGNOSTICS。
 *
 * @param[in] parameters
 * FreeRTOS Task 参数。
 *
 * 当前未使用，应传入 NULL。
 */
static void task_protocol_rx(
    void *parameters
)
{
    protocol_parser_t parser;

    protocol_frame_t received_frame;


    robot_joint_angles_t
        received_joint_states;


    robot_joint_angles_t
        stored_joint_states;


    uint8_t ack_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    uint8_t diagnostics_frame[
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    ];


    /*
     * 最近一次有效 Joint Feedback
     * 对应的 FreeRTOS Tick。
     */
    TickType_t
        previous_feedback_tick = 0U;


    /*
     * 标记 previous_feedback_tick
     * 是否已经有效。
     *
     * 第一组 Feedback 没有上一时刻，
     * 因此无法计算速度。
     */
    uint8_t
        feedback_time_valid = 0U;


    (void)parameters;


    protocol_parser_init(
        &parser
    );


    /*
     * Scheduler 已经运行，
     * ProtocolRX Task 已创建，
     * UART Callback 已注册。
     *
     * 此时再打开 UART RX Interrupt，
     * 避免 ISR 到来时上层尚未准备完成。
     */
    uart_driver_enable_rx_interrupt();


    for (;;)
    {
        uint8_t byte;


        /*
         * 告诉 Performance Monitor：
         *
         * 下一次真正到达的 UART RX ISR
         * 可以作为 Task Wakeup Measurement
         * 的起始点。
         */
        performance_monitor_arm_task_wakeup();


        /*
         * 没有 UART RX Event 时，
         * 当前 Task 完全阻塞。
         *
         * 不再使用 1 ms Polling。
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


        /*
         * Task 从 Notification 中恢复运行后，
         * 记录结束时间。
         *
         * 如果此次 Wakeup 对应一个已经记录的
         * UART ISR 起始时间，
         * Performance Monitor 会生成一个有效 Sample。
         */
        performance_monitor_record_task_wakeup();


        /*
         * 被 ISR 唤醒以后，
         * 一次性消费 UART Driver Ring Buffer
         * 中当前已经到达的全部字节。
         */
        while (
            uart_driver_read_byte(
                &byte
            )
        )
        {
            if (
                protocol_parser_process_byte(
                    &parser,
                    byte,
                    &received_frame
                )
            )
            {
                /* =========================================
                 * JOINT_STATE
                 * ========================================= */

                if (
                    protocol_parse_joint_state(
                        &received_frame,
                        &received_joint_states
                    )
                    == ROBOT_STATUS_OK
                )
                {
                    TickType_t
                        current_feedback_tick;


                    robot_real_t
                        delta_time_s;


                    current_feedback_tick =
                        xTaskGetTickCount();


                    /*
                     * 第一组 Feedback
                     * 没有上一采样时间，
                     * 因此不能计算速度。
                     */
                    if (
                        feedback_time_valid
                    )
                    {
                        TickType_t
                            elapsed_ticks;


                        elapsed_ticks =
                            current_feedback_tick
                            -
                            previous_feedback_tick;


                        delta_time_s =
                            (robot_real_t)elapsed_ticks
                            /
                            (robot_real_t)configTICK_RATE_HZ;
                    }
                    else
                    {
                        delta_time_s =
                            0.0F;
                    }


                    /*
                     * 将新的 Joint Feedback
                     * 交给 Motor Driver。
                     *
                     * Motor Driver 统一负责：
                     *
                     * 1. Position Feedback；
                     * 2. Velocity Feedback。
                     */
                    if (
                        motor_driver_update_feedback(
                            &received_joint_states,
                            delta_time_s
                        )
                        == ROBOT_STATUS_OK
                    )
                    {
                        previous_feedback_tick =
                            current_feedback_tick;


                        feedback_time_valid =
                            1U;


                        /*
                         * ACK 使用 Motor Driver
                         * 已经保存的标准状态。
                         *
                         * Protocol 层不直接成为
                         * Robot State Storage。
                         */
                        if (
                            motor_driver_get_positions(
                                &stored_joint_states
                            )
                            == ROBOT_STATUS_OK
                        )
                        {
                            if (
                                protocol_build_joint_state_ack_frame(
                                    &stored_joint_states,
                                    ack_frame
                                )
                                == ROBOT_STATUS_OK
                            )
                            {
                                uart_driver_write(
                                    ack_frame,
                                    PROTOCOL_JOINT_FRAME_LEN
                                );
                            }
                        }
                    }
                }

                /* =========================================
                 * DIAGNOSTICS
                 * ========================================= */

                else
                {
                    uint8_t
                        diagnostics_selector;


                    if (
                        protocol_parse_diagnostics_request(
                            &received_frame,
                            &diagnostics_selector
                        )
                        == ROBOT_STATUS_OK
                    )
                    {
                        performance_metrics_t
                            metrics;


                        uint32_t
                            diagnostics_value = 0U;


                        if (
                            performance_monitor_get_metrics(
                                &metrics
                            )
                            == ROBOT_STATUS_OK
                        )
                        {
                            switch (
                                diagnostics_selector
                            )
                            {
                                /* -------------------------
                                 * UART RX Drop Count
                                 * ------------------------- */

                                case
                                    DIAGNOSTICS_METRIC_RX_DROP_COUNT:

                                    diagnostics_value =
                                        uart_driver_get_rx_drop_count();

                                    break;


                                /* -------------------------
                                 * Timer Frequency
                                 * ------------------------- */

                                case
                                    DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ:

                                    diagnostics_value =
                                        timer_driver_get_frequency_hz();

                                    break;


                                /* -------------------------
                                 * Protocol Parser
                                 * ------------------------- */

                                case
                                    DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT:

                                    diagnostics_value =
                                        metrics.parser_sample_count;

                                    break;


                                case
                                    DIAGNOSTICS_METRIC_PARSER_MIN_TICKS:

                                    diagnostics_value =
                                        metrics.parser_min_ticks;

                                    break;


                                case
                                    DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS:

                                    diagnostics_value =
                                        metrics.parser_average_ticks;

                                    break;


                                case
                                    DIAGNOSTICS_METRIC_PARSER_MAX_TICKS:

                                    diagnostics_value =
                                        metrics.parser_max_ticks;

                                    break;


                                /* -------------------------
                                 * ISR -> Task Wakeup
                                 * ------------------------- */

                                case
                                    DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT:

                                    diagnostics_value =
                                        metrics.task_wakeup_sample_count;

                                    break;


                                case
                                    DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS:

                                    diagnostics_value =
                                        metrics.task_wakeup_min_ticks;

                                    break;


                                case
                                    DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS:

                                    diagnostics_value =
                                        metrics.task_wakeup_average_ticks;

                                    break;


                                case
                                    DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS:

                                    diagnostics_value =
                                        metrics.task_wakeup_max_ticks;

                                    break;


                                default:

                                    diagnostics_value =
                                        0U;

                                    break;
                            }


                            if (
                                protocol_build_diagnostics_response_frame(
                                    diagnostics_value,
                                    diagnostics_frame
                                )
                                == ROBOT_STATUS_OK
                            )
                            {
                                uart_driver_write(
                                    diagnostics_frame,
                                    PROTOCOL_DIAGNOSTICS_FRAME_LEN
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}


/* =========================================================
 * Firmware Entry
 * ========================================================= */

/**
 * @brief 固件主入口。
 *
 * 初始化 Board、UART、Timer、Performance Monitor、
 * Motor Driver 和 FreeRTOS Task，
 * 随后启动 FreeRTOS Scheduler。
 *
 * @return
 * 正常情况下不会返回。
 */
int main(void)
{
    /*
     * 当前仿真测试初始目标：
     *
     * Joint 1 = 60.00°
     * Joint 2~6 = 0.00°
     */
    const robot_joint_angles_t
        initial_joint_targets =
        {
            .value =
            {
                6000,
                0,
                0,
                0,
                0,
                0
            }
        };


    /* =====================================================
     * Board
     * ===================================================== */

    board_clock_init();


    board_gpio_init();


    /* =====================================================
     * Driver
     * ===================================================== */

    uart_driver_init();


    /*
     * 初始化自由运行 CMSDK APB Timer0。
     *
     * Performance Monitor
     * 后续使用该 Timer 进行高分辨率计时。
     */
    timer_driver_init();


    /* =====================================================
     * Performance Baseline
     * ===================================================== */

    performance_monitor_init();


    /*
     * 在 Scheduler 启动前，
     * 对完整 Protocol Frame Parser
     * 执行 1000 次内部 Benchmark。
     *
     * 结果保存在 Performance Monitor 中，
     * 后续可通过 Diagnostics 查询。
     */
    (void)performance_monitor_run_parser_benchmark(
        1000U
    );


    /* =====================================================
     * Motor Driver
     * ===================================================== */

    if (
        motor_driver_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    if (
        motor_driver_set_target_positions(
            &initial_joint_targets
        )
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * FreeRTOS Task
     * ===================================================== */

    xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        TASK_STACK_DEPTH_PROTOCOL_TX,
        NULL,
        TASK_PRIORITY_PROTOCOL_TX,
        NULL
    );


    xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        TASK_STACK_DEPTH_PROTOCOL_RX,
        NULL,
        TASK_PRIORITY_PROTOCOL_RX,
        &protocol_rx_task_handle
    );


    /*
     * ProtocolRX Task Handle 已经获得，
     * 此时再注册 UART ISR Callback。
     */
    uart_driver_set_rx_event_callback(
        uart_rx_event_from_isr
    );


    /* =====================================================
     * Scheduler
     * ===================================================== */

    vTaskStartScheduler();


    /*
     * 正常情况下 Scheduler 不会返回。
     */
    while (1)
    {
    }
}