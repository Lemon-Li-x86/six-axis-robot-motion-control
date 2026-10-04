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
 * 3. UART TX Manager 初始化；
 * 4. Timer Driver 初始化；
 * 5. Performance Monitor 初始化；
 * 6. Motor Driver 初始化；
 * 7. 创建 ProtocolTX Task；
 * 8. 创建 ProtocolRX Task；
 * 9. 使用 Task Notification 实现 UART RX 事件驱动；
 * 10. 将通信数据交给 Motor Driver；
 * 11. 提供通信与内部性能 Diagnostics；
 * 12. 启动 FreeRTOS Scheduler。
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
#include "uart_tx_manager.h"

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
 * 当前路径：
 *
 * UART RX IRQ
 * ->
 * UART Driver Ring Buffer
 * ->
 * Task Notification
 * ->
 * ProtocolRX Task
 *
 * 同时记录 ISR 时间点，
 * 用于测量 UART ISR 到 ProtocolRX Task
 * 真正恢复运行之间的延迟。
 *
 * @note
 * 本函数运行在 ISR Context，
 * 不能阻塞。
 */
static void uart_rx_event_from_isr(void)
{
    BaseType_t
        higher_priority_task_woken =
            pdFALSE;


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
 * UART TX Manager
 * ->
 * UART Driver
 *
 * @param[in] parameters
 * FreeRTOS Task 参数。
 *
 * 当前未使用。
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
                /*
                 * 完整 Frame 通过 UART TX Manager
                 * 串行发送。
                 *
                 * 即使 ProtocolRX Task 此时需要发送 ACK，
                 * 两个 Task 的 Frame 也不会发生字节交叉。
                 */
                (void)uart_tx_manager_send_frame(
                    frame,
                    PROTOCOL_JOINT_FRAME_LEN
                );
            }
        }


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
 * 当前未使用。
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


    TickType_t
        previous_feedback_tick = 0U;


    uint8_t
        feedback_time_valid = 0U;


    (void)parameters;


    protocol_parser_init(
        &parser
    );


    /*
     * Scheduler、Task 和 Callback
     * 均准备完成后，
     * 再打开 UART RX Interrupt。
     */
    uart_driver_enable_rx_interrupt();


    for (;;)
    {
        uint8_t byte;


        /*
         * 下一次 UART RX ISR
         * 可以作为 Wakeup Measurement 起点。
         */
        performance_monitor_arm_task_wakeup();


        /*
         * 没有 UART RX Event 时完全阻塞。
         *
         * 不使用周期 Polling。
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


        performance_monitor_record_task_wakeup();


        /*
         * 一次性消费当前 Ring Buffer
         * 中全部可用字节。
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
                                (void)uart_tx_manager_send_frame(
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
                                case DIAGNOSTICS_METRIC_RX_DROP_COUNT:

                                    diagnostics_value =
                                        uart_driver_get_rx_drop_count();

                                    break;


                                case DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ:

                                    diagnostics_value =
                                        timer_driver_get_frequency_hz();

                                    break;


                                case DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT:

                                    diagnostics_value =
                                        metrics.parser_sample_count;

                                    break;


                                case DIAGNOSTICS_METRIC_PARSER_MIN_TICKS:

                                    diagnostics_value =
                                        metrics.parser_min_ticks;

                                    break;


                                case DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS:

                                    diagnostics_value =
                                        metrics.parser_average_ticks;

                                    break;


                                case DIAGNOSTICS_METRIC_PARSER_MAX_TICKS:

                                    diagnostics_value =
                                        metrics.parser_max_ticks;

                                    break;


                                case DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT:

                                    diagnostics_value =
                                        metrics.task_wakeup_sample_count;

                                    break;


                                case DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS:

                                    diagnostics_value =
                                        metrics.task_wakeup_min_ticks;

                                    break;


                                case DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS:

                                    diagnostics_value =
                                        metrics.task_wakeup_average_ticks;

                                    break;


                                case DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS:

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
                                (void)uart_tx_manager_send_frame(
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
 * 完成 Board、Driver、Diagnostics、
 * UART TX Manager 和 FreeRTOS Task 初始化，
 * 随后启动 Scheduler。
 *
 * @return
 * 正常情况下不会返回。
 */
int main(void)
{
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


    timer_driver_init();


    /* =====================================================
     * UART TX Serialization
     * ===================================================== */

    if (
        uart_tx_manager_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * Performance Baseline
     * ===================================================== */

    performance_monitor_init();


    /*
     * Scheduler 启动前执行 1000 次
     *完整 Protocol Frame Parser Benchmark。
     */
    if (
        performance_monitor_run_parser_benchmark(
            1000U
        )
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


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

    if (
        xTaskCreate(
            task_protocol_tx,
            "ProtocolTX",
            TASK_STACK_DEPTH_PROTOCOL_TX,
            NULL,
            TASK_PRIORITY_PROTOCOL_TX,
            NULL
        )
        != pdPASS
    )
    {
        while (1)
        {
        }
    }


    if (
        xTaskCreate(
            task_protocol_rx,
            "ProtocolRX",
            TASK_STACK_DEPTH_PROTOCOL_RX,
            NULL,
            TASK_PRIORITY_PROTOCOL_RX,
            &protocol_rx_task_handle
        )
        != pdPASS
    )
    {
        while (1)
        {
        }
    }


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