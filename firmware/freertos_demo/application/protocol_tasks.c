/*
 * 文件：protocol_tasks.c
 *
 * 用途：
 * 实现机器人固件 Protocol Application Tasks。
 *
 * 当前负责：
 *
 * 1. ProtocolTX 周期发送；
 * 2. ProtocolRX 事件驱动接收；
 * 3. UART RX ISR -> Task Notification；
 * 4. Protocol Frame Application Dispatch；
 * 5. Joint State Feedback 处理；
 * 6. Joint State ACK；
 * 7. Diagnostics Request / Response。
 *
 *
 * =========================================================
 * 分层约定
 * =========================================================
 *
 * UART Driver：
 *
 * 只负责 UART 硬件和 RX Ring Buffer。
 *
 *
 * Protocol：
 *
 * 只负责：
 *
 * 1. Frame Encode；
 * 2. Frame Decode；
 * 3. Byte Stream Parser。
 *
 *
 * Protocol Tasks：
 *
 * 负责把 Protocol Frame
 * 分发给具体 Application 功能。
 *
 *
 * Motor Driver：
 *
 * 负责保存和处理关节目标、
 * Canonical Feedback、
 * Continuous Position 和 Velocity。
 *
 *
 * main.c：
 *
 * 只负责系统级初始化和启动。
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "protocol_tasks.h"
#include "task_config.h"
#include "uart_tx_manager.h"

#include "uart_driver.h"
#include "motor_driver.h"
#include "timer_driver.h"

#include "protocol.h"

#include "robot_types.h"
#include "error_code.h"

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
 * 数据路径：
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
 * 用于测量：
 *
 * UART ISR
 * ->
 * ProtocolRX Task
 *
 * 的任务唤醒延迟。
 *
 * @note
 * 本函数运行在 ISR Context。
 *
 * 因此：
 *
 * 1. 不得阻塞；
 * 2. 只能使用 FreeRTOS FromISR API。
 */
static void
uart_rx_event_from_isr(void)
{
    BaseType_t
        higher_priority_task_woken =
            pdFALSE;


    performance_monitor_mark_rx_isr();


    if (
        protocol_rx_task_handle
        !=
        NULL
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
 * Joint State Handler
 * ========================================================= */

/**
 * @brief 处理 CMD_JOINT_STATE。
 *
 * 当前流程：
 *
 * Protocol Frame
 * ->
 * Joint State Decode
 * ->
 * Feedback Delta Time
 * ->
 * Motor Driver
 * ->
 * Canonical Stored State
 * ->
 * Joint State ACK
 *
 * @param[in] frame
 * 已完成 Parser 校验的协议帧。
 *
 * @param[in,out] previous_feedback_tick
 * 上一次成功处理 Feedback 时的
 * FreeRTOS Tick。
 *
 * @param[in,out] feedback_time_valid
 * 是否已经存在上一组有效 Feedback 时间。
 */
static void
protocol_tasks_handle_joint_state(
    const protocol_frame_t *frame,
    TickType_t *previous_feedback_tick,
    uint8_t *feedback_time_valid
)
{
    robot_joint_angles_t
        received_joint_states;


    robot_joint_angles_t
        stored_joint_states;


    uint8_t ack_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    TickType_t
        current_feedback_tick;


    robot_real_t
        delta_time_s;


    if (
        frame
        ==
        NULL
        ||
        previous_feedback_tick
        ==
        NULL
        ||
        feedback_time_valid
        ==
        NULL
    )
    {
        return;
    }


    /*
     * Protocol Layer 负责验证：
     *
     * 1. Command；
     * 2. Payload Length；
     * 3. Payload Decode。
     */
    if (
        protocol_parse_joint_state(
            frame,
            &received_joint_states
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    current_feedback_tick =
        xTaskGetTickCount();


    /*
     * 第一组 Feedback 没有上一采样点，
     * 因此 delta_time_s = 0。
     *
     * Motor Driver 会正常更新位置，
     * 并将当前速度设置为 0。
     */
    if (
        *feedback_time_valid
    )
    {
        TickType_t
            elapsed_ticks;


        elapsed_ticks =
            current_feedback_tick
            -
            *previous_feedback_tick;


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
     * Motor Driver 负责：
     *
     * 1. Canonical Angle Normalize；
     * 2. Wrap Correction；
     * 3. Continuous Position；
     * 4. Velocity。
     */
    if (
        motor_driver_update_feedback(
            &received_joint_states,
            delta_time_s
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    /*
     * 只有 Feedback 真正成功进入
     * Motor Driver 后，
     * 才更新 Feedback 时间基准。
     */
    *previous_feedback_tick =
        current_feedback_tick;


    *feedback_time_valid =
        1U;


    /*
     * ACK 不直接回显收到的 Payload。
     *
     * 而是重新读取 Motor Driver
     * 已经保存和规范化后的状态。
     *
     * 这样 ACK 表示：
     *
     * “Application 已经真正接受了该状态。”
     */
    if (
        motor_driver_get_positions(
            &stored_joint_states
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    if (
        protocol_build_joint_state_ack_frame(
            &stored_joint_states,
            ack_frame
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    /*
     * UART TX Manager 负责完整 Frame
     * 的串行化发送。
     *
     * ProtocolTX 和 ProtocolRX
     * 即使同时要求发送，
     * Frame 字节也不会互相交叉。
     */
    (void)uart_tx_manager_send_frame(
        ack_frame,
        PROTOCOL_JOINT_FRAME_LEN
    );
}


/* =========================================================
 * Diagnostics Value Provider
 * ========================================================= */

/**
 * @brief 根据 Diagnostics Selector 获取当前指标。
 *
 * @param[in] selector
 * Diagnostics Metric Selector。
 *
 * @param[out] value
 * 输出 uint32_t Metric Value。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 获取成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * value 为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * selector 不支持。
 *
 * 其他：
 * Performance Monitor 获取失败。
 */
static robot_status_t
protocol_tasks_get_diagnostics_value(
    uint8_t selector,
    uint32_t *value
)
{
    performance_metrics_t
        metrics;


    robot_status_t
        status;


    if (
        value
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * 当前所有 Diagnostics 请求
     * 都沿用已有 Performance Monitor
     * Snapshot。
     *
     * 即使某些指标来自 Driver，
     * 仍保持原有处理路径，
     * 避免本轮结构重构改变行为。
     */
    status =
        performance_monitor_get_metrics(
            &metrics
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    switch (
        selector
    )
    {
        case DIAGNOSTICS_METRIC_RX_DROP_COUNT:

            *value =
                uart_driver_get_rx_drop_count();

            break;


        case DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ:

            *value =
                timer_driver_get_frequency_hz();

            break;


        case DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT:

            *value =
                metrics.parser_sample_count;

            break;


        case DIAGNOSTICS_METRIC_PARSER_MIN_TICKS:

            *value =
                metrics.parser_min_ticks;

            break;


        case DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS:

            *value =
                metrics.parser_average_ticks;

            break;


        case DIAGNOSTICS_METRIC_PARSER_MAX_TICKS:

            *value =
                metrics.parser_max_ticks;

            break;


        case DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT:

            *value =
                metrics.task_wakeup_sample_count;

            break;


        case DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS:

            *value =
                metrics.task_wakeup_min_ticks;

            break;


        case DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS:

            *value =
                metrics.task_wakeup_average_ticks;

            break;


        case DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS:

            *value =
                metrics.task_wakeup_max_ticks;

            break;


        default:

            return
                ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Diagnostics Handler
 * ========================================================= */

/**
 * @brief 处理 CMD_GET_DIAGNOSTICS。
 *
 * 数据路径：
 *
 * Protocol Frame
 * ->
 * Diagnostics Selector
 * ->
 * Diagnostics Value
 * ->
 * Response Frame
 * ->
 * UART TX Manager
 */
static void
protocol_tasks_handle_diagnostics(
    const protocol_frame_t *frame
)
{
    uint8_t
        diagnostics_selector;


    uint32_t
        diagnostics_value;


    uint8_t diagnostics_frame[
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    ];


    if (
        frame
        ==
        NULL
    )
    {
        return;
    }


    if (
        protocol_parse_diagnostics_request(
            frame,
            &diagnostics_selector
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    if (
        protocol_tasks_get_diagnostics_value(
            diagnostics_selector,
            &diagnostics_value
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    if (
        protocol_build_diagnostics_response_frame(
            diagnostics_value,
            diagnostics_frame
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return;
    }


    (void)uart_tx_manager_send_frame(
        diagnostics_frame,
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    );
}


/* =========================================================
 * Application Frame Dispatch
 * ========================================================= */

/**
 * @brief 将完整合法 Protocol Frame
 *        分发给对应 Application Handler。
 *
 * Parser 只负责识别 Frame，
 * 不负责业务行为。
 *
 * 所有 Command -> Application Action
 * 的映射集中在这里。
 */
static void
protocol_tasks_dispatch_frame(
    const protocol_frame_t *frame,
    TickType_t *previous_feedback_tick,
    uint8_t *feedback_time_valid
)
{
    if (
        frame
        ==
        NULL
    )
    {
        return;
    }


    switch (
        frame->command
    )
    {
        case CMD_JOINT_STATE:

            protocol_tasks_handle_joint_state(
                frame,
                previous_feedback_tick,
                feedback_time_valid
            );

            break;


        case CMD_GET_DIAGNOSTICS:

            protocol_tasks_handle_diagnostics(
                frame
            );

            break;


        default:

            /*
             * 当前 Application 不处理
             * 其他输入 Command。
             *
             * 未知或方向错误的 Command
             * 直接忽略。
             */
            break;
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
 */
static void
task_protocol_tx(
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
            ==
            ROBOT_STATUS_OK
        )
        {
            if (
                protocol_build_joint_target_frame(
                    &joint_targets,
                    frame
                )
                ==
                ROBOT_STATUS_OK
            )
            {
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
 * @brief 处理 UART 接收字节流。
 *
 * 数据路径：
 *
 * UART RX IRQ
 * ->
 * UART Driver Ring Buffer
 * ->
 * Task Notification
 * ->
 * Protocol Parser
 * ->
 * Application Dispatch
 *
 * 没有 UART RX Event 时，
 * Task 完全阻塞。
 *
 * 不使用周期 Polling。
 */
static void
task_protocol_rx(
    void *parameters
)
{
    protocol_parser_t
        parser;


    protocol_frame_t
        received_frame;


    TickType_t
        previous_feedback_tick =
            0U;


    uint8_t
        feedback_time_valid =
            0U;


    (void)parameters;


    protocol_parser_init(
        &parser
    );


    /*
     * 到达这里时：
     *
     * 1. Scheduler 已启动；
     * 2. ProtocolRX Task 已运行；
     * 3. RX Event Callback 已注册。
     *
     * 此时才正式打开 UART RX Interrupt。
     */
    uart_driver_enable_rx_interrupt();


    for (;;)
    {
        uint8_t
            byte;


        /*
         * 下一次 UART RX ISR
         * 可以作为 Wakeup Measurement 起点。
         */
        performance_monitor_arm_task_wakeup();


        /*
         * 没有 RX Event 时完全阻塞。
         *
         * pdTRUE：
         * Task 被唤醒后清空 Notification Count。
         *
         * 即使期间发生多次 IRQ，
         * 后面仍会一次性消费 Ring Buffer
         * 中所有可用字节。
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


        performance_monitor_record_task_wakeup();


        /*
         * 每次被唤醒后，
         * 一次性消费 Ring Buffer
         * 中当前全部字节。
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
                protocol_tasks_dispatch_frame(
                    &received_frame,
                    &previous_feedback_tick,
                    &feedback_time_valid
                );
            }
        }
    }
}


/* =========================================================
 * Public Start API
 * ========================================================= */

robot_status_t
protocol_tasks_start(void)
{
    BaseType_t
        task_create_result;


    /*
     * 创建周期 TX Task。
     */
    task_create_result =
        xTaskCreate(
            task_protocol_tx,
            "ProtocolTX",
            TASK_STACK_DEPTH_PROTOCOL_TX,
            NULL,
            TASK_PRIORITY_PROTOCOL_TX,
            NULL
        );


    if (
        task_create_result
        !=
        pdPASS
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 创建事件驱动 RX Task。
     *
     * 保存 Task Handle，
     * UART RX ISR 将通过该 Handle
     * 发送 Task Notification。
     */
    task_create_result =
        xTaskCreate(
            task_protocol_rx,
            "ProtocolRX",
            TASK_STACK_DEPTH_PROTOCOL_RX,
            NULL,
            TASK_PRIORITY_PROTOCOL_RX,
            &protocol_rx_task_handle
        );


    if (
        task_create_result
        !=
        pdPASS
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Scheduler 尚未启动，
     * 因此此时两个 Task 都不会运行。
     *
     * 先注册 RX Callback，
     * 等 Scheduler 启动后，
     * ProtocolRX Task 才会打开 RX Interrupt。
     */
    uart_driver_set_rx_event_callback(
        uart_rx_event_from_isr
    );


    return
        ROBOT_STATUS_OK;
}