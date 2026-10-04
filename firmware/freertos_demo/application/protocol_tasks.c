/*
 * 文件：protocol_tasks.c
 *
 * 用途：
 * 实现机器人固件 Protocol Application Tasks。
 *
 * 当前负责：
 * 1. ProtocolTX 周期发送；
 * 2. ProtocolRX 事件驱动接收；
 * 3. UART RX ISR -> Task Notification；
 * 4. Joint State 处理与 ACK；
 * 5. Runtime Parameter Configuration；
 * 6. Diagnostics Request / Response。
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "protocol_tasks.h"
#include "task_config.h"
#include "runtime_config.h"
#include "uart_tx_manager.h"

#include "uart_driver.h"
#include "motor_driver.h"
#include "timer_driver.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"

#include "performance_monitor.h"


/* =========================================================
 * Task State
 * ========================================================= */

static TaskHandle_t protocol_rx_task_handle = NULL;


/* =========================================================
 * UART RX ISR Callback
 * ========================================================= */

static void uart_rx_event_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    performance_monitor_mark_rx_isr();

    if (protocol_rx_task_handle == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(
        protocol_rx_task_handle,
        &higher_priority_task_woken
    );

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


/* =========================================================
 * Joint State
 * ========================================================= */

static void protocol_tasks_handle_joint_state(
    const protocol_frame_t *frame,
    TickType_t *previous_feedback_tick,
    uint8_t *feedback_time_valid
)
{
    robot_joint_angles_t received_joint_states;
    robot_joint_angles_t stored_joint_states;

    uint8_t ack_frame[PROTOCOL_JOINT_FRAME_LEN];

    TickType_t current_feedback_tick;
    robot_real_t delta_time_s;

    if (frame == NULL ||
        previous_feedback_tick == NULL ||
        feedback_time_valid == NULL)
    {
        return;
    }

    if (protocol_parse_joint_state(
            frame,
            &received_joint_states) != ROBOT_STATUS_OK)
    {
        return;
    }

    current_feedback_tick = xTaskGetTickCount();

    if (*feedback_time_valid)
    {
        TickType_t elapsed_ticks =
            current_feedback_tick - *previous_feedback_tick;

        delta_time_s =
            (robot_real_t)elapsed_ticks /
            (robot_real_t)configTICK_RATE_HZ;
    }
    else
    {
        delta_time_s = 0.0F;
    }

    /*
     * Motor Driver 负责角度规范化、
     * 连续位置展开和速度计算。
     */
    if (motor_driver_update_feedback(
            &received_joint_states,
            delta_time_s) != ROBOT_STATUS_OK)
    {
        return;
    }

    *previous_feedback_tick = current_feedback_tick;
    *feedback_time_valid = 1U;

    /*
     * ACK 使用已经进入 Motor Driver 的状态，
     * 而不是直接回显收到的 Payload。
     */
    if (motor_driver_get_positions(
            &stored_joint_states) != ROBOT_STATUS_OK)
    {
        return;
    }

    if (protocol_build_joint_state_ack_frame(
            &stored_joint_states,
            ack_frame) != ROBOT_STATUS_OK)
    {
        return;
    }

    (void)uart_tx_manager_send_frame(
        ack_frame,
        PROTOCOL_JOINT_FRAME_LEN
    );
}


/* =========================================================
 * Parameter Configuration
 * ========================================================= */

static void protocol_tasks_handle_set_parameter(
    const protocol_frame_t *frame
)
{
    uint8_t parameter_id;
    uint32_t requested_value;
    uint32_t effective_value = 0U;

    robot_status_t status;

    uint8_t ack_frame[PROTOCOL_PARAMETER_ACK_FRAME_LEN];

    if (frame == NULL)
    {
        return;
    }

    status = protocol_parse_set_parameter(
        frame,
        &parameter_id,
        &requested_value
    );

    if (status != ROBOT_STATUS_OK)
    {
        return;
    }

    switch (parameter_id)
    {
        case PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS:

            status = runtime_config_set_protocol_tx_period_ms(
                requested_value
            );

            /*
             * 即使配置失败，也返回当前真正生效的值。
             */
            effective_value =
                runtime_config_get_protocol_tx_period_ms();

            break;

        default:

            status = ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            effective_value = 0U;

            break;
    }

    if (protocol_build_parameter_ack_frame(
            parameter_id,
            status,
            effective_value,
            ack_frame) != ROBOT_STATUS_OK)
    {
        return;
    }

    (void)uart_tx_manager_send_frame(
        ack_frame,
        PROTOCOL_PARAMETER_ACK_FRAME_LEN
    );
}


/* =========================================================
 * Diagnostics
 * ========================================================= */

static robot_status_t protocol_tasks_get_diagnostics_value(
    uint8_t selector,
    uint32_t *value
)
{
    performance_metrics_t metrics;
    robot_status_t status;

    if (value == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    /*
     * 直接来自 Driver 的指标无需读取
     * Performance Monitor Snapshot。
     */
    switch (selector)
    {
        case DIAGNOSTICS_METRIC_RX_DROP_COUNT:

            *value = uart_driver_get_rx_drop_count();
            return ROBOT_STATUS_OK;

        case DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ:

            *value = timer_driver_get_frequency_hz();
            return ROBOT_STATUS_OK;

        default:

            break;
    }

    status = performance_monitor_get_metrics(&metrics);

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    switch (selector)
    {
        case DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT:

            *value = metrics.parser_sample_count;
            break;

        case DIAGNOSTICS_METRIC_PARSER_MIN_TICKS:

            *value = metrics.parser_min_ticks;
            break;

        case DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS:

            *value = metrics.parser_average_ticks;
            break;

        case DIAGNOSTICS_METRIC_PARSER_MAX_TICKS:

            *value = metrics.parser_max_ticks;
            break;

        case DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT:

            *value = metrics.task_wakeup_sample_count;
            break;

        case DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS:

            *value = metrics.task_wakeup_min_ticks;
            break;

        case DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS:

            *value = metrics.task_wakeup_average_ticks;
            break;

        case DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS:

            *value = metrics.task_wakeup_max_ticks;
            break;

        default:

            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }

    return ROBOT_STATUS_OK;
}


static void protocol_tasks_handle_diagnostics(
    const protocol_frame_t *frame
)
{
    uint8_t selector;
    uint32_t value;

    uint8_t response_frame[PROTOCOL_DIAGNOSTICS_FRAME_LEN];

    if (frame == NULL)
    {
        return;
    }

    if (protocol_parse_diagnostics_request(
            frame,
            &selector) != ROBOT_STATUS_OK)
    {
        return;
    }

    if (protocol_tasks_get_diagnostics_value(
            selector,
            &value) != ROBOT_STATUS_OK)
    {
        return;
    }

    if (protocol_build_diagnostics_response_frame(
            value,
            response_frame) != ROBOT_STATUS_OK)
    {
        return;
    }

    (void)uart_tx_manager_send_frame(
        response_frame,
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    );
}


/* =========================================================
 * Application Dispatch
 * ========================================================= */

static void protocol_tasks_dispatch_frame(
    const protocol_frame_t *frame,
    TickType_t *previous_feedback_tick,
    uint8_t *feedback_time_valid
)
{
    if (frame == NULL)
    {
        return;
    }

    switch (frame->command)
    {
        case CMD_JOINT_STATE:

            protocol_tasks_handle_joint_state(
                frame,
                previous_feedback_tick,
                feedback_time_valid
            );

            break;

        case CMD_SET_PARAMETER:

            protocol_tasks_handle_set_parameter(frame);

            break;

        case CMD_GET_DIAGNOSTICS:

            protocol_tasks_handle_diagnostics(frame);

            break;

        default:

            /*
             * 未知 Command 或方向错误的 Command
             * 不产生业务行为。
             */
            break;
    }
}


/* =========================================================
 * Protocol TX Task
 * ========================================================= */

static void task_protocol_tx(void *parameters)
{
    robot_joint_angles_t joint_targets;
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN];

    (void)parameters;

    for (;;)
    {
        if (motor_driver_get_target_positions(
                &joint_targets) == ROBOT_STATUS_OK)
        {
            if (protocol_build_joint_target_frame(
                    &joint_targets,
                    frame) == ROBOT_STATUS_OK)
            {
                (void)uart_tx_manager_send_frame(
                    frame,
                    PROTOCOL_JOINT_FRAME_LEN
                );
            }
        }

        /*
         * 周期允许通过 SET_PARAMETER
         * 在运行时修改。
         */
        vTaskDelay(
            pdMS_TO_TICKS(
                runtime_config_get_protocol_tx_period_ms()
            )
        );
    }
}


/* =========================================================
 * Protocol RX Task
 * ========================================================= */

static void task_protocol_rx(void *parameters)
{
    protocol_parser_t parser;
    protocol_frame_t received_frame;

    TickType_t previous_feedback_tick = 0U;
    uint8_t feedback_time_valid = 0U;

    (void)parameters;

    protocol_parser_init(&parser);

    /*
     * Task 已经创建并进入 Scheduler 后
     * 再打开 UART RX Interrupt。
     */
    uart_driver_enable_rx_interrupt();

    for (;;)
    {
        uint8_t byte;

        performance_monitor_arm_task_wakeup();

        /*
         * 没有 UART RX Event 时完全阻塞。
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        performance_monitor_record_task_wakeup();

        /*
         * 每次被唤醒后消费 Ring Buffer
         * 中当前所有待处理字节。
         */
        while (uart_driver_read_byte(&byte))
        {
            if (protocol_parser_process_byte(
                    &parser,
                    byte,
                    &received_frame))
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
 * Public API
 * ========================================================= */

robot_status_t protocol_tasks_start(void)
{
    BaseType_t result;

    result = xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        TASK_STACK_DEPTH_PROTOCOL_TX,
        NULL,
        TASK_PRIORITY_PROTOCOL_TX,
        NULL
    );

    if (result != pdPASS)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    result = xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        TASK_STACK_DEPTH_PROTOCOL_RX,
        NULL,
        TASK_PRIORITY_PROTOCOL_RX,
        &protocol_rx_task_handle
    );

    if (result != pdPASS)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * ISR Callback 先注册，
     * RX Interrupt 由 ProtocolRX Task
     * 真正运行后开启。
     */
    uart_driver_set_rx_event_callback(
        uart_rx_event_from_isr
    );

    return ROBOT_STATUS_OK;
}