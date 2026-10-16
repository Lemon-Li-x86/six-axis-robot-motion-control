
/*
 * File: protocol_commands.c
 *
 * Application-level protocol command handling.
 *
 * This module does not create FreeRTOS tasks or
 * receive UART bytes. It processes complete frames
 * supplied by the protocol receiving task.
 */

#include <stddef.h>
#include <stdint.h>

#include "protocol_commands.h"

#include "runtime_config.h"
#include "uart_tx_manager.h"

#include "uart_driver.h"
#include "motor_driver.h"
#include "timer_driver.h"

#include "performance_monitor.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * Command Context
 * ========================================================= */

void protocol_commands_init(
    protocol_command_context_t *context
)
{
    if (context == NULL)
    {
        return;
    }

    context->previous_feedback_tick = 0U;
    context->feedback_time_valid = 0U;
}


/* =========================================================
 * Joint State
 * ========================================================= */

static void handle_joint_state(
    const protocol_frame_t *frame,
    protocol_command_context_t *context,
    uint32_t current_tick,
    uint32_t tick_rate_hz
)
{
    robot_joint_angles_t received_joint_states;
    robot_joint_angles_t stored_joint_states;

    uint8_t ack_frame[PROTOCOL_JOINT_FRAME_LEN];

    robot_real_t delta_time_s;

    if (
        frame == NULL ||
        context == NULL ||
        tick_rate_hz == 0U
    )
    {
        return;
    }

    /*
     * Only valid joint-state frames may update
     * the stored feedback timestamp.
     */
    if (
        protocol_parse_joint_state(
            frame,
            &received_joint_states
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    if (context->feedback_time_valid != 0U)
    {
        /*
         * Unsigned subtraction handles normal
         * 32-bit tick-counter wraparound.
         */
        uint32_t elapsed_ticks =
            current_tick -
            context->previous_feedback_tick;

        delta_time_s =
            (robot_real_t)elapsed_ticks /
            (robot_real_t)tick_rate_hz;
    }
    else
    {
        delta_time_s = 0.0F;
    }

    /*
     * Motor Driver maintains canonical feedback,
     * unwrapped positions and estimated velocities.
     */
    if (
        motor_driver_update_feedback(
            &received_joint_states,
            delta_time_s
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    /*
     * Update timing state only after successful
     * feedback processing.
     */
    context->previous_feedback_tick =
        current_tick;

    context->feedback_time_valid = 1U;

    /*
     * The ACK reflects the accepted Motor Driver
     * state, not the raw received payload.
     */
    if (
        motor_driver_get_positions(
            &stored_joint_states
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    if (
        protocol_build_joint_state_ack_frame(
            &stored_joint_states,
            ack_frame
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    (void)uart_tx_manager_send_frame(
        ack_frame,
        PROTOCOL_JOINT_FRAME_LEN
    );
}


/* =========================================================
 * Runtime Parameter Configuration
 * ========================================================= */

static void handle_set_parameter(
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

            status =
                runtime_config_set_protocol_tx_period_ms(
                    requested_value
                );

            /*
             * Always return the actual effective
             * value, including when setting fails.
             */
            effective_value =
                runtime_config_get_protocol_tx_period_ms();

            break;

        default:

            status =
                ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

            effective_value = 0U;

            break;
    }

    if (
        protocol_build_parameter_ack_frame(
            parameter_id,
            status,
            effective_value,
            ack_frame
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    (void)uart_tx_manager_send_frame(
        ack_frame,
        PROTOCOL_PARAMETER_ACK_FRAME_LEN
    );
}


/* =========================================================
 * Diagnostics Value Lookup
 * ========================================================= */

static robot_status_t get_diagnostics_value(
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
     * Driver-owned metrics do not require a
     * Performance Monitor snapshot.
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

    status =
        performance_monitor_get_metrics(&metrics);

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


/* =========================================================
 * Diagnostics Request
 * ========================================================= */

static void handle_diagnostics(
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

    if (
        protocol_parse_diagnostics_request(
            frame,
            &selector
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    if (
        get_diagnostics_value(
            selector,
            &value
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    if (
        protocol_build_diagnostics_response_frame(
            value,
            response_frame
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    (void)uart_tx_manager_send_frame(
        response_frame,
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    );
}


/* =========================================================
 * Public Command Dispatcher
 * ========================================================= */

void protocol_commands_dispatch_frame(
    const protocol_frame_t *frame,
    protocol_command_context_t *context,
    uint32_t current_tick,
    uint32_t tick_rate_hz
)
{
    if (frame == NULL || context == NULL)
    {
        return;
    }

    switch (frame->command)
    {
        case CMD_JOINT_STATE:

            handle_joint_state(
                frame,
                context,
                current_tick,
                tick_rate_hz
            );

            break;

        case CMD_SET_PARAMETER:

            handle_set_parameter(frame);

            break;

        case CMD_GET_DIAGNOSTICS:

            handle_diagnostics(frame);

            break;

        default:

            /*
             * Unknown or incorrectly directed
             * commands have no application effect.
             */
            break;
    }
}
