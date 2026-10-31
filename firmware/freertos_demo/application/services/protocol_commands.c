
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
#include "motor_state.h"
#include "timer_driver.h"

#include "performance_monitor.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"
#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
#include "motion_pipeline.h"
#include "motion_limits.h"
#include "stage4_motion_config.h"
#include "kinematics.h"
#include "angle_utils.h"
#include "potential_field.h"
#endif


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
    motor_state_observation_t stored_feedback;

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
        motor_state_update_feedback(
            &received_joint_states,
            delta_time_s,
            current_tick
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
        motor_state_get_feedback(
            &stored_feedback
        ) != ROBOT_STATUS_OK
    )
    {
        return;
    }

    if (
        protocol_build_joint_state_ack_frame(
            &stored_feedback.feedback.canonical_positions,
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

#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
    /* Existing UART diagnostic frame; QEMU test only extends selectors,
     * with all values read from the authoritative motion pipeline. */
    if(selector>=DIAGNOSTICS_METRIC_MOTION_ACCEPTED &&
       selector<=DIAGNOSTICS_METRIC_MONITORING_TICKS)
    {
        motion_stage4_diagnostics_t motion={0};
        status=motion_pipeline_get_diagnostics(&motion);
        if(status!=ROBOT_STATUS_OK) return status;
        switch(selector) {
            case DIAGNOSTICS_METRIC_MOTION_ACCEPTED:
                *value=motion.requests_accepted; break;
            case DIAGNOSTICS_METRIC_TRAJECTORY_SAMPLES:
                *value=motion.trajectory_samples; break;
            case DIAGNOSTICS_METRIC_CONTROLLER_STEPS:
                *value=motion.controller_steps; break;
            case DIAGNOSTICS_METRIC_CONTROLLER_FAULTS:
                *value=motion.controller_faults; break;
            case DIAGNOSTICS_METRIC_PLANNER_FAULTS:
                *value=motion.planner_faults; break;
            case DIAGNOSTICS_METRIC_FEEDBACK_STALE:
                *value=motion.feedback_stale_count; break;
            case DIAGNOSTICS_METRIC_MOTION_LAST_FAULT:
                *value=(uint32_t)(int32_t)motion.last_fault; break;
            case DIAGNOSTICS_METRIC_MONITORING_TICKS:
                *value=motion.monitoring_ticks; break;
            default:
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        }
        return ROBOT_STATUS_OK;
    }
#endif

    /* Unknown selectors must not trigger an unnecessary metrics snapshot. */
    switch (selector)
    {
        case DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT:
        case DIAGNOSTICS_METRIC_PARSER_MIN_TICKS:
        case DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS:
        case DIAGNOSTICS_METRIC_PARSER_MAX_TICKS:
        case DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT:
        case DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS:
        case DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS:
        case DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS:
            break;
        default:
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
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


#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
/* QEMU simulation ONLY. The existing target frame is firmware->PyBullet;
 * this separate command is PyBullet->firmware and its ACK only confirms
 * queue admission, never physical path completion. No actuation on startup. */
static void handle_simulated_joint_move(
    const protocol_frame_t *frame,
    uint32_t now_tick,
    uint32_t tick_rate_hz)
{
    robot_joint_angles_t requested={{0}};
    robot_joint_positions_t continuous={{0}};
    motor_state_observation_t feedback={0};
    robot_status_t status;
    uint8_t sequence;
    uint8_t ack[PROTOCOL_MOVE_ACK_FRAME_LEN];
    const motion_stage4_config_t *config=stage4_motion_default_config();
    uint64_t max_age;
    if(frame==NULL || frame->length<1U)
        return; /* no sequence byte to acknowledge */
    sequence=frame->payload[0];
    status=protocol_parse_joint_move_request(frame,&sequence,&requested);
    if(status==ROBOT_STATUS_OK) {
        if(tick_rate_hz==0U) status=ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        else {
            max_age=((uint64_t)config->feedback_timeout_ms*
                     (uint64_t)tick_rate_hz)/1000U;
            if(max_age==0U || max_age>0x7FFFFFFFU)
                status=ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            else {
                status=motor_state_get_feedback(&feedback);
                if(status==ROBOT_STATUS_OK &&
                   !motor_state_feedback_is_fresh(
                      &feedback,now_tick,(uint32_t)max_age))
                    status=ROBOT_STATUS_ERROR_NOT_READY;
            }
        }
    }
    if(status==ROBOT_STATUS_OK)
        status=motion_limits_unwrap_nearest(
            &requested,&feedback.feedback.unwrapped_positions,&continuous);
    if(status==ROBOT_STATUS_OK)
        status=motion_pipeline_submit_joint(&continuous);
    if(protocol_build_joint_move_ack_frame(sequence,status,ack)
       ==ROBOT_STATUS_OK)
        (void)uart_tx_manager_send_frame(ack,PROTOCOL_MOVE_ACK_FRAME_LEN);
}
/* Explicit QEMU-only APF step executed by the ARM firmware, not a Python
 * potential-field replica. No motor command, queue admission or implicit
 * path execution. The host MUST still validate C IK and actual full links
 * before a separate CMD_REQUEST_JOINT_MOVE may be sent. */
static void handle_simulated_apf_step(
    const protocol_frame_t *frame, uint32_t now_tick, uint32_t tick_rate_hz)
{
    protocol_apf_step_request_t request={0};
    motor_state_observation_t feedback={0};
    robot_joint_angles_t canonical={{0}};
    robot_transform_t transform={{{0}}};
    robot_apf_point_t current={{0}}, goal={{0}}, next={{0}};
    robot_apf_aabb_t obstacle={0};
    robot_apf_config_t config={0};
    robot_apf_step_info_t info={0};
    const motion_stage4_config_t *motion=stage4_motion_default_config();
    uint8_t sequence=0U;
    uint8_t response[PROTOCOL_APF_STEP_RESPONSE_FRAME_LEN];
    robot_status_t status;
    uint64_t max_age;
    float coordinates_mm[3]={0.0F,0.0F,0.0F};
    size_t axis;

    if(frame==NULL || frame->length<1U)
        return;
    sequence=frame->payload[0];
    status=protocol_parse_apf_step_request(frame,&request);
    if(status==ROBOT_STATUS_OK) {
        if(tick_rate_hz==0U)
            status=ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        else {
            max_age=((uint64_t)motion->feedback_timeout_ms*
                     (uint64_t)tick_rate_hz)/1000U;
            if(max_age==0U || max_age>0x7FFFFFFFU)
                status=ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            else {
                status=motor_state_get_feedback(&feedback);
                if(status==ROBOT_STATUS_OK &&
                   !motor_state_feedback_is_fresh(
                      &feedback,now_tick,(uint32_t)max_age))
                    status=ROBOT_STATUS_ERROR_NOT_READY;
            }
        }
    }
    if(status==ROBOT_STATUS_OK)
        status=robot_angles_canonicalize(
            &feedback.feedback.unwrapped_positions,&canonical);
    if(status==ROBOT_STATUS_OK)
        status=kinematics_forward(&canonical,&transform);
    if(status==ROBOT_STATUS_OK) {
        config.attraction_gain=(double)request.apf_parameters[0];
        config.repulsion_gain=(double)request.apf_parameters[1];
        config.influence_distance_mm=(double)request.apf_parameters[2];
        config.clearance_mm=(double)request.apf_parameters[3];
        config.maximum_step_mm=(double)request.apf_parameters[4];
        config.goal_tolerance_mm=(double)request.apf_parameters[5];
        for(axis=0U;axis<3U;axis++) {
            current.mm[axis]=(double)transform.matrix[axis][3];
            goal.mm[axis]=current.mm[axis]+
                (double)request.goal_offset_mm[axis];
            obstacle.min_corner.mm[axis]=current.mm[axis]+
                (double)request.obstacle_min_offset_mm[axis];
            obstacle.max_corner.mm[axis]=current.mm[axis]+
                (double)request.obstacle_max_offset_mm[axis];
        }
        status=robot_apf_step(
            &current,&goal,&obstacle,1U,&config,&next,&info);
        if(status==ROBOT_STATUS_OK) {
            for(axis=0U;axis<3U;axis++)
                coordinates_mm[axis]=(float)next.mm[axis];
        }
    }
    if(protocol_build_apf_step_response(sequence,status,
           status==ROBOT_STATUS_OK ? coordinates_mm : NULL,
           response)==ROBOT_STATUS_OK)
        (void)uart_tx_manager_send_frame(
            response,PROTOCOL_APF_STEP_RESPONSE_FRAME_LEN);
}
#endif

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

#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
        case CMD_REQUEST_JOINT_MOVE:
            handle_simulated_joint_move(frame,current_tick,tick_rate_hz);
            break;
        case CMD_REQUEST_APF_STEP:
            handle_simulated_apf_step(frame,current_tick,tick_rate_hz);
            break;
#endif

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
