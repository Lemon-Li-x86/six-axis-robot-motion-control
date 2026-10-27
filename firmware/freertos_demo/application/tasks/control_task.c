/*
 * Stage 4 periodic feedback/PID consumer.
 * Motor access is ONLY through the existing synchronized motor_state API.
 * No command is emitted before request + fresh feedback. Output is gated
 * separately by ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT (default OFF).
 */
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_tasks_internal.h"
#include "task_config.h"
#include "joint_controller.h"
#include "joint_command_limiter.h"
#include "motor_state.h"
#include "motion_limits.h"
#include "angle_utils.h"
#include "robot_model.h"
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
#include "stage4_timing_probe.h"
#endif

static void task_control(void *arg)
{
    motion_runtime_t *r=(motion_runtime_t *)arg;
    joint_controller_t controller;
    joint_command_limiter_t limiter={0};
    motion_reference_t reference={0};
    TickType_t last_wake=xTaskGetTickCount();
    TickType_t previous_tick=last_wake;
    TickType_t period=pdMS_TO_TICKS(TASK_PERIOD_CONTROL_MS);
    uint8_t active=0U;

    if(joint_controller_init(&controller,r->config.pid)!=ROBOT_STATUS_OK) {
        motion_record_fault(ROBOT_STATUS_ERROR_INVALID_ARGUMENT,1U);
        vTaskDelete(NULL);
        return;
    }

    for(;;)
    {
        motor_state_observation_t observation;
        joint_control_corrections_t correction;
        robot_joint_positions_t candidate={{0}};
        robot_joint_angles_t canonical={{0}};
        TickType_t now;
        double dt;
        robot_status_t status;
        const robot_model_t *model;

        motion_task_wait_next_period(&last_wake,period);
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
        uint32_t timing_start=stage4_timing_start(STAGE4_TIMING_CONTROL);
#endif
        /* Always sample after a fault/empty-reference early exit. */
        do {
            now=xTaskGetTickCount();
            dt=(double)(now-previous_tick)/(double)configTICK_RATE_HZ;
            previous_tick=now;
            if(xQueueReceive(r->reference_queue,&reference,0U)==pdTRUE) {
                if(reference.enabled==0U) {
                    active=0U;
                    joint_command_limiter_reset(&limiter);
                    joint_controller_reset(&controller);
                } else active=1U;
            }
            if(!active) break;

            if((now-reference.created_tick)>
                 pdMS_TO_TICKS(r->config.reference_timeout_ms)) {
                motion_record_fault(ROBOT_STATUS_ERROR_NOT_READY,1U);
                active=0U; joint_command_limiter_reset(&limiter);
                joint_controller_reset(&controller);
                break;
            }
            status=motor_state_get_feedback(&observation);
            if(status!=ROBOT_STATUS_OK ||
               !motor_state_feedback_is_fresh(&observation,(uint32_t)now,
                  (uint32_t)pdMS_TO_TICKS(r->config.feedback_timeout_ms))) {
                taskENTER_CRITICAL();
                r->diagnostics.feedback_stale_count++;
                taskEXIT_CRITICAL();
                motion_record_fault(ROBOT_STATUS_ERROR_NOT_READY,1U);
                active=0U; joint_command_limiter_reset(&limiter);
                joint_controller_reset(&controller);
                break;
            }
            status=joint_controller_step(&controller,&reference.target,
                 &observation.feedback.unwrapped_positions,dt,&correction);
            if(status!=ROBOT_STATUS_OK) {
                motion_record_fault(status,1U);
                active=0U; joint_command_limiter_reset(&limiter);
                joint_controller_reset(&controller);
                break;
            }
            /*
             * The pure limiter owns fractional command state, applies BOTH
             * per-joint velocity and acceleration constraints, and rounds only
             * when publishing 0.01 degree motor positions.
             */
            model=robot_model_get_active();
            status=joint_command_limiter_step(
                &limiter,
                &observation.feedback.unwrapped_positions,
                &correction,
                &r->config.max_velocity_deg_s,
                &r->config.max_acceleration_deg_s2,
                model,
                dt,
                &candidate);
            if(status!=ROBOT_STATUS_OK) {
                motion_record_fault(status,1U);
                active=0U;
                joint_command_limiter_reset(&limiter);
                joint_controller_reset(&controller);
                break;
            }
            status=motion_limits_check_positions(&candidate);
            if(status==ROBOT_STATUS_OK)
                status=robot_angles_canonicalize(&candidate,&canonical);
            if(status!=ROBOT_STATUS_OK) {
                motion_record_fault(status,1U);
                active=0U; joint_command_limiter_reset(&limiter);
                break;
            }

#if ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT
            status=motor_state_set_targets(&canonical);
            if(status!=ROBOT_STATUS_OK) {
                motion_record_fault(status,1U);
                active=0U; joint_command_limiter_reset(&limiter);
                break;
            }
#else
            (void)canonical; /* algorithm/task dry-run; no actuator commands */
#endif
            taskENTER_CRITICAL();
            r->diagnostics.controller_steps++;
            taskEXIT_CRITICAL();
        } while(0);
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
        stage4_timing_end(STAGE4_TIMING_CONTROL,timing_start);
#endif
    }
}

BaseType_t control_task_create(void)
{
    return xTaskCreate(task_control,"Control",
        TASK_STACK_DEPTH_CONTROL,&g_motion_runtime,
        TASK_PRIORITY_CONTROL,&g_motion_runtime.control_handle);
}
