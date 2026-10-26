/*
 * Stage 4 periodic feedback/PID consumer.
 * Motor access is ONLY through the existing synchronized motor_state API.
 * No command is emitted before request + fresh feedback. Output is gated
 * separately by ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT (default OFF).
 */
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_tasks_internal.h"
#include "task_config.h"
#include "joint_controller.h"
#include "motor_state.h"
#include "motion_limits.h"
#include "angle_utils.h"
#include "robot_model.h"

static void task_control(void *arg)
{
    motion_runtime_t *r=(motion_runtime_t *)arg;
    joint_controller_t controller;
    motion_reference_t reference={0};
    TickType_t last_wake=xTaskGetTickCount();
    TickType_t previous_tick=last_wake;
    TickType_t period=pdMS_TO_TICKS(TASK_PERIOD_CONTROL_MS);
    uint8_t active=0U;
    robot_joint_positions_t last_command={0};
    uint8_t last_command_valid=0U;

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
        uint32_t j;
        robot_status_t status;
        const robot_model_t *model;

        vTaskDelayUntil(&last_wake,(period==0U)?1U:period);
        now=xTaskGetTickCount();
        dt=(double)(now-previous_tick)/(double)configTICK_RATE_HZ;
        previous_tick=now;
        if(xQueueReceive(r->reference_queue,&reference,0U)==pdTRUE) {
            if(reference.enabled==0U) {
                active=0U;
                last_command_valid=0U;
                joint_controller_reset(&controller);
            } else active=1U;
        }
        if(!active) continue;

        if((now-reference.created_tick)>
             pdMS_TO_TICKS(3U*TASK_PERIOD_TRAJECTORY_MS)) {
            motion_record_fault(ROBOT_STATUS_ERROR_NOT_READY,1U);
            active=0U; last_command_valid=0U;
            joint_controller_reset(&controller);
            continue;
        }
        status=motor_state_get_feedback(&observation);
        if(status!=ROBOT_STATUS_OK ||
           !motor_state_feedback_is_fresh(&observation,(uint32_t)now,
              (uint32_t)pdMS_TO_TICKS(r->config.feedback_timeout_ms))) {
            taskENTER_CRITICAL();
            r->diagnostics.feedback_stale_count++;
            taskEXIT_CRITICAL();
            active=0U; last_command_valid=0U;
            joint_controller_reset(&controller);
            continue;
        }
        status=joint_controller_step(&controller,&reference.target,
             &observation.feedback.unwrapped_positions,dt,&correction);
        if(status!=ROBOT_STATUS_OK) {
            motion_record_fault(status,1U);
            active=0U; last_command_valid=0U;
            joint_controller_reset(&controller);
            continue;
        }
        model=robot_model_get_active();
        if(model==NULL) {
            motion_record_fault(ROBOT_STATUS_ERROR_INTERNAL,1U);
            active=0U;
            continue;
        }

        for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
            double proposal=(double)observation.feedback.unwrapped_positions.value[j]+
              (double)correction.value[j]/
              (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
            double high=(double)model->joint_max_positions.value[j];
            double low=(double)model->joint_min_positions.value[j];
            double max_delta=(double)r->config.max_velocity_deg_s.value[j]*
                dt/(double)ROBOT_JOINT_ANGLE_UNIT_DEG;
            if(!isfinite(proposal) || !isfinite(max_delta)) {
                status=ROBOT_STATUS_ERROR_OUT_OF_RANGE;
                break;
            }
            if(last_command_valid) {
                double upper=(double)last_command.value[j]+max_delta;
                double lower=(double)last_command.value[j]-max_delta;
                if(proposal>upper) proposal=upper;
                if(proposal<lower) proposal=lower;
            } else {
                /* First command ramps from measured feedback; no jump. */
                double fb=(double)observation.feedback.unwrapped_positions.value[j];
                if(proposal>fb+max_delta) proposal=fb+max_delta;
                if(proposal<fb-max_delta) proposal=fb-max_delta;
            }
            if(proposal>high) proposal=high;
            if(proposal<low) proposal=low;
            if(proposal>(double)INT32_MAX || proposal<(double)INT32_MIN) {
                status=ROBOT_STATUS_ERROR_OUT_OF_RANGE;
                break;
            }
            candidate.value[j]=(robot_joint_position_t)llround(proposal);
        }
        if(j!=ROBOT_JOINT_COUNT) {
            motion_record_fault(ROBOT_STATUS_ERROR_OUT_OF_RANGE,1U);
            active=0U; last_command_valid=0U;
            continue;
        }
        status=motion_limits_check_positions(&candidate);
        if(status==ROBOT_STATUS_OK)
            status=robot_angles_canonicalize(&candidate,&canonical);
        if(status!=ROBOT_STATUS_OK) {
            motion_record_fault(status,1U);
            active=0U; last_command_valid=0U;
            continue;
        }

#if ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT
        status=motor_state_set_targets(&canonical);
        if(status!=ROBOT_STATUS_OK) {
            motion_record_fault(status,1U);
            active=0U; last_command_valid=0U;
            continue;
        }
#else
        (void)canonical; /* algorithm/task dry-run; no actuator commands */
#endif
        last_command=candidate;
        last_command_valid=1U;
        taskENTER_CRITICAL();
        r->diagnostics.controller_steps++;
        taskEXIT_CRITICAL();
    }
}

BaseType_t control_task_create(void)
{
    return xTaskCreate(task_control,"Control",
        TASK_STACK_DEPTH_CONTROL,&g_motion_runtime,
        TASK_PRIORITY_CONTROL,&g_motion_runtime.control_handle);
}
