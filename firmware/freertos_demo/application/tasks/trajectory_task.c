/*
 * Periodic trajectory producer; low-level math stays in algorithms/.
 * Consumes one-slot request queue and publishes latest continuous target.
 * Does not call UART/motor driver or change robot runtime configuration.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_tasks_internal.h"
#include "task_config.h"
#include "motor_state.h"
#include "trajectory_request_adapter.h"
#include "kinematics.h"
#include "motion_limits.h"
#include "angle_utils.h"
#include "robot_model.h"
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
#include "stage4_timing_probe.h"
#endif

static void cancel_motion(motion_runtime_t *r,robot_status_t status)
{
    motion_reference_t cancel={0};
    cancel.created_tick=xTaskGetTickCount();
    cancel.enabled=0U;
    (void)xQueueOverwrite(r->reference_queue,&cancel);
    motion_record_fault(status,0U);
}

/* Verify the actual chosen/quantized IK branch reconstructs its
 * requested Cartesian waypoint. Merely having one IK solution is not enough.
 * Thresholds are injected through the application config. */
static robot_status_t verify_waypoint_fk(
    const motion_runtime_t *r,
    const robot_joint_positions_t *chosen,
    const robot_transform_t *requested)
{
    robot_joint_angles_t canonical={{0}};
    robot_transform_t reconstructed={{{0}}};
    robot_status_t status;
    status=robot_angles_canonicalize(chosen,&canonical);
    if(status!=ROBOT_STATUS_OK) return status;
    status=kinematics_forward(&canonical,&reconstructed);
    if(status!=ROBOT_STATUS_OK) return status;
    return kinematics_pose_within_tolerance(
        &reconstructed,requested,
        r->config.cartesian_waypoint_position_tolerance_mm,
        r->config.cartesian_waypoint_rotation_matrix_tolerance);
}

static robot_status_t start_request(
    motion_runtime_t *r, const motion_request_t *request,
    trajectory_trapezoid_plan_t *joint_plan,
    robot_joint_positions_t *last_joint)
{
    motor_state_observation_t observation;
    robot_status_t s;
    TickType_t now=xTaskGetTickCount();
    if(motor_state_get_feedback(&observation)!=ROBOT_STATUS_OK)
        return ROBOT_STATUS_ERROR_NOT_READY;
    if(!motor_state_feedback_is_fresh(&observation,(uint32_t)now,
          (uint32_t)pdMS_TO_TICKS(r->config.feedback_timeout_ms)))
        return ROBOT_STATUS_ERROR_NOT_READY;
    *last_joint=observation.feedback.unwrapped_positions;
    if(request->kind==MOTION_REQUEST_JOINT) {
        ik_selection_result_t selected={0};
        selected.target_positions=request->joint_goal;
        return trajectory_request_from_ik(last_joint,&selected,
            &r->config.max_velocity_deg_s,
            &r->config.max_acceleration_deg_s2,joint_plan);
    }
    if(request->kind==MOTION_REQUEST_POLYNOMIAL) {
        uint32_t j;
        if(motion_limits_check_positions(
             &request->polynomial.goal_positions)!=ROBOT_STATUS_OK)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        /* Trajectory must originate at the observed feedback snapshot.
         * No silent instantaneous jump to a caller-selected start. */
        for(j=0U;j<ROBOT_JOINT_COUNT;j++)
            if(request->polynomial.start_positions.value[j]!=last_joint->value[j])
                return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        /* Validate the complete curve before publishing any reference.
         * Periodic sampling alone can miss an interior rate/position peak.
         * Runs on the planner stack, not the smaller UART receive stack. */
        {
            const robot_model_t *model = robot_model_get_active();
            if(model==NULL) return ROBOT_STATUS_ERROR_INTERNAL;
            return trajectory_polynomial_validate_limits(
                &request->polynomial,
                &model->joint_min_positions,&model->joint_max_positions,
                &r->config.max_velocity_deg_s,
                &r->config.max_acceleration_deg_s2);
        }
    }
    if(request->kind==MOTION_REQUEST_CARTESIAN) {
        /* Reject a large discontinuity between robot feedback and
         * prescribed first Cartesian point; otherwise the first command
         * could jump before interpolating. Orientation discontinuities
         * are additionally checked by the first IK waypoint. */
        robot_joint_angles_t canonical;
        robot_transform_t actual;
        s=robot_angles_canonicalize(last_joint,&canonical);
        if(s!=ROBOT_STATUS_OK) return s;
        s=kinematics_forward(&canonical,&actual);
        if(s!=ROBOT_STATUS_OK) return s;
        return kinematics_pose_within_tolerance(
            &actual,&request->cartesian.start,
            r->config.cartesian_start_tolerance_mm,
            r->config.cartesian_start_rotation_matrix_tolerance);
    }
    return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
}

static void task_trajectory(void *arg)
{
    motion_runtime_t *r=(motion_runtime_t *)arg;
    trajectory_trapezoid_plan_t joint_plan={0};
    motion_request_t request={0};
    motion_reference_t reference={0};
    robot_joint_positions_t last={0};
    TickType_t last_wake=xTaskGetTickCount();
    TickType_t begun=0U;
    uint32_t cartesian_index=0U;
    uint32_t sequence=0U;
    uint8_t armed=0U;
    uint8_t moving=0U;
    motion_request_kind_t mode=MOTION_REQUEST_JOINT;
    const TickType_t interval=pdMS_TO_TICKS(TASK_PERIOD_TRAJECTORY_MS);

    for(;;)
    {
        robot_status_t s=ROBOT_STATUS_OK;
        motion_task_wait_next_period(&last_wake,interval);
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
        uint32_t timing_start=stage4_timing_start(STAGE4_TIMING_TRAJECTORY);
#endif
        /* Each early exit still participates in period/execution records. */
        do {

            /* Any control-loop safety fault cancels the entire trajectory.
             * Do not re-arm merely because the reference queue repeats data. */
            if(r->abort_requested!=0U) {
                taskENTER_CRITICAL();
                r->abort_requested=0U;
                taskEXIT_CRITICAL();
                reference.enabled=0U;
                reference.created_tick=xTaskGetTickCount();
                (void)xQueueOverwrite(r->reference_queue,&reference);
                armed=0U;
                moving=0U;
                break;
            }

            if(xQueueReceive(r->request_queue,&request,0U)==pdTRUE) {
                armed=0U;
                moving=0U;
                /* A disable reference can be overwritten by the first enabled
                 * sample before Control next wakes. Carry a persistent request
                 * generation so Control cannot miss the required state reset. */
                reference.request_generation++;
                /* Invalidate previously streamed target BEFORE replanning. */
                reference.enabled=0U;
                reference.created_tick=xTaskGetTickCount();
                (void)xQueueOverwrite(r->reference_queue,&reference);
                s=start_request(r,&request,&joint_plan,&last);
                if(s!=ROBOT_STATUS_OK) {
                    cancel_motion(r,s);
                    break;
                }
                mode=request.kind;
                begun=xTaskGetTickCount();
                cartesian_index=0U;
                armed=1U;
                moving=1U;
            }

            if(!armed) break;

            /* The planner must also stop on stale feedback; an old trajectory
             * must not resume at a later, jumped setpoint after reconnect. */
            {
                motor_state_observation_t observation;
                TickType_t now=xTaskGetTickCount();
                if(motor_state_get_feedback(&observation)!=ROBOT_STATUS_OK ||
                   !motor_state_feedback_is_fresh(&observation,(uint32_t)now,
                        (uint32_t)pdMS_TO_TICKS(r->config.feedback_timeout_ms))) {
                    cancel_motion(r,ROBOT_STATUS_ERROR_NOT_READY);
                    armed=0U; moving=0U;
                    break;
                }
            }

            if(moving)
            {
                if(mode==MOTION_REQUEST_JOINT) {
                    trajectory_joint_sample_t sample;
                    double t=(double)(xTaskGetTickCount()-begun) /
                        (double)configTICK_RATE_HZ;
                    s=trajectory_trapezoid_sample(&joint_plan,t,&sample);
                    if(s==ROBOT_STATUS_OK) {
                        last=sample.positions;
                        if(t>=joint_plan.total_time_s) moving=0U;
                    }
                } else if(mode==MOTION_REQUEST_POLYNOMIAL) {
                    trajectory_joint_sample_t sample;
                    double t=(double)(xTaskGetTickCount()-begun) /
                        (double)configTICK_RATE_HZ;
                    uint32_t j;
                    s=trajectory_polynomial_sample(
                        &request.polynomial,t,&sample);
                    if(s==ROBOT_STATUS_OK)
                        s=motion_limits_check_positions(&sample.positions);
                    if(s==ROBOT_STATUS_OK) {
                        for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
                            if(fabs((double)sample.velocities.value[j])>
                                   (double)r->config.max_velocity_deg_s.value[j] ||
                               fabs((double)sample.accelerations.value[j])>
                                   (double)r->config.max_acceleration_deg_s2.value[j]) {
                                s=ROBOT_STATUS_ERROR_OUT_OF_RANGE;
                                break;
                            }
                        }
                    }
                    if(s==ROBOT_STATUS_OK) {
                        last=sample.positions;
                        if(t>=request.polynomial.duration_s) moving=0U;
                    }
                } else {
                    robot_joint_positions_t next={0};
                    robot_transform_t sampled_pose;
                    /* IK+selector runs at each exact interpolation index;
                     * rejects unavailable/limit/singular samples without
                     * blindly reusing the previous solution. */
                    s=cartesian_trajectory_joint_waypoint(&request.cartesian,
                        cartesian_index,&last,&next,&sampled_pose);
                    if(s==ROBOT_STATUS_OK)
                        s=verify_waypoint_fk(r,&next,&sampled_pose);
                    if(s==ROBOT_STATUS_OK) {
                        double dt=(double)TASK_PERIOD_TRAJECTORY_MS/1000.0;
                        uint32_t j;
                        /* Rate discontinuity guard, joint 0.01deg -> deg/s.
                         * Conservative rejection, not online re-timing. */
                        for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
                            double v=fabs((double)next.value[j]-
                                 (double)last.value[j])*
                                 (double)ROBOT_JOINT_ANGLE_UNIT_DEG/dt;
                            double allowance=(double)ROBOT_JOINT_ANGLE_UNIT_DEG/dt;
                            if(v>(double)r->config.max_velocity_deg_s.value[j]+
                                 allowance) {
                                s=ROBOT_STATUS_ERROR_OUT_OF_RANGE;
                                break;
                            }
                        }
                        if(s==ROBOT_STATUS_OK) {
                            last=next;
                            if(cartesian_index>=request.cartesian.interval_count)
                                moving=0U;
                            else cartesian_index++;
                        }
                    }
                }
                if(s!=ROBOT_STATUS_OK) {
                    cancel_motion(r,s);
                    armed=0U;
                    moving=0U;
                    break;
                }
            }
            reference.target=last;
            reference.sequence=++sequence;
            reference.created_tick=xTaskGetTickCount();
            reference.enabled=1U;
            if(xQueueOverwrite(r->reference_queue,&reference)!=pdPASS)
                motion_record_fault(ROBOT_STATUS_ERROR_INTERNAL,0U);
            else {
                taskENTER_CRITICAL();
                r->diagnostics.trajectory_samples++;
                taskEXIT_CRITICAL();
            }
        } while(0);
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
        stage4_timing_end(STAGE4_TIMING_TRAJECTORY,timing_start);
#endif
    }
}

BaseType_t trajectory_task_create(void)
{
    return xTaskCreate(task_trajectory,"Trajectory",
        TASK_STACK_DEPTH_TRAJECTORY,&g_motion_runtime,
        TASK_PRIORITY_TRAJECTORY,&g_motion_runtime.trajectory_handle);
}
