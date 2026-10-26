/*
 * Stage 4 application motion request/diagnostics service.
 * Motion algorithms are independent of this RTOS-facing boundary.
 */
#ifndef ROBOT_MOTION_PIPELINE_H
#define ROBOT_MOTION_PIPELINE_H
#include <stdint.h>
#include "trajectory.h"
#include "cartesian_trajectory.h"
#include "pid.h"
#include "robot_types.h"
#include "error_code.h"

typedef struct {
    pid_config_t pid[ROBOT_JOINT_COUNT];
    robot_joint_velocities_t max_velocity_deg_s;
    trajectory_joint_accelerations_t max_acceleration_deg_s2;
    uint32_t feedback_timeout_ms;
} motion_stage4_config_t;

typedef struct {
    uint32_t requests_accepted;
    uint32_t trajectory_samples;
    uint32_t controller_steps;
    uint32_t feedback_stale_count;
    uint32_t planner_faults;
    uint32_t controller_faults;
    uint32_t monitoring_ticks;
    uint32_t control_stack_free_words;
    uint32_t trajectory_stack_free_words;
    uint32_t monitor_stack_free_words;
    robot_status_t last_fault;
} motion_stage4_diagnostics_t;

/* Must be called once before vTaskStartScheduler, AFTER motor_state_init.
 * Called only when ROBOT_ENABLE_STAGE4_TASKS is enabled. No automatic move.
 */
robot_status_t motion_pipeline_start(const motion_stage4_config_t *cfg);

/* No implicit move: an application caller must explicitly submit request.
 * If a second request arrives before execution, it replaces the first.
 */
robot_status_t motion_pipeline_submit_joint(
    const robot_joint_positions_t *continuous_goal);
robot_status_t motion_pipeline_submit_cartesian(
    const cartesian_trajectory_plan_t *plan);
robot_status_t motion_pipeline_get_diagnostics(motion_stage4_diagnostics_t *out);

#endif
