/**
 * @file motion_pipeline.h
 * @brief 运动服务对外请求/配置接口。
 */
#ifndef ROBOT_MOTION_PIPELINE_H
#define ROBOT_MOTION_PIPELINE_H
#include <stdint.h>
#include "trajectory.h"
#include "cartesian_trajectory.h"
#include "pid.h"
#include "robot_types.h"
#include "error_code.h"
#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
#include "protocol.h"
#endif

typedef struct {
    pid_config_t pid[ROBOT_JOINT_COUNT];
    robot_joint_velocities_t max_velocity_deg_s;
    trajectory_joint_accelerations_t max_acceleration_deg_s2;
    uint32_t feedback_timeout_ms;
    uint32_t reference_timeout_ms;
    robot_real_t cartesian_start_tolerance_mm;
    robot_real_t cartesian_start_rotation_matrix_tolerance;
    robot_real_t cartesian_waypoint_position_tolerance_mm;
    robot_real_t cartesian_waypoint_rotation_matrix_tolerance;
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

#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
/* Read-only Stage5 QEMU telemetry. Never used by motion/safety decisions.
 * Joint units: raw centidegrees, PID correction: millidegrees,
 * limiter internal position: millicentidegrees. */
typedef struct {
    uint32_t rtos_tick;
    uint32_t control_dt_us;
    uint32_t reference_age_ticks;
    uint32_t request_generation;
    int32_t reference_raw[ROBOT_JOINT_COUNT];
    int32_t feedback_raw[ROBOT_JOINT_COUNT];
    int32_t pid_correction_millideg[ROBOT_JOINT_COUNT];
    int32_t limiter_position_millicentideg[ROBOT_JOINT_COUNT];
    int32_t command_raw[ROBOT_JOINT_COUNT];
    uint8_t valid;
} motion_stage5_control_trace_t;

/* Returns the last successful control-publication snapshot atomically. */
robot_status_t motion_pipeline_get_stage5_control_trace(
    motion_stage5_control_trace_t *out);
#endif

/* Must be called once before vTaskStartScheduler, AFTER motor_state_init.
 * Called only when ROBOT_ENABLE_STAGE4_TASKS is enabled. No automatic move.
 */
robot_status_t motion_pipeline_start(const motion_stage4_config_t *cfg);

/* No implicit move: an application caller must explicitly submit request.
 * If a second request arrives before execution, it replaces the first.
 */
robot_status_t motion_pipeline_submit_joint(
    const robot_joint_positions_t *continuous_goal);
robot_status_t motion_pipeline_submit_polynomial(
    const trajectory_polynomial_plan_t *plan);
robot_status_t motion_pipeline_submit_cartesian(
    const cartesian_trajectory_plan_t *plan);
#if ROBOT_ENABLE_STAGE5_QEMU_UART_INTEGRATION
/* Simulation: asynchronously compute C APF/IK in trajectory task.
 * The resulting UART message is a proposal, never a motor command.
 * External swept-link check must precede separate joint execution.
 */
robot_status_t motion_pipeline_submit_apf_proposal(
    const protocol_apf_step_request_t *request);
#endif
robot_status_t motion_pipeline_get_diagnostics(motion_stage4_diagnostics_t *out);

#endif
