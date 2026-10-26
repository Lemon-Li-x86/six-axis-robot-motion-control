/*
 * Stage 4 application boundary: selected IK result -> joint trajectory.
 * The adapter consumes a COPY of current continuous joint positions.
 * A future Control/Trajectory Task must first obtain synchronized,
 * fresh feedback through motor_state_get_feedback().
 * No direct motor-driver, UART or FreeRTOS calls here.
 */
#ifndef TRAJECTORY_REQUEST_ADAPTER_H
#define TRAJECTORY_REQUEST_ADAPTER_H

#include "ik_solution_selector.h"
#include "trajectory.h"

/*
 * Check both endpoints against the one authoritative Robot Model via
 * motion_limits_check_positions(), then build synchronized rest-to-rest
 * trapezoidal joint-space trajectory. Returns errors without changing out.
 * Speed/accel limits are explicit caller-supplied configuration in deg/s
 * and deg/s^2, never duplicated here as UR5 constants.
 */
robot_status_t trajectory_request_from_ik(
    const robot_joint_positions_t *current_positions,
    const ik_selection_result_t *selection,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    trajectory_trapezoid_plan_t *out);

#endif
