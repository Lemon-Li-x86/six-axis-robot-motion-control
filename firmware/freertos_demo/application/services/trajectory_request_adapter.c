/*
 * Stage 4 request adapter. A narrow, testable application boundary.
 */
#include <stddef.h>

#include "trajectory_request_adapter.h"
#include "motion_limits.h"

robot_status_t trajectory_request_from_ik(
    const robot_joint_positions_t *current_positions,
    const ik_selection_result_t *selection,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    trajectory_trapezoid_plan_t *out)
{
    robot_status_t status;

    if (current_positions == NULL || selection == NULL ||
        max_velocity_deg_s == NULL ||
        max_acceleration_deg_s2 == NULL || out == NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;

    status = motion_limits_check_positions(current_positions);
    if (status != ROBOT_STATUS_OK)
        return status;

    status = motion_limits_check_positions(&selection->target_positions);
    if (status != ROBOT_STATUS_OK)
        return status;

    return trajectory_trapezoid_init(out,
        current_positions, &selection->target_positions,
        max_velocity_deg_s, max_acceleration_deg_s2);
}
