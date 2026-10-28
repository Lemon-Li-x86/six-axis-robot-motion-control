#include <stddef.h>
#include "cartesian_trajectory.h"
#include "kinematics.h"
#include "ik_solution_selector.h"
#include "motion_limits.h"
#include "angle_utils.h"

/* App supplies current joint continuity; no static pose, no caching. */
robot_status_t cartesian_trajectory_joint_waypoint(
    const cartesian_trajectory_plan_t *plan,
    uint32_t sample_index,
    const robot_joint_positions_t *current,
    robot_joint_positions_t *selected,
    robot_transform_t *sampled_pose)
{
    robot_transform_t pose;
    kinematics_ik_solutions_t solutions = {0};
    robot_joint_angles_t reference = {{0}};
    ik_selection_result_t selection = {0};
    robot_status_t status;

    if(plan==NULL || current==NULL || selected==NULL ||
       sampled_pose==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    status=motion_limits_check_positions(current);
    if(status!=ROBOT_STATUS_OK) return status;
    status=cartesian_trajectory_sample(plan,sample_index,&pose);
    if(status!=ROBOT_STATUS_OK) return status;
    status=robot_angles_canonicalize(current,&reference);
    if(status!=ROBOT_STATUS_OK) return status;
    status=kinematics_inverse_with_reference(&pose,&reference,&solutions);
    if(status!=ROBOT_STATUS_OK) return status;
    status=ik_solution_selector_select(&solutions,current,&selection);
    if(status!=ROBOT_STATUS_OK) return status;
    *selected=selection.target_positions;
    *sampled_pose=pose;
    return ROBOT_STATUS_OK;
}
