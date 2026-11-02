/**
 * @file stage5_apf_proposal.c
 * @brief Compute an APF waypoint and select a feasible joint target.
 *
 * Planning uses the measured pose and returns a proposal only.
 * Swept-link collision validation and motion execution are separate steps.
 */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include "stage5_apf_proposal.h"
#include "stage5_apf_joint_path.h"
#include "angle_utils.h"
#include "kinematics.h"

robot_status_t stage5_apf_propose_from_feedback(
    const robot_joint_positions_t *measured,
    const protocol_apf_step_request_t *request,
    const motion_stage4_config_t *limits,
    stage5_apf_proposal_t *out)
{
    robot_joint_angles_t canonical={{0}};
    robot_transform_t current_pose={{{0}}};
    robot_apf_point_t current={{0}}, goal={{0}}, next={{0}};
    robot_apf_aabb_t obstacle={0};
    robot_apf_config_t cfg={0};
    robot_apf_step_info_t info={0};
    stage5_apf_joint_segment_t segment={0};
    stage5_apf_proposal_t proposed={0};
    robot_status_t status;
    size_t count=0U,axis;
    if(measured==NULL || request==NULL || limits==NULL || out==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    status=robot_angles_canonicalize(measured,&canonical);
    if(status!=ROBOT_STATUS_OK) return status;
    status=kinematics_forward(&canonical,&current_pose);
    if(status!=ROBOT_STATUS_OK) return status;

    cfg.attraction_gain=request->apf_parameters[0];
    cfg.repulsion_gain=request->apf_parameters[1];
    cfg.influence_distance_mm=request->apf_parameters[2];
    cfg.clearance_mm=request->apf_parameters[3];
    cfg.maximum_step_mm=request->apf_parameters[4];
    cfg.goal_tolerance_mm=request->apf_parameters[5];
    for(axis=0U;axis<3U;axis++) {
        double pos=(double)current_pose.matrix[axis][3];
        current.mm[axis]=pos;
        goal.mm[axis]=pos+(double)request->goal_offset_mm[axis];
        obstacle.min_corner.mm[axis]=pos+
            (double)request->obstacle_min_offset_mm[axis];
        obstacle.max_corner.mm[axis]=pos+
            (double)request->obstacle_max_offset_mm[axis];
    }
    status=robot_apf_step(&current,&goal,&obstacle,1U,
                          &cfg,&next,&info);
    if(status!=ROBOT_STATUS_OK) return status;
    proposed.cartesian_mm=next;
    proposed.goal_reached=(uint8_t)(info.reached_goal!=0U);
    if(proposed.goal_reached) {
        proposed.canonical_joint=canonical;
    } else {
        status=stage5_apf_points_to_joint_route(
            measured,&next,1U,
            &limits->max_velocity_deg_s,
            &limits->max_acceleration_deg_s2,
            limits->cartesian_waypoint_position_tolerance_mm,
            limits->cartesian_waypoint_rotation_matrix_tolerance,
            &segment,1U,&count);
        if(status!=ROBOT_STATUS_OK) return status;
        if(count!=1U) return ROBOT_STATUS_ERROR_INTERNAL;
        status=robot_angles_canonicalize(&segment.target,
                                          &proposed.canonical_joint);
        if(status!=ROBOT_STATUS_OK) return status;
    }
    /* Validate the Cartesian result before publishing the proposal. */
    for(axis=0U;axis<3U;axis++) {
        if(!isfinite(proposed.cartesian_mm.mm[axis]) ||
           fabs(proposed.cartesian_mm.mm[axis])>(double)FLT_MAX)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }
    *out=proposed;
    return ROBOT_STATUS_OK;
}
