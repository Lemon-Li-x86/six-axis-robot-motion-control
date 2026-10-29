#include <float.h>
#include <math.h>
#include <stddef.h>
#include "stage5_apf_joint_path.h"
#include "kinematics.h"
#include "ik_solution_selector.h"
#include "motion_limits.h"
#include "angle_utils.h"

static robot_status_t convert_pass(
    const robot_joint_positions_t *start_joints,
    const robot_apf_point_t *cartesian_points,
    size_t point_count,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    robot_real_t max_fk_position_error_mm,
    robot_real_t max_fk_rotation_element_error,
    stage5_apf_joint_segment_t *segments)
{
    robot_joint_angles_t initial_canonical={{0}};
    robot_transform_t constant_tool_orientation={{{0}}};
    robot_joint_positions_t previous=*start_joints;
    size_t i;
    robot_status_t status;
    status=robot_angles_canonicalize(start_joints,&initial_canonical);
    if(status!=ROBOT_STATUS_OK) return status;
    status=kinematics_forward(&initial_canonical,&constant_tool_orientation);
    if(status!=ROBOT_STATUS_OK) return status;
    for(i=0U;i<point_count;++i) {
        robot_transform_t requested=constant_tool_orientation;
        robot_transform_t actual={{{0}}};
        robot_joint_angles_t reference={{0}},selected_canonical={{0}};
        kinematics_ik_solutions_t solutions={0};
        ik_selection_result_t selection={0};
        trajectory_trapezoid_plan_t timing={0};
        robot_joint_positions_t next={0};
        size_t axis;
        for(axis=0U;axis<3U;++axis) {
            const double p=cartesian_points[i].mm[axis];
            if(!isfinite(p) || p>(double)FLT_MAX || p<-(double)FLT_MAX)
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            requested.matrix[axis][3]=(robot_real_t)p;
        }
        status=robot_angles_canonicalize(&previous,&reference);
        if(status!=ROBOT_STATUS_OK) return status;
        status=kinematics_inverse_with_reference(
            &requested,&reference,&solutions);
        if(status!=ROBOT_STATUS_OK) return status;
        status=ik_solution_selector_select(&solutions,&previous,&selection);
        if(status!=ROBOT_STATUS_OK) return status;
        next=selection.target_positions;
        status=motion_limits_check_positions(&next);
        if(status!=ROBOT_STATUS_OK) return status;
        status=robot_angles_canonicalize(&next,&selected_canonical);
        if(status!=ROBOT_STATUS_OK) return status;
        status=kinematics_forward(&selected_canonical,&actual);
        if(status!=ROBOT_STATUS_OK) return status;
        status=kinematics_pose_within_tolerance(
            &actual,&requested,max_fk_position_error_mm,
            max_fk_rotation_element_error);
        if(status!=ROBOT_STATUS_OK) return status;
        /* Rest-to-rest for each segment is safe under model-provided joint
         * limits and caller-provided velocity/acceleration constraints.
         * Not a constant-period direct setpoint jump. */
        status=trajectory_trapezoid_init(&timing,&previous,&next,
            max_velocity_deg_s,max_acceleration_deg_s2);
        if(status!=ROBOT_STATUS_OK) return status;
        if(segments!=NULL) {
            segments[i].target=next;
            segments[i].segment_duration_s=timing.total_time_s;
        }
        previous=next;
    }
    return ROBOT_STATUS_OK;
}

robot_status_t stage5_apf_points_to_joint_route(
    const robot_joint_positions_t *start_joints,
    const robot_apf_point_t *cartesian_points,
    size_t point_count,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    robot_real_t max_fk_position_error_mm,
    robot_real_t max_fk_rotation_element_error,
    stage5_apf_joint_segment_t *segments,
    size_t capacity,
    size_t *out_count)
{
    robot_status_t status;
    if(out_count==NULL || start_joints==NULL || segments==NULL ||
       max_velocity_deg_s==NULL || max_acceleration_deg_s2==NULL ||
       (point_count>0U && cartesian_points==NULL))
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    *out_count=0U;
    if(point_count>capacity ||
       !(max_fk_position_error_mm>0.0F) ||
       !(max_fk_rotation_element_error>0.0F) ||
       !isfinite((double)max_fk_position_error_mm) ||
       !isfinite((double)max_fk_rotation_element_error) ||
       max_fk_position_error_mm>KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM ||
       max_fk_rotation_element_error>
           KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    status=motion_limits_check_positions(start_joints);
    if(status!=ROBOT_STATUS_OK) return status;
    /* Pass one: validate all IK/rate segments WITHOUT touching output. */
    status=convert_pass(start_joints,cartesian_points,point_count,
        max_velocity_deg_s,max_acceleration_deg_s2,
        max_fk_position_error_mm,max_fk_rotation_element_error,NULL);
    if(status!=ROBOT_STATUS_OK) return status;
    /* Pass two: deterministic recomputation with immutable input contract. */
    status=convert_pass(start_joints,cartesian_points,point_count,
        max_velocity_deg_s,max_acceleration_deg_s2,
        max_fk_position_error_mm,max_fk_rotation_element_error,segments);
    if(status!=ROBOT_STATUS_OK) return ROBOT_STATUS_ERROR_INTERNAL;
    *out_count=point_count;
    return ROBOT_STATUS_OK;
}
