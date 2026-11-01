#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stage5_apf_joint_path.h"
#include "kinematics.h"
#include "angle_utils.h"
#include "motion_limits.h"
#include "stage5_apf_proposal.h"

static unsigned tests,failed;
#define CHECK(x) do { ++tests; if(!(x)) { ++failed; \
    fprintf(stderr,"FAIL line %u: %s\\n",(unsigned)__LINE__,#x); } } while(0)

static const robot_joint_positions_t initial={{9000,-6000,8000,3500,5500,-12000}};
static const robot_joint_velocities_t velocities={{15,15,15,15,15,15}};
static const trajectory_joint_accelerations_t accelerations={{30,30,30,30,30,30}};

static robot_status_t start_pose(robot_transform_t *p)
{
    robot_joint_angles_t angles={{0}};
    robot_status_t s=robot_angles_canonicalize(&initial,&angles);
    if(s!=ROBOT_STATUS_OK) return s;
    return kinematics_forward(&angles,p);
}

static void real_apf_to_production_ik(void)
{
    robot_transform_t fk={{{0}}};
    robot_apf_point_t start,goal,cartesian[16];
    robot_apf_config_t cfg={1.0,0.0,30.0,1.0,1.5,0.1};
    stage5_apf_joint_segment_t segments[16]={0};
    size_t c=0,n=0,i;
    CHECK(start_pose(&fk)==ROBOT_STATUS_OK);
    for(i=0;i<3;i++)start.mm[i]=fk.matrix[i][3];
    goal=start;
    goal.mm[0]+=3.0;
    goal.mm[1]+=1.5;
    CHECK(robot_apf_plan_path(&start,&goal,NULL,0U,&cfg,
        cartesian,16U,&c)==ROBOT_STATUS_OK);
    CHECK(c>=2U && c<16U);
    CHECK(stage5_apf_points_to_joint_route(&initial,cartesian,c,
        &velocities,&accelerations,KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL,
        segments,16U,&n)==ROBOT_STATUS_OK);
    CHECK(n==c);
    for(i=0U;i<n;i++) {
        robot_joint_angles_t canonical={{0}};
        robot_transform_t reconstructed={{{0}}},requested=fk;
        size_t axis;
        CHECK(motion_limits_check_positions(&segments[i].target)==ROBOT_STATUS_OK);
        CHECK(isfinite(segments[i].segment_duration_s));
        CHECK(segments[i].segment_duration_s>=0.0);
        CHECK(robot_angles_canonicalize(&segments[i].target,&canonical)==ROBOT_STATUS_OK);
        CHECK(kinematics_forward(&canonical,&reconstructed)==ROBOT_STATUS_OK);
        for(axis=0;axis<3;++axis)
            requested.matrix[axis][3]=(robot_real_t)cartesian[i].mm[axis];
        CHECK(kinematics_pose_within_tolerance(
            &reconstructed,&requested,
            KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
            KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL)==ROBOT_STATUS_OK);
    }
}

static void fail_closed_no_partial_joint_path(void)
{
    robot_transform_t fk={{{0}}};
    robot_apf_point_t p[2]={{{0}}},unused={{0}};
    stage5_apf_joint_segment_t out[2]={0};
    size_t n=123U,i,j;
    CHECK(start_pose(&fk)==ROBOT_STATUS_OK);
    for(i=0;i<3;++i)p[0].mm[i]=fk.matrix[i][3];
    p[1]=p[0];p[1].mm[0]=5000.0;
    for(i=0;i<2;++i) {
        out[i].segment_duration_s=-777.0;
        for(j=0;j<ROBOT_JOINT_COUNT;j++)out[i].target.value[j]=-777777;
    }
    CHECK(stage5_apf_points_to_joint_route(&initial,p,2,&velocities,
        &accelerations,KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL,out,2,&n)
        !=ROBOT_STATUS_OK);
    CHECK(n==0U);
    for(i=0;i<2;++i) {
        CHECK(out[i].segment_duration_s==-777.0);
        for(j=0;j<ROBOT_JOINT_COUNT;j++)
            CHECK(out[i].target.value[j]==-777777);
    }
    n=123U;
    CHECK(stage5_apf_points_to_joint_route(&initial,NULL,0,&velocities,
        &accelerations,KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL,out,2,&n)
        ==ROBOT_STATUS_OK);
    CHECK(n==0U && out[0].segment_duration_s==-777.0);
    n=123U;
    CHECK(stage5_apf_points_to_joint_route(&initial,&unused,1,&velocities,
        &accelerations,KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL,out,0,&n)
        ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(n==0U && out[0].segment_duration_s==-777.0);
    n=123U;
    CHECK(stage5_apf_points_to_joint_route(&initial,&unused,1,&velocities,
        &accelerations,KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM+1.0F,
        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL,out,2,&n)
        ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(n==0U && out[0].segment_duration_s==-777.0);
}

static void authoritative_pose_limits(void)
{
    robot_real_t position=0.0F,rotation=0.0F;
    CHECK(stage5_apf_joint_pose_limits(&position,&rotation)==ROBOT_STATUS_OK);
    CHECK(position==KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM);
    CHECK(rotation==KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL);
    CHECK(stage5_apf_joint_pose_limits(NULL,&rotation)
          ==ROBOT_STATUS_ERROR_NULL_POINTER);
    CHECK(stage5_apf_joint_pose_limits(&position,NULL)
          ==ROBOT_STATUS_ERROR_NULL_POINTER);
}

static void embedded_rtos_apf_single_step_proposal(void)
{
    /* Route via production C APF -> production C IK -> trapezoid.
     * Start pose and limits are those used by the existing C IK test.
     * The resulting proposal MUST NOT trigger any motor action.
     */
    protocol_apf_step_request_t request={0};
    motion_stage4_config_t config={0};
    stage5_apf_proposal_t result={0};
    robot_joint_angles_t initial_canonical={{0}};
    robot_transform_t fk={{{0}}};
    size_t axis;
    request.sequence=37U;
    request.goal_offset_mm[0]=3;
    request.goal_offset_mm[1]=2;
    for(axis=0U;axis<3U;axis++) {
        request.obstacle_min_offset_mm[axis]=100;
        request.obstacle_max_offset_mm[axis]=120;
    }
    request.apf_parameters[0]=1.0F;
    request.apf_parameters[1]=0.0F;
    request.apf_parameters[2]=30.0F;
    request.apf_parameters[3]=1.0F;
    request.apf_parameters[4]=1.5F;
    request.apf_parameters[5]=0.1F;
    config.max_velocity_deg_s=velocities;
    config.max_acceleration_deg_s2=accelerations;
    config.cartesian_waypoint_position_tolerance_mm=
        KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM;
    config.cartesian_waypoint_rotation_matrix_tolerance=
        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL;

    CHECK(robot_angles_canonicalize(&initial,&initial_canonical)==ROBOT_STATUS_OK);
    CHECK(kinematics_forward(&initial_canonical,&fk)==ROBOT_STATUS_OK);
    CHECK(stage5_apf_propose_from_feedback(
        &initial,&request,&config,&result)==ROBOT_STATUS_OK);
    CHECK(result.goal_reached==0U);
    CHECK(isfinite(result.cartesian_mm.mm[0]));
    CHECK(result.cartesian_mm.mm[0] > (double)fk.matrix[0][3]);
    CHECK(result.cartesian_mm.mm[0] < (double)fk.matrix[0][3]+3.0);
    {
        robot_joint_angles_t chosen=result.canonical_joint;
        robot_transform_t actual={{{0}}}, desired=fk;
        CHECK(kinematics_forward(&chosen,&actual)==ROBOT_STATUS_OK);
        for(axis=0U;axis<3U;axis++)
            desired.matrix[axis][3]=(robot_real_t)result.cartesian_mm.mm[axis];
        CHECK(kinematics_pose_within_tolerance(
            &actual,&desired,
            KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
            KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL)==ROBOT_STATUS_OK);
    }

    /* A goal already reached is non-actuating: same canonical angles. */
    request.goal_offset_mm[0]=0;
    request.goal_offset_mm[1]=0;
    CHECK(stage5_apf_propose_from_feedback(
        &initial,&request,&config,&result)==ROBOT_STATUS_OK);
    CHECK(result.goal_reached==1U);
    for(axis=0U;axis<ROBOT_JOINT_COUNT;axis++)
        CHECK(result.canonical_joint.value[axis]==initial_canonical.value[axis]);

    /* Malformed obstacle and nonfinite gains must never modify the output. */
    {
        stage5_apf_proposal_t preserved=result;
        request.obstacle_min_offset_mm[0]=-1;
        request.obstacle_max_offset_mm[0]=1;
        CHECK(stage5_apf_propose_from_feedback(
            &initial,&request,&config,&result)!=ROBOT_STATUS_OK);
        CHECK(memcmp(&preserved,&result,sizeof(result))==0);
        request.obstacle_min_offset_mm[0]=100;
        request.obstacle_max_offset_mm[0]=120;
        request.apf_parameters[0]=NAN;
        CHECK(stage5_apf_propose_from_feedback(
            &initial,&request,&config,&result)!=ROBOT_STATUS_OK);
        CHECK(memcmp(&preserved,&result,sizeof(result))==0);
    }
}

int main(void)
{
    authoritative_pose_limits();
    embedded_rtos_apf_single_step_proposal();
    real_apf_to_production_ik();
    fail_closed_no_partial_joint_path();
    printf("Stage5 APF->C IK->trapezoid: %u checks, %u failures\\n",tests,failed);
    return failed!=0U;
}
