#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "stage5_apf_joint_path.h"
#include "kinematics.h"
#include "angle_utils.h"
#include "motion_limits.h"

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

int main(void)
{
    real_apf_to_production_ik();
    fail_closed_no_partial_joint_path();
    printf("Stage5 APF->C IK->trapezoid: %u checks, %u failures\\n",tests,failed);
    return failed!=0U;
}
