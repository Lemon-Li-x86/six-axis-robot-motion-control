/*
 * Native end-to-end Cartesian point conversion against production
 * Classic UR5 FK / IK and the same motion selector as QEMU.
 */
#include <stdio.h>
#include "cartesian_trajectory.h"
#include "kinematics.h"
#include "angle_utils.h"
static int failures=0;
#define TEST(v) do {if(!(v)){++failures;fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#v);}}while(0)
int main(void)
{
    robot_joint_angles_t q={{3000,-4500,6000,2000,500,-1000}};
    robot_joint_positions_t current={{3000,-4500,6000,2000,500,-1000}};
    robot_joint_positions_t chosen={{0}};
    robot_joint_angles_t canonical={{0}};
    robot_transform_t initial={{{0}}},target={{{0}}},again={{{0}}};
    cartesian_trajectory_plan_t plan={0};
    robot_status_t result;
    unsigned j;
    TEST(kinematics_forward(&q,&initial)==ROBOT_STATUS_OK);
    TEST(cartesian_line_init(&plan,&initial,&initial,0.1,0.05)
         ==ROBOT_STATUS_OK);
    result=cartesian_trajectory_joint_waypoint(&plan,0U,&current,
                                               &chosen,&target);
    TEST(result==ROBOT_STATUS_OK);
    if(result==ROBOT_STATUS_OK) {
        TEST(robot_angles_canonicalize(&chosen,&canonical)==ROBOT_STATUS_OK);
        TEST(kinematics_forward(&canonical,&again)==ROBOT_STATUS_OK);
        for(j=0U;j<3U;j++) {
            double dx=(double)again.matrix[j][3]-(double)target.matrix[j][3];
            TEST(dx*dx<0.25);
        }
    }
    printf("Cartesian FK IK selector integration: failures %d\n",failures);
    return failures==0?0:1;
}
