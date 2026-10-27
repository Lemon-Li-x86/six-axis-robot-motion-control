#include <math.h>
#include <stdio.h>
#include <string.h>
#include "cartesian_trajectory.h"
#include "joint_controller.h"
static unsigned failed=0U,checks=0U;
#define ASSERT(x) do {++checks;if(!(x)){++failed;fprintf(stderr,"FAIL %s:%d %s\n",__FILE__,__LINE__,#x);}}while(0)
static robot_transform_t pose(double x,double y,double z)
{
    robot_transform_t p={{{0}}};
    p.matrix[0][0]=1.0F;p.matrix[1][1]=1.0F;
    p.matrix[2][2]=1.0F;p.matrix[3][3]=1.0F;
    p.matrix[0][3]=(robot_real_t)x;
    p.matrix[1][3]=(robot_real_t)y;
    p.matrix[2][3]=(robot_real_t)z;
    return p;
}
static void test_line(void)
{
    robot_transform_t a=pose(0,0,0),b=pose(100,40,0),q;
    cartesian_trajectory_plan_t plan={0};
    ASSERT(cartesian_line_init(&plan,&a,&b,1.0,0.2)==ROBOT_STATUS_OK);
    ASSERT(plan.interval_count==5U);
    ASSERT(cartesian_trajectory_sample(&plan,2U,&q)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)q.matrix[0][3]-40.0)<0.0001);
    ASSERT(fabs((double)q.matrix[1][3]-16.0)<0.0001);
    ASSERT(cartesian_trajectory_sample(&plan,5U,&q)==ROBOT_STATUS_OK);
    ASSERT(q.matrix[0][3]==100.0F);
    ASSERT(cartesian_trajectory_sample(&plan,6U,&q)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    ASSERT(cartesian_line_init(&plan,&a,&b,-1,0.2)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
}
static void test_arc(void)
{
    robot_transform_t a=pose(100,0,0),mid=pose(0,100,0);
    robot_transform_t b=pose(-100,0,0),q={{{0}}};
    robot_transform_t collinear=pose(50,0,0);
    cartesian_trajectory_plan_t p={0};
    ASSERT(cartesian_arc_init(&p,&a,&mid,&b,2.0,0.5)==ROBOT_STATUS_OK);
    ASSERT(p.interval_count==4U);
    ASSERT(cartesian_trajectory_sample(&p,2U,&q)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)q.matrix[0][3])<0.001);
    ASSERT(fabs((double)q.matrix[1][3]-100.0)<0.001);
    ASSERT(cartesian_trajectory_sample(&p,4U,&q)==ROBOT_STATUS_OK);
    ASSERT(q.matrix[0][3]==-100.0F);
    ASSERT(cartesian_arc_init(&p,&a,&collinear,&b,2,0.5)
           ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
}
static void test_so3(void)
{
    robot_transform_t a=pose(0,0,0),b=pose(0,0,0),m={{{0}}};
    cartesian_trajectory_plan_t p={0};
    b.matrix[0][0]=-1.0F;b.matrix[1][1]=-1.0F;
    ASSERT(cartesian_line_init(&p,&a,&b,2,1)==ROBOT_STATUS_OK);
    ASSERT(cartesian_trajectory_sample(&p,1,&m)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)m.matrix[0][0])<0.001);
    ASSERT(fabs(fabs((double)m.matrix[1][0])-1.0)<0.001);
    b.matrix[0][0]=2.0F;
    ASSERT(cartesian_line_init(&p,&a,&b,2,1)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
}
static void test_pid(void)
{
    pid_config_t cfg={1.0F,0.0F,0.0F,10.0F,3.0F,0.0F};
    pid_controller_t pid={0};
    robot_real_t correction=-999.0F;
    ASSERT(pid_init(&pid,&cfg)==ROBOT_STATUS_OK);
    ASSERT(pid_step(&pid,10.0,0.0,0.01,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs(correction-3.0F)<1e-5);
    ASSERT(pid_step(&pid,0.0,0.0,0.01,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs(correction+3.0F)<1e-5);
    pid_reset(&pid);
    cfg.kp=0.0F;cfg.ki=2.0F;cfg.integral_limit_deg_s=0.2F;
    ASSERT(pid_init(&pid,&cfg)==ROBOT_STATUS_OK);
    ASSERT(pid_step(&pid,10.0,0.0,0.1,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs(correction-0.4F)<1e-4);
    ASSERT(fabs(pid.integral_deg_s-0.2)<1e-8);
    cfg.kp=-1.0F;
    ASSERT(pid_init(&pid,&cfg)==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
}
/* Stage4 PDF explicitly requires independently configurable D gain.
 * A derivative-only controller must NEVER create a spurious impulse
 * merely because a nonzero error remains constant for two samples. */
static void test_pid_derivative_startup(void)
{
    pid_config_t cfg={0.0F,0.0F,0.5F,10.0F,100.0F,0.0F};
    pid_controller_t pid={0};
    robot_real_t correction=-999.0F;
    ASSERT(pid_init(&pid,&cfg)==ROBOT_STATUS_OK);
    ASSERT(pid_step(&pid,1.0,0.0,0.1,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)correction)<1e-6);
    /* Same error: derivative remains zero after initialization. */
    ASSERT(pid_step(&pid,1.0,0.0,0.1,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)correction)<1e-6);
    /* A step down in error should change output, then settle at zero. */
    ASSERT(pid_step(&pid,0.5,0.0,0.1,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)correction+2.5)<1e-5);
    ASSERT(pid_step(&pid,0.5,0.0,0.1,&correction)==ROBOT_STATUS_OK);
    ASSERT(fabs((double)correction)<1e-5);
}

static void test_joint_wrapper(void)
{
    pid_config_t gains[ROBOT_JOINT_COUNT]={{0}};
    joint_controller_t c;
    joint_control_corrections_t corrections;
    robot_joint_positions_t target={{100,200,0,0,0,0}};
    robot_joint_positions_t feedback={{0}};
    unsigned j;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
        gains[j].kp=1.0F;
        gains[j].output_limit_deg=10.0F;
    }
    ASSERT(joint_controller_init(&c,gains)==ROBOT_STATUS_OK);
    ASSERT(joint_controller_step(&c,&target,&feedback,0.02,&corrections)
           ==ROBOT_STATUS_OK);
    ASSERT(fabs((double)corrections.value[0]-1.0)<1e-5);
    ASSERT(fabs((double)corrections.value[1]-2.0)<1e-5);
}
int main(void)
{
    test_line();test_arc();test_so3();test_pid();test_pid_derivative_startup();test_joint_wrapper();
    printf("Cartesian and PID checks: %u failures / %u\n",failed,checks);
    return failed==0U?0:1;
}
