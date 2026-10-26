/* Standalone ARM Cortex-M4 / QEMU Stage4 smoke regression.
 * GDB: break qemu_stage4_test_done; p g_stage4_test_report.
 * Not a runtime PID tuning or FreeRTOS scheduling benchmark.
 */
#include <stdint.h>
#include <math.h>
#include "trajectory.h"
#include "cartesian_trajectory.h"
#include "pid.h"

typedef struct {
    uint32_t checks;
    uint32_t passed;
    uint32_t failed;
    int32_t last_failure_line;
} stage4_test_report_t;
volatile stage4_test_report_t g_stage4_test_report={0};
volatile uint32_t g_stage4_test_finished=0U;

#define EXPECT(cond) do {g_stage4_test_report.checks++;     if(cond)g_stage4_test_report.passed++;     else {g_stage4_test_report.failed++;         g_stage4_test_report.last_failure_line=__LINE__;}     } while(0)

__attribute__((noinline))
void qemu_stage4_test_done(void)
{
    for(;;) __asm volatile ("nop");
}

int main(void)
{
    robot_joint_positions_t start={{0}},goal={{10000,19000,0,0,0,0}};
    robot_joint_velocities_t zero={{0}};
    robot_joint_velocities_t vmax={{50,100,50,50,50,50}};
    trajectory_joint_accelerations_t amax={{100,200,100,100,100,100}};
    trajectory_trapezoid_plan_t trap={0};
    trajectory_polynomial_plan_t poly={0};
    trajectory_joint_sample_t sampled={0};
    robot_transform_t a={{{0}}},b={{{0}}},pose={{{0}}};
    cartesian_trajectory_plan_t cart={0};
    pid_controller_t pid={0};
    pid_config_t gains={1.0F,0.0F,0.0F,1.0F,3.0F,0.0F};
    robot_real_t correction=0.0F;

    EXPECT(trajectory_polynomial_init_cubic(
        &poly,&start,&goal,&zero,&zero,2.0)==ROBOT_STATUS_OK);
    EXPECT(trajectory_polynomial_sample(&poly,1.0,&sampled)==ROBOT_STATUS_OK);
    EXPECT(sampled.positions.value[0]==5000);
    EXPECT(sampled.positions.value[1]==9500);

    EXPECT(trajectory_trapezoid_init(
        &trap,&start,&goal,&vmax,&amax)==ROBOT_STATUS_OK);
    EXPECT(trap.total_time_s>0.0);
    EXPECT(trajectory_trapezoid_sample(&trap,trap.total_time_s,
                                     &sampled)==ROBOT_STATUS_OK);
    EXPECT(sampled.positions.value[0]==10000);

    a.matrix[0][0]=a.matrix[1][1]=a.matrix[2][2]=a.matrix[3][3]=1.0F;
    b=a;b.matrix[0][3]=100.0F;
    EXPECT(cartesian_line_init(&cart,&a,&b,2.0,1.0)==ROBOT_STATUS_OK);
    EXPECT(cartesian_trajectory_sample(&cart,1U,&pose)==ROBOT_STATUS_OK);
    EXPECT(fabsf(pose.matrix[0][3]-50.0F)<0.001F);

    EXPECT(pid_init(&pid,&gains)==ROBOT_STATUS_OK);
    EXPECT(pid_step(&pid,10.0,0.0,0.01,&correction)==ROBOT_STATUS_OK);
    EXPECT(fabsf(correction-3.0F)<0.001F);

    g_stage4_test_finished=1U;
    qemu_stage4_test_done();
    return 0;
}
