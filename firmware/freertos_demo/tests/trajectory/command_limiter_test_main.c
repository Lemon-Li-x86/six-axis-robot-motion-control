/*
 * Pure Stage4 joint command gate tests.
 * Acceleration cap uses unquantized internal velocity; the integer
 * driver boundary is deliberately checked independently.
 */
#include <math.h>
#include <stdio.h>
#include "joint_command_limiter.h"

static unsigned checks=0U,failed=0U;
#define CHECK(x) do { ++checks; if(!(x)) { ++failed; \
    fprintf(stderr,"command limiter line %d: %s\n",__LINE__,#x); \
} } while(0)

int main(void)
{
    joint_command_limiter_t state={0};
    robot_joint_positions_t feedback={{0}};
    robot_joint_positions_t output={{0}};
    joint_control_corrections_t correction={{0}};
    robot_joint_velocities_t vmax={{15,15,15,15,15,15}};
    trajectory_joint_accelerations_t amax={{30,30,30,30,30,30}};
    const robot_model_t *model=robot_model_get_active();
    double previous_velocity=0.0;
    unsigned i;

    correction.value[0]=10.0F;
    CHECK(model!=NULL);
    for(i=0U;i<100U;i++)
    {
        double dv;
        robot_status_t s=joint_command_limiter_step(
            &state,&feedback,&correction,&vmax,&amax,model,0.01,&output);
        CHECK(s==ROBOT_STATUS_OK);
        CHECK(state.initialized==1U);
        CHECK(output.value[0]>=model->joint_min_positions.value[0]);
        CHECK(output.value[0]<=model->joint_max_positions.value[0]);
        CHECK(fabs(state.velocity_deg_s[0])<=15.0+1e-7);
        dv=fabs(state.velocity_deg_s[0]-previous_velocity);
        CHECK(dv<=30.0*0.01+1e-7);
        previous_velocity=state.velocity_deg_s[0];
        if(i==0U) {
            CHECK(state.command_raw[0]>0.0);
            CHECK(output.value[0]==0);
        }
        if(i==1U) CHECK(output.value[0]>=1);
    }
    CHECK(output.value[0]>0);

    /* Atomic failure: invalid dt and config don't corrupt state/output. */
    {
        joint_command_limiter_t snapshot=state;
        robot_joint_positions_t previous=output;
        CHECK(joint_command_limiter_step(
            &state,&feedback,&correction,&vmax,&amax,model,0.0,&output)
            ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
        CHECK(state.command_raw[0]==snapshot.command_raw[0]);
        CHECK(output.value[0]==previous.value[0]);
        amax.value[0]=0.0F;
        CHECK(joint_command_limiter_step(
            &state,&feedback,&correction,&vmax,&amax,model,0.01,&output)
            ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
        CHECK(state.command_raw[0]==snapshot.command_raw[0]);
        CHECK(output.value[0]==previous.value[0]);
        CHECK(joint_command_limiter_step(
            NULL,&feedback,&correction,&vmax,&amax,model,0.01,&output)
            ==ROBOT_STATUS_ERROR_NULL_POINTER);
    }

    joint_command_limiter_reset(&state);
    CHECK(state.initialized==0U);
    CHECK(state.velocity_deg_s[0]==0.0);

    printf("Stage4 limiter: %u checks, %u failures\n",checks,failed);
    return failed==0U ? 0:1;
}
