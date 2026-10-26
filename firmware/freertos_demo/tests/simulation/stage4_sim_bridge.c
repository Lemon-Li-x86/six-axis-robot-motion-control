/*
 * Host-only C bridge for deterministic PyBullet dynamic trials.
 * The Python caller NEVER reimplements PID, trapezoid or command limits.
 * The same production firmware algorithm objects are linked into this DLL.
 *
 * The bridge is NOT compiled into the ARM firmware; malloc is host-only.
 * Every configuration value comes from stage4_motion_default_config().
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stage4_motion_config.h"
#include "trajectory.h"
#include "joint_controller.h"
#include "joint_command_limiter.h"
#include "motion_limits.h"
#include "robot_model.h"

typedef struct
{
    trajectory_trapezoid_plan_t path;
    joint_controller_t controller;
    joint_command_limiter_t limiter;
    motion_stage4_config_t config;
    double time_s;
} stage4_sim_t;

static void read_positions(
    const int32_t source[ROBOT_JOINT_COUNT],
    robot_joint_positions_t *dest)
{
    uint32_t i;
    for(i=0U;i<ROBOT_JOINT_COUNT;i++)
        dest->value[i]=source[i];
}

static void write_positions(
    const robot_joint_positions_t *source,
    int32_t dest[ROBOT_JOINT_COUNT])
{
    uint32_t i;
    for(i=0U;i<ROBOT_JOINT_COUNT;i++)
        dest[i]=source->value[i];
}

#if defined(_WIN32)
#define STAGE4_SIM_EXPORT __declspec(dllexport)
#else
#define STAGE4_SIM_EXPORT __attribute__((visibility("default")))
#endif

STAGE4_SIM_EXPORT void *stage4_sim_create_with_kp_scale(
    const int32_t start_raw[ROBOT_JOINT_COUNT],
    const int32_t goal_raw[ROBOT_JOINT_COUNT],
    double kp_scale)
{
    stage4_sim_t *sim;
    robot_joint_positions_t start={{0}},goal={{0}};
    robot_status_t status;
    uint32_t i;
    if(start_raw==NULL || goal_raw==NULL || !isfinite(kp_scale) ||
       !(kp_scale>0.0) || kp_scale>2.0) return NULL;
    read_positions(start_raw,&start);
    read_positions(goal_raw,&goal);
    if(motion_limits_check_positions(&start)!=ROBOT_STATUS_OK ||
       motion_limits_check_positions(&goal)!=ROBOT_STATUS_OK)
        return NULL;

    sim=(stage4_sim_t *)calloc(1U,sizeof(*sim));
    if(sim==NULL) return NULL;
    sim->config=*stage4_motion_default_config();
    /*
     * Host-only sweep override. The authoritative default gains still
     * come from stage4_motion_config.c, never from Python copies.
     */
    for(i=0U;i<ROBOT_JOINT_COUNT;i++)
        sim->config.pid[i].kp =
            (robot_real_t)((double)sim->config.pid[i].kp*kp_scale);
    status=joint_controller_init(&sim->controller,sim->config.pid);
    if(status==ROBOT_STATUS_OK)
        status=trajectory_trapezoid_init(
            &sim->path,&start,&goal,
            &sim->config.max_velocity_deg_s,
            &sim->config.max_acceleration_deg_s2);
    if(status!=ROBOT_STATUS_OK)
    {
        free(sim);
        return NULL;
    }
    return sim;
}

STAGE4_SIM_EXPORT void *stage4_sim_create(
    const int32_t start_raw[ROBOT_JOINT_COUNT],
    const int32_t goal_raw[ROBOT_JOINT_COUNT])
{
    return stage4_sim_create_with_kp_scale(start_raw,goal_raw,1.0);
}

STAGE4_SIM_EXPORT double stage4_sim_duration(const void *handle)
{
    const stage4_sim_t *sim=(const stage4_sim_t *)handle;
    return sim==NULL ? -1.0 : sim->path.total_time_s;
}

STAGE4_SIM_EXPORT int stage4_sim_step(
    void *handle,
    const int32_t measured_raw[ROBOT_JOINT_COUNT],
    double elapsed_s,
    double dt_s,
    int32_t commands_raw[ROBOT_JOINT_COUNT],
    int32_t trajectory_raw[ROBOT_JOINT_COUNT])
{
    stage4_sim_t *sim=(stage4_sim_t *)handle;
    robot_joint_positions_t measured={{0}};
    robot_joint_positions_t commanded={{0}};
    trajectory_joint_sample_t sampled={0};
    joint_control_corrections_t corrections={{0}};
    joint_controller_t next_controller;
    joint_command_limiter_t next_limiter;
    robot_status_t status;

    if(sim==NULL || measured_raw==NULL || commands_raw==NULL ||
       trajectory_raw==NULL)
        return (int)ROBOT_STATUS_ERROR_NULL_POINTER;
    if(!isfinite(elapsed_s) || elapsed_s<sim->time_s ||
       !isfinite(dt_s) || !(dt_s>0.0))
        return (int)ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    read_positions(measured_raw,&measured);
    if(motion_limits_check_positions(&measured)!=ROBOT_STATUS_OK)
        return (int)ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    status=trajectory_trapezoid_sample(&sim->path,elapsed_s,&sampled);
    if(status!=ROBOT_STATUS_OK) return (int)status;

    /* Atomic update on failure: retain the previous PID/limiter state. */
    next_controller=sim->controller;
    next_limiter=sim->limiter;
    status=joint_controller_step(&next_controller,
        &sampled.positions,&measured,dt_s,&corrections);
    if(status!=ROBOT_STATUS_OK) return (int)status;
    status=joint_command_limiter_step(&next_limiter,
        &measured,&corrections,
        &sim->config.max_velocity_deg_s,
        &sim->config.max_acceleration_deg_s2,
        robot_model_get_active(),dt_s,&commanded);
    if(status!=ROBOT_STATUS_OK) return (int)status;

    sim->controller=next_controller;
    sim->limiter=next_limiter;
    sim->time_s=elapsed_s;
    write_positions(&commanded,commands_raw);
    write_positions(&sampled.positions,trajectory_raw);
    return (int)ROBOT_STATUS_OK;
}

STAGE4_SIM_EXPORT void stage4_sim_destroy(void *handle)
{
    free(handle);
}
