#include <math.h>
#include <stddef.h>
#include "cartesian_internal.h"

robot_status_t cartesian_line_init(
    cartesian_trajectory_plan_t *out,
    const robot_transform_t *start,
    const robot_transform_t *end,
    double duration_s, double interpolation_period_s)
{
    return cartesian_prepare(out,start,end,duration_s,
                             interpolation_period_s,CARTESIAN_PATH_LINE);
}

robot_status_t cartesian_trajectory_sample(
    const cartesian_trajectory_plan_t *plan,
    uint32_t sample_index, robot_transform_t *out)
{
    robot_transform_t next;
    double t,u,ang;
    uint32_t i;
    if(plan==NULL || out==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if((plan->kind!=CARTESIAN_PATH_LINE &&
        plan->kind!=CARTESIAN_PATH_ARC) ||
       plan->interval_count==0U ||
       !(plan->duration_s>0.0) || !(plan->period_s>0.0) ||
       sample_index>plan->interval_count)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    t=fmin(plan->duration_s,(double)sample_index*plan->period_s);
    if(sample_index==plan->interval_count) t=plan->duration_s;
    u=t/plan->duration_s;
    next=plan->start;
    cartesian_interpolate_rotation(plan,u,&next);
    if(plan->kind==CARTESIAN_PATH_LINE) {
        for(i=0U;i<3U;i++)
            next.matrix[i][3]=(robot_real_t)(
                (1.0-u)*(double)plan->start.matrix[i][3]+
                u*(double)plan->end.matrix[i][3]);
    } else {
        if(!(plan->radius_mm>0.0) || !isfinite(plan->signed_sweep_rad))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        ang=u*plan->signed_sweep_rad;
        for(i=0U;i<3U;i++)
            next.matrix[i][3]=(robot_real_t)(
                plan->center_mm[i]+plan->radius_mm*
                (plan->radial_u[i]*cos(ang)+
                 plan->radial_v[i]*sin(ang)));
    }
    if(sample_index==0U) next=plan->start;
    if(sample_index==plan->interval_count) next=plan->end;
    for(i=0U;i<3U;i++)
        if(!isfinite((double)next.matrix[i][3]))
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    *out=next;
    return ROBOT_STATUS_OK;
}
