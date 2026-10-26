#include <math.h>
#include <float.h>
#include <stddef.h>
#include <string.h>
#include "pid.h"

static double clamp(double x,double lo,double hi)
{
    if(x>hi) return hi;
    if(x<lo) return lo;
    return x;
}

robot_status_t pid_init(pid_controller_t *state,const pid_config_t *config)
{
    pid_controller_t next;
    if(state==NULL || config==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(!isfinite((double)config->kp) ||
       !isfinite((double)config->ki) ||
       !isfinite((double)config->kd) ||
       !isfinite((double)config->integral_limit_deg_s) ||
       !isfinite((double)config->output_limit_deg) ||
       !isfinite((double)config->deadband_deg) ||
       config->kp<0.0F || config->ki<0.0F || config->kd<0.0F ||
       config->integral_limit_deg_s<0.0F ||
       !(config->output_limit_deg>0.0F) ||
       config->deadband_deg<0.0F)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    memset(&next,0,sizeof(next));
    next.config=*config;
    *state=next;
    return ROBOT_STATUS_OK;
}

void pid_reset(pid_controller_t *state)
{
    if(state==NULL) return;
    state->integral_deg_s=0.0;
    state->prev_error_deg=0.0;
    state->prev_prev_error_deg=0.0;
    state->output_deg=0.0;
    state->initialized=0U;
}

robot_status_t pid_step(pid_controller_t *state,double setpoint_deg,
                         double measured_deg,double dt_s,
                         robot_real_t *correction_deg)
{
    pid_controller_t next;
    double error,new_i,delta_p,delta_d,raw,output,ilim,olim;
    if(state==NULL || correction_deg==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(!isfinite(setpoint_deg) || !isfinite(measured_deg) ||
       !(dt_s>0.0) || !isfinite(dt_s) ||
       !(state->config.output_limit_deg>0.0F))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    next=*state;
    error=setpoint_deg-measured_deg;
    if(fabs(error)<=(double)next.config.deadband_deg) error=0.0;
    if(!next.initialized) {
        /* First sample applies proportional error without derivative kick. */
        next.prev_error_deg=0.0;
        next.prev_prev_error_deg=-error;
        next.initialized=1U;
    }
    ilim=(double)next.config.integral_limit_deg_s;
    olim=(double)next.config.output_limit_deg;
    new_i=clamp(next.integral_deg_s+error*dt_s,-ilim,ilim);
    delta_p=(double)next.config.kp*(error-next.prev_error_deg);
    delta_d=(double)next.config.kd*
        (error-2.0*next.prev_error_deg+next.prev_prev_error_deg)/dt_s;
    raw=next.output_deg+delta_p+
        (double)next.config.ki*(new_i-next.integral_deg_s)+delta_d;
    if(!isfinite(raw)) return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

    /* Conditional integration prevents windup in the saturated direction. */
    if((raw>olim && error>0.0) || (raw< -olim && error<0.0)) {
        new_i=next.integral_deg_s;
        raw=next.output_deg+delta_p+delta_d;
    }
    output=clamp(raw,-olim,olim);
    if(!isfinite(output) || fabs(output)>FLT_MAX)
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    next.integral_deg_s=new_i;
    next.prev_prev_error_deg=next.prev_error_deg;
    next.prev_error_deg=error;
    next.output_deg=output;
    *state=next;
    *correction_deg=(robot_real_t)output;
    return ROBOT_STATUS_OK;
}
