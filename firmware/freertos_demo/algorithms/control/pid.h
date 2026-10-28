/* Stage 4 incremental (velocity-form) PID.
 * Error/outputs use position degrees; dt seconds; all config is caller-owned.
 * This module is pure calculation, without RTOS or actuator interfaces.
 */
#ifndef ROBOT_PID_H
#define ROBOT_PID_H
#include "robot_types.h"
#include "error_code.h"
#include <stdint.h>

typedef struct {
    robot_real_t kp;
    robot_real_t ki;
    robot_real_t kd;
    robot_real_t integral_limit_deg_s;
    robot_real_t output_limit_deg;
    robot_real_t deadband_deg;
} pid_config_t;

typedef struct {
    pid_config_t config;
    double integral_deg_s;
    double prev_error_deg;
    double prev_prev_error_deg;
    double output_deg;
    uint8_t initialized;
} pid_controller_t;

robot_status_t pid_init(pid_controller_t *state,const pid_config_t *config);
void pid_reset(pid_controller_t *state);
robot_status_t pid_step(pid_controller_t *state,double setpoint_deg,
                         double measured_deg,double dt_s,
                         robot_real_t *correction_deg);
#endif
