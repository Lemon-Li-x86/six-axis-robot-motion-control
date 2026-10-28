/*
 * Joint command velocity/acceleration limiter.
 *
 * Pure algorithm: six-axis continuous position commands, no RTOS, no UART.
 * Command positions use centidegrees; correction in degrees; dt in seconds.
 * Fractional command positions are preserved across ticks to prevent a
 * 0.01 degree integer quantization deadlock at small dt / acceleration.
 *
 * Configuration and physical joint limits come from callers; nothing in
 * this module redefines UR5 geometry or global Stage4 defaults.
 */
#ifndef ROBOT_JOINT_COMMAND_LIMITER_H
#define ROBOT_JOINT_COMMAND_LIMITER_H

#include <stdint.h>
#include "joint_controller.h"
#include "trajectory.h"
#include "robot_model.h"

typedef struct
{
    double command_raw[ROBOT_JOINT_COUNT];
    double velocity_deg_s[ROBOT_JOINT_COUNT];
    uint8_t initialized;
} joint_command_limiter_t;

void joint_command_limiter_reset(joint_command_limiter_t *limiter);

robot_status_t joint_command_limiter_step(
    joint_command_limiter_t *limiter,
    const robot_joint_positions_t *measured,
    const joint_control_corrections_t *corrections,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    const robot_model_t *model,
    double dt_s,
    robot_joint_positions_t *out);

#endif
