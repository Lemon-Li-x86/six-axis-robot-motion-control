/* Six axis PID wrapper. One controller instance per joint; no global state.
 * Inputs positions in continuous 0.01deg; output CORRECTIONS in degrees.
 */
#ifndef ROBOT_JOINT_CONTROLLER_H
#define ROBOT_JOINT_CONTROLLER_H
#include "pid.h"
typedef struct {
    robot_real_t value[ROBOT_JOINT_COUNT];
} joint_control_corrections_t;
typedef struct {
    pid_controller_t joints[ROBOT_JOINT_COUNT];
} joint_controller_t;
robot_status_t joint_controller_init(joint_controller_t *controller,
                                      const pid_config_t config[ROBOT_JOINT_COUNT]);
void joint_controller_reset(joint_controller_t *controller);
robot_status_t joint_controller_step(joint_controller_t *controller,
                                      const robot_joint_positions_t *setpoint,
                                      const robot_joint_positions_t *feedback,
                                      double dt_s,
                                      joint_control_corrections_t *out);
#endif
