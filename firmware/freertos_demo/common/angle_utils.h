/* One canonicalization utility for continuous->wire/kinematics angles.
 * Not a model or configuration file. Units are defined only in robot_types.
 */
#ifndef ROBOT_ANGLE_UTILS_H
#define ROBOT_ANGLE_UTILS_H
#include "robot_types.h"
#include "error_code.h"
robot_status_t robot_angles_canonicalize(
    const robot_joint_positions_t *continuous,
    robot_joint_angles_t *canonical);
#endif
