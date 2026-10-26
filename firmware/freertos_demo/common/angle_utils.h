/* One canonicalization utility for continuous->wire/kinematics angles.
 * Not a model or configuration file. Units are defined only in robot_types.
 */
#ifndef ROBOT_ANGLE_UTILS_H
#define ROBOT_ANGLE_UTILS_H
#include "robot_types.h"
#include "error_code.h"
/* Single implementation of canonical angle wrap used by driver and Stage4. */
robot_joint_angle_t robot_joint_angle_normalize_raw(int32_t raw_angle);
robot_status_t robot_angles_canonicalize(
    const robot_joint_positions_t *continuous,
    robot_joint_angles_t *canonical);
#endif
