/* Private reusable SO(3), input-validation and fixed-time helpers.
 * Only Cartesian modules include this header; one implementation owns math.
 */
#ifndef ROBOT_CARTESIAN_INTERNAL_H
#define ROBOT_CARTESIAN_INTERNAL_H
#include "cartesian_trajectory.h"
robot_status_t cartesian_prepare(
    cartesian_trajectory_plan_t *out,
    const robot_transform_t *start,
    const robot_transform_t *end,
    double duration_s, double period_s,
    cartesian_path_kind_t kind);
void cartesian_interpolate_rotation(
    const cartesian_trajectory_plan_t *plan,
    double u,
    robot_transform_t *out);
#endif
