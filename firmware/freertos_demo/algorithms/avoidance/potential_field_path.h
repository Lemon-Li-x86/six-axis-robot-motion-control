/* Bounded multi-step Cartesian APF path. Pure C, never actuates motors.
 * Output contains intermediate end-effector points, NOT a robot-safe plan.
 * Each swept point segment is checked by robot_apf_step against AABBs.
 * ALL robot links, IK, timing and joint constraints remain caller gates.
 */
#ifndef ROBOT_POTENTIAL_FIELD_PATH_H
#define ROBOT_POTENTIAL_FIELD_PATH_H
#include "potential_field.h"

/* On success, points[0..*count-1] are successive waypoints from start.
 * *count=0 is legal when start is already inside goal tolerance.
 * On error, *count=0 and points are unchanged (two-pass preflight).
 * No allocation, no global state. Capacity must be >0.
 * Finite capacity is the mandatory escape from APF local minima/cycles.
 */
robot_status_t robot_apf_plan_path(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *points,
    size_t capacity,
    size_t *count);

/* Deterministic, allocation-free AABB bypass candidates.
 * This is a geometric proposal generator, NOT a second potential-field law.
 * The application must run each proposed via through production C APF,
 * C IK, the complete PyBullet swept-link preflight and runtime clearance.
 *
 * The current bounded version supports one static AABB intersecting the
 * direct segment. Other obstacle sets are rejected rather than ignored.
 * Generated candidates depend only on observed geometry and the caller's
 * existing APF clearance/step policy: no hidden robot/scene defaults.
 */
#define ROBOT_APF_BYPASS_MAX_CANDIDATES 12U
robot_status_t robot_apf_generate_bypass_vias(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacle,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *via_candidates,
    size_t capacity,
    size_t *out_count);

#endif
