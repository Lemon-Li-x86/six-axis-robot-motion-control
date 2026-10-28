/*
 * Stage5 experimental Cartesian potential-field step for static AABBs.
 *
 * Pure C11: no FreeRTOS, PyBullet, UART, globals or memory allocation.
 * All positions and AABB bounds are in the public base_link frame, mm.
 * This controls only the END-EFFECTOR POINT. It is not a full-link
 * collision checker and is never safe to drive a real robot without
 * full-robot collision checking, IK and trajectory rate validation.
 */
#ifndef ROBOT_POTENTIAL_FIELD_H
#define ROBOT_POTENTIAL_FIELD_H

#include <stddef.h>
#include "error_code.h"

#define ROBOT_APF_MAX_OBSTACLES 8U

typedef struct {
    double mm[3];
} robot_apf_point_t;

typedef struct {
    robot_apf_point_t min_corner;
    robot_apf_point_t max_corner;
} robot_apf_aabb_t;

typedef struct {
    double attraction_gain;
    double repulsion_gain;
    double influence_distance_mm;
    double clearance_mm;
    double maximum_step_mm;
    double goal_tolerance_mm;
} robot_apf_config_t;

typedef struct {
    double nearest_surface_distance_mm; /* +INFINITY if no obstacles */
    double chosen_step_mm;
    unsigned int reached_goal;
} robot_apf_step_info_t;

/*
 * Return a bounded next waypoint outside every expanded obstacle.
 * A current/goal point inside any expanded AABB is rejected.
 * The SEGMENT from current to next is tested against expanded AABBs.
 *
 * If potential-field direction stagnates or the segment crosses a box,
 * return NO_SOLUTION; do not silently move through the obstacle.
 * Parameters belong to the caller; this module has NO default gains.
 *
 * No partial output writes on failure, including in-place next==current.
 */
robot_status_t robot_apf_step(
    const robot_apf_point_t *current,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *next,
    robot_apf_step_info_t *details);

#endif
