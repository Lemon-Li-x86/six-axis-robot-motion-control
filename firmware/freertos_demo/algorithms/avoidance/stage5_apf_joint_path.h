/* Stage5: convert an already validated Cartesian APF point route into
 * continuous UR5 joints using existing production IK, selector, FK and
 * trapezoidal segment timing. No UART, FreeRTOS or motor commands.
 *
 * This is NOT an all-link collision check: the returned joint path must be
 * checked by the separate PyBullet all-link static obstacle gate BEFORE
 * any simulation-only motion request is allowed.
 */
#ifndef STAGE5_APF_JOINT_PATH_H
#define STAGE5_APF_JOINT_PATH_H
#include <stddef.h>
#include "potential_field_path.h"
#include "trajectory.h"
#include "robot_types.h"
#include "error_code.h"

typedef struct {
    robot_joint_positions_t target;
    /* Minimum rest-to-rest travel time from the existing trapezoid planner.
     * Units: seconds; no implicit 20ms-per-APF-point execution. */
    double segment_duration_s;
} stage5_apf_joint_segment_t;

/* The constant orientation is taken from FK(start_joints). Every APF point
 * is in base_link millimetres, NOT PyBullet world-frame metres.
 * Uses existing robot model joint limits and IK solution selection.
 * Rejects every failed FK residual, IK singularity, unreachable point or
 * invalid velocity/acceleration configuration (all provided by caller).
 *
 * Transactional result: on rejection, *out_count=0; segments are untouched.
 * Two-pass preflight requires immutable, non-overlapping inputs throughout
 * the call. This function does not guarantee collision-free swept links.
 * A caller must provide non-NULL segments and capacity>=point_count.
 * Empty point_count is valid: no actuation requested.
 */
robot_status_t stage5_apf_points_to_joint_route(
    const robot_joint_positions_t *start_joints,
    const robot_apf_point_t *cartesian_points,
    size_t point_count,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    robot_real_t max_fk_position_error_mm,
    robot_real_t max_fk_rotation_element_error,
    stage5_apf_joint_segment_t *segments,
    size_t capacity,
    size_t *out_count);
#endif
