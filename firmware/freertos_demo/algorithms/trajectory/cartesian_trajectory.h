/* Stage 4 Cartesian line/arc sampled trajectory.
 * Transform convention is existing robot_transform_t: base_link -> ee_link,
 * translation mm, rotation matrix. All time is seconds.
 * No FreeRTOS, UART, motor state, global configuration or allocation.
 */
#ifndef ROBOT_CARTESIAN_TRAJECTORY_H
#define ROBOT_CARTESIAN_TRAJECTORY_H

#include <stdint.h>
#include "robot_types.h"
#include "error_code.h"

typedef enum {
    CARTESIAN_PATH_LINE = 1,
    CARTESIAN_PATH_ARC = 2
} cartesian_path_kind_t;

typedef struct {
    cartesian_path_kind_t kind;
    robot_transform_t start;
    robot_transform_t end;
    double duration_s;
    double period_s;
    uint32_t interval_count;
    /* Circle geometry is meaningful only for ARC. */
    double center_mm[3];
    double radial_u[3];
    double radial_v[3];
    double radius_mm;
    double signed_sweep_rad;
    /* Shortest-rotation unit quaternion of endpoint orientation. */
    double start_quat[4];  /* w,x,y,z */
    double end_quat[4];
} cartesian_trajectory_plan_t;

robot_status_t cartesian_line_init(
    cartesian_trajectory_plan_t *out,
    const robot_transform_t *start,
    const robot_transform_t *end,
    double duration_s, double interpolation_period_s);

/* Circle through start, via, end (three noncollinear translations).
 * The via pose supplies geometric position only; orientation uses
 * shortest rotation from start to end at constant normalized progress. */
robot_status_t cartesian_arc_init(
    cartesian_trajectory_plan_t *out,
    const robot_transform_t *start,
    const robot_transform_t *via,
    const robot_transform_t *end,
    double duration_s, double interpolation_period_s);

/* Samples 0..interval_count inclusive, last sample EXACTLY at end time. */
robot_status_t cartesian_trajectory_sample(
    const cartesian_trajectory_plan_t *plan,
    uint32_t sample_index,
    robot_transform_t *out);

/* Convert a Cartesian sample through existing IK + selector.
 * current continuous joints are a SEPARATE input, never global state.
 * Caller validates selected FK consistency against its own tolerance. */
robot_status_t cartesian_trajectory_joint_waypoint(
    const cartesian_trajectory_plan_t *plan,
    uint32_t sample_index,
    const robot_joint_positions_t *current,
    robot_joint_positions_t *selected,
    robot_transform_t *sampled_pose);

#endif
