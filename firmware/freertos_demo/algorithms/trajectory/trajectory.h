/*
 * Stage 4: pure joint-space trajectory planning API.
 * No FreeRTOS, UART, motor driver, dynamic allocation or hidden state.
 *
 * Units: positions in continuous centidegrees (0.01 degree);
 * velocities in degree/s; accelerations in degree/s^2; time in seconds.
 * Caller owns model-limit, collision and control-loop checks.
 */
#ifndef ROBOT_TRAJECTORY_H
#define ROBOT_TRAJECTORY_H

#include <stdint.h>
#include "robot_types.h"
#include "error_code.h"

typedef struct
{
    robot_real_t value[ROBOT_JOINT_COUNT];
} trajectory_joint_accelerations_t;

typedef struct
{
    robot_joint_positions_t positions;
    robot_joint_velocities_t velocities;
    trajectory_joint_accelerations_t accelerations;
} trajectory_joint_sample_t;

typedef struct
{
    robot_joint_positions_t start_positions;
    robot_joint_positions_t goal_positions;
    double coefficients_deg[ROBOT_JOINT_COUNT][6];
    double duration_s;
    uint8_t degree; /* 3 or 5; set by a successful init only */
} trajectory_polynomial_plan_t;

/*
 * Cubic interpolation matches start/end position and velocity.
 * An initialized plan can be sampled repeatedly without mutation.
 * Invalid inputs leave the output plan unchanged.
 */
robot_status_t trajectory_polynomial_init_cubic(
    trajectory_polynomial_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *start_velocity_deg_s,
    const robot_joint_velocities_t *goal_velocity_deg_s,
    double duration_s);

/*
 * Quintic interpolation also matches acceleration at both endpoints.
 */
robot_status_t trajectory_polynomial_init_quintic(
    trajectory_polynomial_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *start_velocity_deg_s,
    const robot_joint_velocities_t *goal_velocity_deg_s,
    const trajectory_joint_accelerations_t *start_acceleration_deg_s2,
    const trajectory_joint_accelerations_t *goal_acceleration_deg_s2,
    double duration_s);

/* t<0 or NaN/Inf is rejected; t>duration clamps to the final state.
 * Outputs are continuous centidegrees, NOT wrapped angles.
 * Invalid inputs or unsafe numeric results leave output unchanged.
 */
robot_status_t trajectory_polynomial_sample(
    const trajectory_polynomial_plan_t *plan,
    double elapsed_s,
    trajectory_joint_sample_t *out);

/*
 * Rest-to-rest synchronized six-axis trapezoid:
 * all moving joints use one scalar progress s(t) and finish together.
 * max velocity/acceleration are strictly positive on every joint,
 * even joints whose displacement is zero.
 * Zero displacement on all axes produces duration=0 (stationary plan).
 */
typedef struct
{
    robot_joint_positions_t start_positions;
    robot_joint_positions_t goal_positions;
    double acceleration_fraction_s2;
    double velocity_fraction_s;
    double acceleration_time_s;
    double cruise_time_s;
    double total_time_s;
} trajectory_trapezoid_plan_t;

robot_status_t trajectory_trapezoid_init(
    trajectory_trapezoid_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2);

robot_status_t trajectory_trapezoid_sample(
    const trajectory_trapezoid_plan_t *plan,
    double elapsed_s,
    trajectory_joint_sample_t *out);

#endif
