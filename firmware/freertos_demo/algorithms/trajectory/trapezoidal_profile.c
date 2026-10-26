/*
 * Synchronized rest-to-rest joint-space trapezoid.
 * One dimensionless progress s(t) makes all six axes finish together.
 * Plan and sample functions are pure, stateless and heap-free.
 */
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "trajectory.h"

robot_status_t trajectory_trapezoid_init(
    trajectory_trapezoid_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2)
{
    trajectory_trapezoid_plan_t next;
    double v_limit = HUGE_VAL;
    double a_limit = HUGE_VAL;
    uint32_t j;
    uint8_t moving = 0U;

    if (out == NULL || start == NULL || goal == NULL ||
        max_velocity_deg_s == NULL || max_acceleration_deg_s2 == NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;

    memset(&next, 0, sizeof(next));
    next.start_positions = *start;
    next.goal_positions = *goal;

    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
    {
        double v = (double)max_velocity_deg_s->value[j];
        double a = (double)max_acceleration_deg_s2->value[j];
        double distance = fabs((double)goal->value[j] -
                               (double)start->value[j]) * (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
        if (!(v > 0.0) || !(a > 0.0) ||
            !isfinite(v) || !isfinite(a))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        if (distance > 0.0)
        {
            double vs = v / distance;
            double as = a / distance;
            moving = 1U;
            if (vs < v_limit) v_limit = vs;
            if (as < a_limit) a_limit = as;
        }
    }

    if (!moving)
    {
        *out = next;
        return ROBOT_STATUS_OK;
    }

    next.acceleration_fraction_s2 = a_limit;
    next.acceleration_time_s = v_limit / a_limit;
    if (a_limit * next.acceleration_time_s *
        next.acceleration_time_s >= 1.0)
    {
        /* Triangular profile: acceleration occupies all path progress. */
        next.acceleration_time_s = sqrt(1.0 / a_limit);
        next.cruise_time_s = 0.0;
        next.velocity_fraction_s =
            a_limit * next.acceleration_time_s;
    }
    else
    {
        next.velocity_fraction_s = v_limit;
        next.cruise_time_s = (1.0 - a_limit *
            next.acceleration_time_s * next.acceleration_time_s) / v_limit;
    }
    next.total_time_s = 2.0 * next.acceleration_time_s +
        next.cruise_time_s;

    if (!isfinite(next.total_time_s) || next.total_time_s <= 0.0 ||
        !isfinite(next.velocity_fraction_s))
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

    *out = next;
    return ROBOT_STATUS_OK;
}

robot_status_t trajectory_trapezoid_sample(
    const trajectory_trapezoid_plan_t *plan,
    double elapsed_s,
    trajectory_joint_sample_t *out)
{
    trajectory_joint_sample_t next;
    double s = 0.0, sd = 0.0, sdd = 0.0;
    double ta, tc, T, a, v;
    uint32_t j;

    if (plan == NULL || out == NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if (!isfinite(elapsed_s) || elapsed_s < 0.0 ||
        !isfinite(plan->total_time_s) || plan->total_time_s < 0.0)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    memset(&next, 0, sizeof(next));
    T = plan->total_time_s;
    ta = plan->acceleration_time_s;
    tc = plan->cruise_time_s;
    a = plan->acceleration_fraction_s2;
    v = plan->velocity_fraction_s;

    if (T == 0.0)
    {
        for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
            if (plan->start_positions.value[j] !=
                plan->goal_positions.value[j])
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }
    else if (elapsed_s >= T)
    {
        s = 1.0;
    }
    else if (elapsed_s < ta)
    {
        s = 0.5 * a * elapsed_s * elapsed_s;
        sd = a * elapsed_s;
        sdd = a;
    }
    else if (elapsed_s < ta + tc)
    {
        s = 0.5 * a * ta * ta + v * (elapsed_s - ta);
        sd = v;
    }
    else
    {
        double remain = T - elapsed_s;
        s = 1.0 - 0.5 * a * remain * remain;
        sd = a * remain;
        sdd = -a;
    }

    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
    {
        double delta = ((double)plan->goal_positions.value[j] -
                        (double)plan->start_positions.value[j]);
        double raw = (double)plan->start_positions.value[j] + delta * s;
        double velocity = delta * (double)ROBOT_JOINT_ANGLE_UNIT_DEG * sd;
        double accel = delta * (double)ROBOT_JOINT_ANGLE_UNIT_DEG * sdd;
        if (elapsed_s >= T)
            raw = (double)plan->goal_positions.value[j];
        if (!isfinite(raw) || !isfinite(velocity) || !isfinite(accel) ||
            raw < (double)INT32_MIN || raw > (double)INT32_MAX ||
            fabs(velocity) > FLT_MAX || fabs(accel) > FLT_MAX)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        next.positions.value[j] = (robot_joint_position_t)llround(raw);
        next.velocities.value[j] = (robot_real_t)velocity;
        next.accelerations.value[j] = (robot_real_t)accel;
    }

    *out = next;
    return ROBOT_STATUS_OK;
}
