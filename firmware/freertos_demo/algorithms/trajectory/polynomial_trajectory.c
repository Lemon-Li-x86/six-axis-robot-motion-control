/*
 * Six-axis cubic/quintic time-normalized Hermite interpolation.
 * Position inputs are continuous centidegrees; polynomial coefficients
 * represent degrees as a function of u=t/T. No motor commands.
 */
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "trajectory.h"

static int finite_velocity(const robot_joint_velocities_t *v)
{
    uint32_t j;
    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
        if (!isfinite((double)v->value[j])) return 0;
    return 1;
}

static int finite_acceleration(const trajectory_joint_accelerations_t *a)
{
    uint32_t j;
    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
        if (!isfinite((double)a->value[j])) return 0;
    return 1;
}

static robot_status_t init_common(
    trajectory_polynomial_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *v0,
    const robot_joint_velocities_t *v1,
    const trajectory_joint_accelerations_t *acc0,
    const trajectory_joint_accelerations_t *acc1,
    double duration_s,
    uint8_t degree)
{
    trajectory_polynomial_plan_t next;
    uint32_t j, k;

    if (out == NULL || start == NULL || goal == NULL ||
        v0 == NULL || v1 == NULL ||
        (degree == 5U && (acc0 == NULL || acc1 == NULL)))
        return ROBOT_STATUS_ERROR_NULL_POINTER;

    if (!(duration_s > 0.0) || !isfinite(duration_s) ||
        !finite_velocity(v0) || !finite_velocity(v1) ||
        (degree == 5U && (!finite_acceleration(acc0) ||
                           !finite_acceleration(acc1))))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    memset(&next, 0, sizeof(next));
    next.start_positions = *start;
    next.goal_positions = *goal;
    next.duration_s = duration_s;
    next.degree = degree;

    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
    {
        double p0 = (double)start->value[j] * 0.01;
        double p1 = (double)goal->value[j] * 0.01;
        double d = p1 - p0;
        double v0t = (double)v0->value[j] * duration_s;
        double v1t = (double)v1->value[j] * duration_s;
        double *c = next.coefficients_deg[j];

        c[0] = p0;
        c[1] = v0t;
        if (degree == 3U)
        {
            c[2] = 3.0 * d - 2.0 * v0t - v1t;
            c[3] = -2.0 * d + v0t + v1t;
        }
        else
        {
            double a0t2 = (double)acc0->value[j] *
                duration_s * duration_s;
            double a1t2 = (double)acc1->value[j] *
                duration_s * duration_s;
            double b0, b1, b2;
            c[2] = 0.5 * a0t2;
            b0 = d - c[1] - c[2];
            b1 = v1t - c[1] - 2.0 * c[2];
            b2 = a1t2 - 2.0 * c[2];
            c[3] = 10.0 * b0 - 4.0 * b1 + 0.5 * b2;
            c[4] = -15.0 * b0 + 7.0 * b1 - b2;
            c[5] = 6.0 * b0 - 3.0 * b1 + 0.5 * b2;
        }
        for (k = 0U; k < 6U; ++k)
            if (!isfinite(c[k]))
                return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }

    *out = next;
    return ROBOT_STATUS_OK;
}

robot_status_t trajectory_polynomial_init_cubic(
    trajectory_polynomial_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *start_velocity_deg_s,
    const robot_joint_velocities_t *goal_velocity_deg_s,
    double duration_s)
{
    return init_common(out, start, goal,
        start_velocity_deg_s, goal_velocity_deg_s,
        NULL, NULL, duration_s, 3U);
}

robot_status_t trajectory_polynomial_init_quintic(
    trajectory_polynomial_plan_t *out,
    const robot_joint_positions_t *start,
    const robot_joint_positions_t *goal,
    const robot_joint_velocities_t *start_velocity_deg_s,
    const robot_joint_velocities_t *goal_velocity_deg_s,
    const trajectory_joint_accelerations_t *start_acceleration_deg_s2,
    const trajectory_joint_accelerations_t *goal_acceleration_deg_s2,
    double duration_s)
{
    return init_common(out, start, goal,
        start_velocity_deg_s, goal_velocity_deg_s,
        start_acceleration_deg_s2, goal_acceleration_deg_s2,
        duration_s, 5U);
}

robot_status_t trajectory_polynomial_sample(
    const trajectory_polynomial_plan_t *plan,
    double elapsed_s,
    trajectory_joint_sample_t *out)
{
    trajectory_joint_sample_t next;
    double u;
    uint32_t j;

    if (plan == NULL || out == NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if (!isfinite(elapsed_s) || elapsed_s < 0.0 ||
        !isfinite(plan->duration_s) || plan->duration_s <= 0.0 ||
        (plan->degree != 3U && plan->degree != 5U))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    u = elapsed_s >= plan->duration_s ? 1.0 :
        elapsed_s / plan->duration_s;
    memset(&next, 0, sizeof(next));

    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
    {
        const double *c = plan->coefficients_deg[j];
        double p = c[plan->degree];
        double v = 0.0;
        double a = 0.0;
        double raw;
        int k;

        /* Extended Horner evaluates p, dp/du, d2p/du2 together. */
        for (k = (int)plan->degree - 1; k >= 0; --k)
        {
            a = a * u + 2.0 * v;
            v = v * u + p;
            p = p * u + c[k];
        }
        v /= plan->duration_s;
        a /= plan->duration_s * plan->duration_s;

        if (!isfinite(p) || !isfinite(v) || !isfinite(a) ||
            fabs(v) > FLT_MAX || fabs(a) > FLT_MAX)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

        raw = p * 100.0;
        if (u == 0.0) raw = (double)plan->start_positions.value[j];
        if (u == 1.0) raw = (double)plan->goal_positions.value[j];
        if (!isfinite(raw) || raw < (double)INT32_MIN ||
            raw > (double)INT32_MAX)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

        next.positions.value[j] = (robot_joint_position_t)llround(raw);
        next.velocities.value[j] = (robot_real_t)v;
        next.accelerations.value[j] = (robot_real_t)a;
    }

    *out = next;
    return ROBOT_STATUS_OK;
}
