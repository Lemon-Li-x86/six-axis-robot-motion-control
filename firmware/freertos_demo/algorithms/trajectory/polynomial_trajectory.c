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

/* Evaluate ascending power coefficients on normalized time u in [0,1]. */
static double polynomial_value(const double *c, unsigned degree, double u)
{
    double value = c[degree];
    while (degree != 0U) value = value * u + c[--degree];
    return value;
}

/* Isolate roots between derivative roots, where the polynomial is monotone.
 * Only interior roots are returned; callers separately test both endpoints.
 * Degree is at most four here, so recursion and storage are statically bounded.
 * A repeated root without a sign change is not an extremum of its integral.
 */
static unsigned polynomial_unit_roots(
    const double *c, unsigned degree, double *roots)
{
    double derivative[5], stationary[5];
    double left = 0.0;
    unsigned i, count = 0U, stationary_count;
    while (degree != 0U && c[degree] == 0.0) --degree;
    if (degree == 0U) return 0U;
    if (degree == 1U) {
        double root = -c[0] / c[1];
        if (root > 0.0 && root < 1.0) {
            roots[0] = root;
            return 1U;
        }
        return 0U;
    }
    for (i = 1U; i <= degree; ++i)
        derivative[i - 1U] = (double)i * c[i];
    stationary_count = polynomial_unit_roots(
        derivative, degree - 1U, stationary);
    for (i = 0U; i <= stationary_count; ++i) {
        double right = i < stationary_count ? stationary[i] : 1.0;
        double a = left, b = right;
        double fa = polynomial_value(c, degree, a);
        double fb = polynomial_value(c, degree, b);
        if ((fa < 0.0 && fb > 0.0) || (fa > 0.0 && fb < 0.0)) {
            unsigned iteration;
            for (iteration = 0U; iteration < 64U; ++iteration) {
                double middle = a + (b - a) * 0.5;
                double fm = polynomial_value(c, degree, middle);
                if (fm == 0.0) { a = b = middle; break; }
                if ((fa < 0.0 && fm > 0.0) || (fa > 0.0 && fm < 0.0))
                    b = middle;
                else { a = middle; fa = fm; }
            }
            if (count < degree) roots[count++] = a + (b - a) * 0.5;
        }
        if (fb == 0.0 && right < 1.0 && count < degree)
            roots[count++] = right;
        left = right;
    }
    return count;
}

/* A polynomial reaches its bounds at endpoints or derivative roots.
 * The epsilon allowance accounts only for double arithmetic, not actuator
 * or quantization tolerance. No configurable motion bound is widened.
 */
static int polynomial_within_bounds(
    const double *c, unsigned degree, double minimum, double maximum)
{
    double derivative[5], roots[5];
    double scale = fmax(1.0, fmax(fabs(minimum), fabs(maximum)));
    double epsilon = 64.0 * DBL_EPSILON * scale;
    unsigned i, count;
    for (i = 1U; i <= degree; ++i)
        derivative[i - 1U] = (double)i * c[i];
    count = polynomial_unit_roots(derivative, degree - 1U, roots);
    for (i = 0U; i < count + 2U; ++i) {
        double u = i < count ? roots[i] : (i == count ? 0.0 : 1.0);
        double value = polynomial_value(c, degree, u);
        if (!isfinite(value) || value < minimum - epsilon ||
            value > maximum + epsilon) return 0;
    }
    return 1;
}

robot_status_t trajectory_polynomial_validate_limits(
    const trajectory_polynomial_plan_t *plan,
    const robot_joint_positions_t *minimum,
    const robot_joint_positions_t *maximum,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2)
{
    unsigned j, k;
    double inverse_time;
    if (plan == NULL || minimum == NULL || maximum == NULL ||
        max_velocity_deg_s == NULL || max_acceleration_deg_s2 == NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if ((plan->degree != 3U && plan->degree != 5U) ||
        !isfinite(plan->duration_s) || !(plan->duration_s > 0.0))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    inverse_time = 1.0 / plan->duration_s;
    if (!isfinite(inverse_time) || !isfinite(inverse_time * inverse_time))
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j) {
        const double *c = plan->coefficients_deg[j];
        double velocity[5] = {0}, acceleration[4] = {0};
        double vmax = max_velocity_deg_s->value[j];
        double amax = max_acceleration_deg_s2->value[j];
        double endpoint_scale = 1.0;
        double p0 = (double)plan->start_positions.value[j] * ROBOT_JOINT_ANGLE_UNIT_DEG;
        double p1 = (double)plan->goal_positions.value[j] * ROBOT_JOINT_ANGLE_UNIT_DEG;
        if (minimum->value[j] > maximum->value[j] || !isfinite(vmax) ||
            !isfinite(amax) || !(vmax > 0.0) || !(amax > 0.0))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        for (k = 0U; k < 6U; ++k) {
            if (!isfinite(c[k])) return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            /* Leave headroom for all derivative/Horner intermediate values. */
            if (fabs(c[k]) > DBL_MAX / 720.0)
                return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
            if (k > plan->degree && c[k] != 0.0)
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            endpoint_scale += fabs(c[k]);
        }
        if (fabs(c[0] - p0) > 64.0 * DBL_EPSILON * endpoint_scale ||
            fabs(polynomial_value(c, plan->degree, 1.0) - p1) >
                64.0 * DBL_EPSILON * endpoint_scale)
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        for (k = 1U; k <= plan->degree; ++k) {
            velocity[k - 1U] = (double)k * c[k] * inverse_time;
            if (!isfinite(velocity[k - 1U]) ||
                fabs(velocity[k - 1U]) > DBL_MAX / 720.0)
                return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        }
        for (k = 1U; k < plan->degree; ++k) {
            acceleration[k - 1U] = (double)k * velocity[k] * inverse_time;
            if (!isfinite(acceleration[k - 1U]) ||
                fabs(acceleration[k - 1U]) > DBL_MAX / 720.0)
                return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        }
        if (!polynomial_within_bounds(c, plan->degree,
                (double)minimum->value[j] * ROBOT_JOINT_ANGLE_UNIT_DEG,
                (double)maximum->value[j] * ROBOT_JOINT_ANGLE_UNIT_DEG) ||
            !polynomial_within_bounds(velocity, plan->degree - 1U, -vmax, vmax) ||
            !polynomial_within_bounds(acceleration, plan->degree - 2U, -amax, amax))
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }
    return ROBOT_STATUS_OK;
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
        double p0 = (double)start->value[j] * (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
        double p1 = (double)goal->value[j] * (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
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

        raw = p / (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
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
