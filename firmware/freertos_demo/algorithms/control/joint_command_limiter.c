#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "joint_command_limiter.h"

static double clamp_double(double value, double minimum, double maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void joint_command_limiter_reset(joint_command_limiter_t *limiter)
{
    if (limiter != NULL)
        memset(limiter, 0, sizeof(*limiter));
}

robot_status_t joint_command_limiter_step(
    joint_command_limiter_t *limiter,
    const robot_joint_positions_t *measured,
    const joint_control_corrections_t *corrections,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    const robot_model_t *model,
    double dt_s,
    robot_joint_positions_t *out)
{
    joint_command_limiter_t next;
    robot_joint_positions_t positions = {{0}};
    uint32_t j;

    if (limiter == NULL || measured == NULL || corrections == NULL ||
        max_velocity_deg_s == NULL || max_acceleration_deg_s2 == NULL ||
        model == NULL || out == NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;

    if (!(dt_s > 0.0) || !isfinite(dt_s))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

    next = *limiter;

    for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
    {
        double low = (double)model->joint_min_positions.value[j];
        double high = (double)model->joint_max_positions.value[j];
        double measured_raw = (double)measured->value[j];
        double base_raw = next.initialized ?
            next.command_raw[j] : measured_raw;
        double last_v = next.initialized ?
            next.velocity_deg_s[j] : 0.0;
        double vmax = (double)max_velocity_deg_s->value[j];
        double amax = (double)max_acceleration_deg_s2->value[j];
        double requested_raw, requested_velocity;
        double lower_velocity, upper_velocity;
        double constrained_velocity;
        double next_raw, actual_velocity;
        long long rounded;

        if (!(low < high) || !isfinite(low) || !isfinite(high) ||
            measured_raw < low || measured_raw > high ||
            !(vmax > 0.0) || !isfinite(vmax) ||
            !(amax > 0.0) || !isfinite(amax) ||
            !isfinite((double)corrections->value[j]) ||
            !isfinite(base_raw) || !isfinite(last_v))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

        requested_raw = measured_raw +
            (double)corrections->value[j] /
            (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
        requested_velocity =
            (requested_raw - base_raw) *
            (double)ROBOT_JOINT_ANGLE_UNIT_DEG / dt_s;

        if (!isfinite(requested_velocity))
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

        /* Intersect acceleration bounds and absolute velocity bounds. */
        lower_velocity = fmax(-vmax, last_v - amax * dt_s);
        upper_velocity = fmin(vmax, last_v + amax * dt_s);
        if (!isfinite(lower_velocity) || !isfinite(upper_velocity) ||
            lower_velocity > upper_velocity)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

        constrained_velocity = clamp_double(
            requested_velocity, lower_velocity, upper_velocity);
        next_raw = base_raw + constrained_velocity * dt_s /
            (double)ROBOT_JOINT_ANGLE_UNIT_DEG;
        next_raw = clamp_double(next_raw, low, high);
        actual_velocity = (next_raw - base_raw) *
            (double)ROBOT_JOINT_ANGLE_UNIT_DEG / dt_s;

        if (!isfinite(next_raw) || !isfinite(actual_velocity) ||
            next_raw < (double)INT32_MIN ||
            next_raw > (double)INT32_MAX)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;

        /*
         * Preserve double-precision fractional command internally.
         * Only the published driver boundary is quantized to 0.01 deg.
         */
        rounded = llround(next_raw);
        if ((double)rounded > high) rounded = (long long)high;
        if ((double)rounded < low) rounded = (long long)low;
        positions.value[j] = (robot_joint_position_t)rounded;
        next.command_raw[j] = next_raw;
        next.velocity_deg_s[j] = actual_velocity;
    }

    next.initialized = 1U;
    *limiter = next;
    *out = positions;
    return ROBOT_STATUS_OK;
}
