
/*
 * File: ik_solution_selector.c
 *
 * UR5 IK candidate selection policy.
 *
 * Pure algorithm module:
 * no FreeRTOS, UART, motor driver or application state.
 */

#include <stddef.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include "ik_solution_selector.h"
#include "motion_limits.h"
#include "robot_model.h"


/* =========================================================
 * Configuration
 * ========================================================= */

#define IK_SELECTOR_DEG_TO_RAD \
    (0.01745329251994329577F)

/*
 * Hard singularity threshold:
 * reject candidates when |sin(q)| < 0.01.
 *
 * Applies to q3 (elbow) and q5 (wrist).
 */
#define IK_SELECTOR_HARD_SINGULAR_SIN \
    (0.01F)

/*
 * Soft singularity proximity threshold.
 */
#define IK_SELECTOR_SOFT_SINGULAR_SIN \
    (0.25F)

/*
 * A limit margin smaller than 10% of the
 * corresponding joint range receives a penalty.
 */
#define IK_SELECTOR_SOFT_LIMIT_FRACTION \
    (0.10F)

/*
 * Selection score weights.
 */
#define IK_SELECTOR_LIMIT_WEIGHT \
    (0.35F)

#define IK_SELECTOR_SINGULAR_WEIGHT \
    (0.50F)

/*
 * Floating-point comparison tolerance.
 */
#define IK_SELECTOR_SCORE_TIE_EPS \
    (0.00001F)


/* =========================================================
 * Helpers
 * ========================================================= */

static robot_real_t soft_penalty(
    robot_real_t value,
    robot_real_t threshold
)
{
    if (value >= threshold)
    {
        return 0.0F;
    }

    return (threshold - value) / threshold;
}


static uint32_t abs_difference(
    robot_joint_position_t lhs,
    robot_joint_position_t rhs
)
{
    int64_t delta;

    delta = (int64_t)lhs - (int64_t)rhs;

    if (delta < 0)
    {
        delta = -delta;
    }

    return (uint32_t)delta;
}


/* =========================================================
 * Input Validation
 * ========================================================= */

static robot_status_t validate_inputs(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_positions_t *current_positions,
    const robot_model_t *model
)
{
    uint32_t i;
    uint32_t j;

    if (model == NULL)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    for (j = 0U; j < ROBOT_JOINT_COUNT; j++)
    {
        if (
            model->joint_max_positions.value[j]
            <= model->joint_min_positions.value[j]
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }
    }

    if (
        solutions->count
        > KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }

    if (solutions->count == 0U)
    {
        return ROBOT_STATUS_ERROR_NO_SOLUTION;
    }

    /*
     * Reject malformed candidates rather than
     * silently normalizing invalid input.
     */
    for (i = 0U; i < solutions->count; i++)
    {
        for (j = 0U; j < ROBOT_JOINT_COUNT; j++)
        {
            int32_t angle;

            angle = solutions->solutions[i].value[j];

            if (
                angle < ROBOT_JOINT_ANGLE_MIN_RAW
                || angle > ROBOT_JOINT_ANGLE_MAX_RAW
            )
            {
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            }
        }
    }

    return motion_limits_check_positions(
        current_positions
    );
}


/* =========================================================
 * Singularity Risk Estimation
 * ========================================================= */

/*
 * Current heuristic:
 *
 * - q3 approaches 0 or +/-180 degrees:
 *   elbow singularity risk.
 *
 * - q5 approaches 0 or +/-180 degrees:
 *   wrist singularity risk.
 *
 * This is not a complete UR5 Jacobian condition-number
 * estimator. Shoulder singularity analysis remains
 * a separate extension.
 */

static robot_real_t singular_penalty(
    const robot_joint_angles_t *angles,
    uint8_t *near_singular
)
{
    robot_real_t elbow_rad;
    robot_real_t wrist_rad;
    robot_real_t elbow_sin;
    robot_real_t wrist_sin;

    elbow_rad =
        (robot_real_t)angles->value[2]
        * ROBOT_JOINT_ANGLE_UNIT_DEG
        * IK_SELECTOR_DEG_TO_RAD;

    wrist_rad =
        (robot_real_t)angles->value[4]
        * ROBOT_JOINT_ANGLE_UNIT_DEG
        * IK_SELECTOR_DEG_TO_RAD;

    elbow_sin = fabsf(sinf(elbow_rad));
    wrist_sin = fabsf(sinf(wrist_rad));

    *near_singular = (uint8_t)(
        elbow_sin < IK_SELECTOR_HARD_SINGULAR_SIN
        || wrist_sin < IK_SELECTOR_HARD_SINGULAR_SIN
    );

    return
        soft_penalty(
            elbow_sin,
            IK_SELECTOR_SOFT_SINGULAR_SIN
        )
        +
        soft_penalty(
            wrist_sin,
            IK_SELECTOR_SOFT_SINGULAR_SIN
        );
}


/* =========================================================
 * Public Selection API
 * ========================================================= */

robot_status_t ik_solution_selector_select(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_positions_t *current_positions,
    ik_selection_result_t *result
)
{
    const robot_model_t *model;

    robot_status_t status;

    ik_selection_result_t best = {0};

    uint32_t i;

    uint8_t have_best = 0U;
    uint8_t have_limit_feasible = 0U;

    if (
        solutions == NULL
        || current_positions == NULL
        || result == NULL
    )
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    model = robot_model_get_active();

    status = validate_inputs(
        solutions,
        current_positions,
        model
    );

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    /*
     * Evaluate all IK candidates.
     */
    for (i = 0U; i < solutions->count; i++)
    {
        robot_joint_positions_t target = {{0}};

        robot_real_t travel_cost = 0.0F;
        robot_real_t limit_cost = 0.0F;
        robot_real_t singular_cost;
        robot_real_t score;

        robot_joint_position_t min_margin = INT32_MAX;

        uint32_t travel = 0U;
        uint32_t j;

        uint8_t near_singular = 0U;

        /*
         * 1. Convert canonical angles to the nearest
         *    legal continuous joint positions.
         */
        status = motion_limits_unwrap_nearest(
            &solutions->solutions[i],
            current_positions,
            &target
        );

        if (status == ROBOT_STATUS_ERROR_NO_SOLUTION)
        {
            continue;
        }

        if (status != ROBOT_STATUS_OK)
        {
            return status;
        }

        have_limit_feasible = 1U;

        /*
         * 2. Evaluate singularity risk.
         */
        singular_cost = singular_penalty(
            &solutions->solutions[i],
            &near_singular
        );

        if (near_singular != 0U)
        {
            continue;
        }

        /*
         * 3. Evaluate motion and joint-limit margins.
         */
        for (j = 0U; j < ROBOT_JOINT_COUNT; j++)
        {
            int64_t lower;
            int64_t upper;
            int64_t value;

            int64_t left;
            int64_t right;

            robot_joint_position_t margin;

            robot_real_t span;

            uint32_t movement;

            lower =
                model->joint_min_positions.value[j];

            upper =
                model->joint_max_positions.value[j];

            value =
                target.value[j];

            left = value - lower;
            right = upper - value;

            margin = (robot_joint_position_t)(
                left < right ? left : right
            );

            span = (robot_real_t)(upper - lower);

            movement = abs_difference(
                target.value[j],
                current_positions->value[j]
            );

            travel += movement;

            /*
             * Normalize each joint's movement
             * by its allowed travel range.
             */
            travel_cost +=
                (robot_real_t)movement / span;

            /*
             * Penalize candidates near joint limits.
             */
            limit_cost += soft_penalty(
                (robot_real_t)margin / span,
                IK_SELECTOR_SOFT_LIMIT_FRACTION
            );

            if (margin < min_margin)
            {
                min_margin = margin;
            }
        }

        /*
         * 4. Weighted cost.
         */
        score =
            travel_cost
            + IK_SELECTOR_LIMIT_WEIGHT * limit_cost
            + IK_SELECTOR_SINGULAR_WEIGHT * singular_cost;

        /*
         * 5. Select the lowest-score candidate.
         *
         * Tie-breaking:
         * - Prefer smaller total joint travel.
         * - Then retain the earlier candidate.
         */
        if (
            have_best == 0U
            || score < best.score - IK_SELECTOR_SCORE_TIE_EPS
            || (
                fabsf(score - best.score)
                    <= IK_SELECTOR_SCORE_TIE_EPS
                && travel < best.total_travel_raw
            )
        )
        {
            best.target_positions = target;

            best.candidate_index = (uint8_t)i;

            best.total_travel_raw = travel;

            best.minimum_limit_margin_raw = min_margin;

            best.score = score;

            have_best = 1U;
        }
    }

    /*
     * No acceptable candidate.
     */
    if (have_best == 0U)
    {
        if (have_limit_feasible != 0U)
        {
            return ROBOT_STATUS_ERROR_SINGULAR;
        }

        return ROBOT_STATUS_ERROR_NO_SOLUTION;
    }

    /*
     * Commit output only after success.
     */
    *result = best;

    return ROBOT_STATUS_OK;
}
