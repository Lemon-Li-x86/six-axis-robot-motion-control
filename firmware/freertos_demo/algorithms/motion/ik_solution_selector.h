
/*
 * File: ik_solution_selector.h
 *
 * Stage 3: IK Candidate Selection
 *
 * Responsibilities:
 * 1. Filter candidates using robot joint limits.
 * 2. Convert canonical angles to continuous positions.
 * 3. Evaluate joint travel.
 * 4. Penalize proximity to joint limits.
 * 5. Detect selected elbow and wrist singularity risks.
 * 6. Return the preferred continuous joint target.
 *
 * This module does NOT:
 * - Generate a trajectory;
 * - Control motors;
 * - Access FreeRTOS;
 * - Perform collision detection.
 */

#ifndef IK_SOLUTION_SELECTOR_H
#define IK_SOLUTION_SELECTOR_H

#include <stdint.h>

#include "kinematics.h"
#include "robot_types.h"
#include "error_code.h"


typedef struct
{
    /*
     * Selected continuous joint positions.
     *
     * Unit: 0.01 degree.
     * Unlike canonical angles, these do not wrap at 180 deg.
     */
    robot_joint_positions_t target_positions;

    /*
     * Original candidate index in the IK solution array.
     */
    uint8_t candidate_index;

    /*
     * Sum of absolute travel over all six joints.
     *
     * Unit: 0.01 degree.
     */
    uint32_t total_travel_raw;

    /*
     * Minimum distance to any joint position limit.
     *
     * Unit: 0.01 degree.
     */
    robot_joint_position_t minimum_limit_margin_raw;

    /*
     * Dimensionless weighted selection score.
     * Lower is better.
     */
    robot_real_t score;

} ik_selection_result_t;


/*
 * Select the preferred IK solution.
 *
 * Inputs:
 *
 * solutions:
 *   Up to 8 canonical IK solutions,
 *   normally returned by kinematics_inverse().
 *
 * current_positions:
 *   Current six-axis continuous joint positions.
 *
 * result:
 *   Selected result, written only on success.
 *
 * Selection policy:
 *
 * 1. Reject invalid input;
 * 2. Unwrap every candidate into legal continuous positions;
 * 3. Reject configurations close to selected singularities;
 * 4. Evaluate normalized joint travel;
 * 5. Apply joint-limit proximity penalty;
 * 6. Apply singularity proximity penalty;
 * 7. Select lowest score.
 *
 * Current singularity screening covers:
 * - Elbow straight/folded configurations (q3);
 * - Wrist alignment configurations (q5).
 *
 * It does not yet implement a full Jacobian singular-value
 * analysis or independent shoulder-singularity screening.
 *
 * Returns:
 *
 * ROBOT_STATUS_OK
 * ROBOT_STATUS_ERROR_NULL_POINTER
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE
 * ROBOT_STATUS_ERROR_NO_SOLUTION
 * ROBOT_STATUS_ERROR_SINGULAR
 * ROBOT_STATUS_ERROR_INTERNAL
 *
 * On failure, result remains unchanged.
 */
robot_status_t ik_solution_selector_select(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_positions_t *current_positions,
    ik_selection_result_t *result
);

#endif
