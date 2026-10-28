/*
 * 文件：kinematics_inverse.c
 *
 * 用途：
 * 实现 UR5 Public Analytic IK Orchestration。
 *
 * 数学公式本身位于：
 *
 * ur5_analytic_ik.c
 *
 * 本文件负责：
 *
 * 1. Public -> DH Transform；
 * 2. Branch Enumeration；
 * 3. Singular Branch Handling；
 * 4. Solution Assembly；
 * 5. Duplicate Removal；
 * 6. Public Result。
 */

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "kinematics.h"
#include "kinematics_private.h"

#include "robot_model.h"


static robot_status_t kinematics_inverse_implementation(
    const robot_transform_t *transform,
    const robot_joint_angles_t *reference,
    kinematics_ik_solutions_t *solutions
)
{
    const robot_model_t *model;

    robot_transform_t dh_transform;

    robot_real_t q1_candidates_rad[2];

    robot_real_t q5_candidates_rad[2][2];

    robot_real_t q6_candidates_rad[2][2];

    robot_real_t q3_candidates_rad[2][2][2];

    robot_real_t q2_candidates_rad[2][2][2];

    uint8_t q5_branch_valid[2] =
    {
        0U,
        0U
    };

    uint8_t q6_branch_valid[2][2] =
    {
        {
            0U,
            0U
        },

        {
            0U,
            0U
        }
    };

    uint8_t q3_pair_valid[2][2] =
    {
        {
            0U,
            0U
        },

        {
            0U,
            0U
        }
    };

    uint8_t q2_branch_valid[2][2][2] =
    {
        {
            {
                0U,
                0U
            },

            {
                0U,
                0U
            }
        },

        {
            {
                0U,
                0U
            },

            {
                0U,
                0U
            }
        }
    };

    /* Set only when a near-singular q6 branch uses a close reference. */
    uint8_t q6_conditioned[2][2] = {{0U, 0U}, {0U, 0U}};
    robot_real_t reference_q1_rad = 0.0F;
    robot_real_t reference_q5_rad = 0.0F;
    robot_real_t reference_q6_rad = 0.0F;

    uint32_t shoulder_index;
    uint32_t wrist_index;
    uint32_t elbow_index;

    uint32_t singular_branch_count = 0U;

    robot_status_t status;

    if (
        transform == NULL
        || solutions == NULL
    )
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    solutions->count = 0U;

    if (reference != NULL)
    {
        uint32_t joint;
        for (joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint)
            if (reference->value[joint] < ROBOT_JOINT_ANGLE_MIN_RAW ||
                reference->value[joint] > ROBOT_JOINT_ANGLE_MAX_RAW)
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;

        reference_q1_rad =
            (robot_real_t)reference->value[0] *
            ROBOT_JOINT_ANGLE_UNIT_DEG * KINEMATICS_DEG_TO_RAD_F;
        reference_q5_rad =
            (robot_real_t)reference->value[4] *
            ROBOT_JOINT_ANGLE_UNIT_DEG * KINEMATICS_DEG_TO_RAD_F;
        reference_q6_rad =
            (robot_real_t)reference->value[5] *
            ROBOT_JOINT_ANGLE_UNIT_DEG * KINEMATICS_DEG_TO_RAD_F;
    }

    model = robot_model_get_active();

    /* =====================================================
     * Public T_base_ee
     * ->
     * Standard DH T_0_6
     * ===================================================== */

    status = kinematics_public_to_dh_transform(
        model,
        transform,
        &dh_transform
    );

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    /* =====================================================
     * Stage 1
     * q1 × 2
     * ===================================================== */

    status = ur5_analytic_ik_solve_q1_candidates(
        model,
        &dh_transform,
        q1_candidates_rad
    );

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    /* =====================================================
     * Stage 2
     * q5 × 2
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        status = ur5_analytic_ik_solve_q5_candidates(
            model,
            &dh_transform,
            q1_candidates_rad[shoulder_index],
            q5_candidates_rad[shoulder_index]
        );

        if (status == ROBOT_STATUS_ERROR_NO_SOLUTION)
        {
            continue;
        }

        if (status != ROBOT_STATUS_OK)
        {
            return status;
        }

        q5_branch_valid[shoulder_index] = 1U;
    }

    /* =====================================================
     * Stage 3
     * q6
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        if (!q5_branch_valid[shoulder_index])
        {
            continue;
        }

        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            status = ur5_analytic_ik_solve_q6(
                model,
                &dh_transform,
                q1_candidates_rad[shoulder_index],
                q5_candidates_rad[
                    shoulder_index
                ][
                    wrist_index
                ],
                &q6_candidates_rad[
                    shoulder_index
                ][
                    wrist_index
                ]
            );

            if (status == ROBOT_STATUS_ERROR_SINGULAR)
            {
                singular_branch_count++;
                continue;
            }

            if (status != ROBOT_STATUS_OK)
            {
                return status;
            }

            /*
             * Reference-guided recovery is intentionally VERY narrow.
             *
             * At |sin(q5)| << 1 the two orientation projections used by
             * atan2(q6) are comparable to the float pose rounding error.
             * Only substitute the nearby reference on a matching shoulder
             * and wrist-sign branch. Never substitute an arbitrary target
             * J6 (there is no such known target in Cartesian IK).
             *
             * We also check each substituted candidate's true production FK
             * pose below, after 0.01 degree quantization; incompatible
             * references must never manufacture a valid IK candidate.
             */
            if (reference != NULL)
            {
                robot_real_t q1 =
                    q1_candidates_rad[shoulder_index];
                robot_real_t q5 =
                    q5_candidates_rad[shoulder_index][wrist_index];
                robot_real_t derived_q6 =
                    q6_candidates_rad[shoulder_index][wrist_index];
                robot_real_t shoulder_difference =
                    kinematics_normalize_angle_rad(
                        q1 - reference_q1_rad);
                robot_real_t wrist_sin = sinf(q5);
                robot_real_t q6_difference =
                    kinematics_normalize_angle_rad(
                        derived_q6 - reference_q6_rad);

                if (fabsf(wrist_sin) <
                        KINEMATICS_SEEDED_WRIST_SIN_THRESHOLD &&
                    fabsf(shoulder_difference) <
                        KINEMATICS_SEEDED_SHOULDER_MAX_ERROR_RAD &&
                    ((q5 > 0.0F && reference_q5_rad > 0.0F) ||
                     (q5 < 0.0F && reference_q5_rad < 0.0F)) &&
                    fabsf(q6_difference) <
                        KINEMATICS_SEEDED_Q6_MAX_ERROR_RAD)
                {
                    q6_candidates_rad[shoulder_index][wrist_index] =
                        reference_q6_rad;
                    q6_conditioned[shoulder_index][wrist_index] = 1U;
                }
            }

            q6_branch_valid[
                shoulder_index
            ][
                wrist_index
            ] = 1U;
        }
    }

    /* =====================================================
     * Stage 4
     * q3 × 2
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            if (
                !q6_branch_valid[
                    shoulder_index
                ][
                    wrist_index
                ]
            )
            {
                continue;
            }

            status = ur5_analytic_ik_solve_q3_candidates(
                model,
                &dh_transform,
                q1_candidates_rad[shoulder_index],
                q5_candidates_rad[
                    shoulder_index
                ][
                    wrist_index
                ],
                q6_candidates_rad[
                    shoulder_index
                ][
                    wrist_index
                ],
                q3_candidates_rad[
                    shoulder_index
                ][
                    wrist_index
                ]
            );

            if (status == ROBOT_STATUS_ERROR_NO_SOLUTION)
            {
                continue;
            }

            if (status != ROBOT_STATUS_OK)
            {
                return status;
            }

            q3_pair_valid[
                shoulder_index
            ][
                wrist_index
            ] = 1U;
        }
    }

    /* =====================================================
     * Stage 5
     * q2
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            if (
                !q3_pair_valid[
                    shoulder_index
                ][
                    wrist_index
                ]
            )
            {
                continue;
            }

            for (
                elbow_index = 0U;
                elbow_index < 2U;
                elbow_index++
            )
            {
                status = ur5_analytic_ik_solve_q2(
                    model,
                    &dh_transform,

                    q1_candidates_rad[
                        shoulder_index
                    ],

                    q5_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],

                    q6_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],

                    q3_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ],

                    &q2_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ]
                );

                if (status == ROBOT_STATUS_ERROR_SINGULAR)
                {
                    singular_branch_count++;
                    continue;
                }

                if (status != ROBOT_STATUS_OK)
                {
                    return status;
                }

                q2_branch_valid[
                    shoulder_index
                ][
                    wrist_index
                ][
                    elbow_index
                ] = 1U;
            }
        }
    }

    /* =====================================================
     * Stage 6
     *
     * q4
     *
     * +
     *
     * Complete Solution Assembly
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            for (
                elbow_index = 0U;
                elbow_index < 2U;
                elbow_index++
            )
            {
                robot_real_t q4_rad;

                robot_joint_angles_t candidate;

                if (
                    !q2_branch_valid[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ]
                )
                {
                    continue;
                }

                status = ur5_analytic_ik_solve_q4(
                    model,
                    &dh_transform,

                    q1_candidates_rad[
                        shoulder_index
                    ],

                    q2_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ],

                    q3_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ],

                    q5_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],

                    q6_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],

                    &q4_rad
                );

                if (status == ROBOT_STATUS_ERROR_SINGULAR)
                {
                    singular_branch_count++;
                    continue;
                }

                if (status != ROBOT_STATUS_OK)
                {
                    return status;
                }

                kinematics_build_joint_solution(
                    q1_candidates_rad[
                        shoulder_index
                    ],

                    q2_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ],

                    q3_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ],

                    q4_rad,

                    q5_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],

                    q6_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],

                    &candidate
                );

                if (q6_conditioned[shoulder_index][wrist_index] != 0U)
                {
                    /* Quantized candidate must reconstruct the REQUESTED
                     * pose. The finite-safe residual gate is shared with
                     * the Cartesian trajectory consumer. */
                    robot_transform_t verified={{{0}}};
                    status=kinematics_forward(&candidate,&verified);
                    if(status!=ROBOT_STATUS_OK)
                        continue;
                    status=kinematics_pose_within_tolerance(
                        &verified,transform,
                        KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM,
                        KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL);
                    if(status!=ROBOT_STATUS_OK)
                        continue;
                }

                status = kinematics_add_solution_if_unique(
                    solutions,
                    &candidate
                );

                if (status != ROBOT_STATUS_OK)
                {
                    return status;
                }
            }
        }
    }

    /* =====================================================
     * Final Result
     * ===================================================== */

    if (solutions->count > 0U)
    {
        return ROBOT_STATUS_OK;
    }

    if (singular_branch_count > 0U)
    {
        return ROBOT_STATUS_ERROR_SINGULAR;
    }

    return ROBOT_STATUS_ERROR_NO_SOLUTION;
}
/* Preserve legacy call behavior: no implicit reference state. */
robot_status_t kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions)
{
    return kinematics_inverse_implementation(transform, NULL, solutions);
}

/* The caller supplies current canonical joints; the solver never caches
 * a global pose or assumes the generating joint angles are available.
 */
robot_status_t kinematics_inverse_with_reference(
    const robot_transform_t *transform,
    const robot_joint_angles_t *reference,
    kinematics_ik_solutions_t *solutions)
{
    if (reference == NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    return kinematics_inverse_implementation(transform, reference, solutions);
}
