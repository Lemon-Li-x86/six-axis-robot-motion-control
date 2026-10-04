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

#include "kinematics.h"
#include "kinematics_private.h"

#include "robot_model.h"


robot_status_t
kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions
)
{
    const robot_model_t
        *model;


    robot_transform_t
        dh_transform;


    robot_real_t
        q1_candidates_rad[2];


    robot_real_t
        q5_candidates_rad[2][2];


    robot_real_t
        q6_candidates_rad[2][2];


    robot_real_t
        q3_candidates_rad[2][2][2];


    robot_real_t
        q2_candidates_rad[2][2][2];


    uint8_t
        q5_branch_valid[2] =
        {
            0U,
            0U
        };


    uint8_t
        q6_branch_valid[2][2] =
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


    uint8_t
        q3_pair_valid[2][2] =
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


    uint8_t
        q2_branch_valid[2][2][2] =
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


    uint32_t
        shoulder_index;

    uint32_t
        wrist_index;

    uint32_t
        elbow_index;


    uint32_t
        singular_branch_count =
            0U;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        solutions
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    solutions->count =
        0U;


    model =
        robot_model_get_active();


    /* =====================================================
     * Public T_base_ee
     * ->
     * Standard DH T_0_6
     * ===================================================== */

    status =
        kinematics_public_to_dh_transform(
            model,
            transform,
            &dh_transform
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    /* =====================================================
     * Stage 1
     * q1 × 2
     * ===================================================== */

    status =
        ur5_analytic_ik_solve_q1_candidates(
            model,
            &dh_transform,
            q1_candidates_rad
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
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
        status =
            ur5_analytic_ik_solve_q5_candidates(
                model,
                &dh_transform,
                q1_candidates_rad[
                    shoulder_index
                ],
                q5_candidates_rad[
                    shoulder_index
                ]
            );


        if (
            status
            ==
            ROBOT_STATUS_ERROR_NO_SOLUTION
        )
        {
            continue;
        }


        if (
            status
            !=
            ROBOT_STATUS_OK
        )
        {
            return
                status;
        }


        q5_branch_valid[
            shoulder_index
        ] =
            1U;
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
        if (
            !q5_branch_valid[
                shoulder_index
            ]
        )
        {
            continue;
        }


        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            status =
                ur5_analytic_ik_solve_q6(
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
                    &q6_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ]
                );


            if (
                status
                ==
                ROBOT_STATUS_ERROR_SINGULAR
            )
            {
                singular_branch_count++;

                continue;
            }


            if (
                status
                !=
                ROBOT_STATUS_OK
            )
            {
                return
                    status;
            }


            q6_branch_valid[
                shoulder_index
            ][
                wrist_index
            ] =
                1U;
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


            status =
                ur5_analytic_ik_solve_q3_candidates(
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
                    ]
                );


            if (
                status
                ==
                ROBOT_STATUS_ERROR_NO_SOLUTION
            )
            {
                continue;
            }


            if (
                status
                !=
                ROBOT_STATUS_OK
            )
            {
                return
                    status;
            }


            q3_pair_valid[
                shoulder_index
            ][
                wrist_index
            ] =
                1U;
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
                status =
                    ur5_analytic_ik_solve_q2(
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


                if (
                    status
                    ==
                    ROBOT_STATUS_ERROR_SINGULAR
                )
                {
                    singular_branch_count++;

                    continue;
                }


                if (
                    status
                    !=
                    ROBOT_STATUS_OK
                )
                {
                    return
                        status;
                }


                q2_branch_valid[
                    shoulder_index
                ][
                    wrist_index
                ][
                    elbow_index
                ] =
                    1U;
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
                robot_real_t
                    q4_rad;


                robot_joint_angles_t
                    candidate;


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


                status =
                    ur5_analytic_ik_solve_q4(
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


                if (
                    status
                    ==
                    ROBOT_STATUS_ERROR_SINGULAR
                )
                {
                    singular_branch_count++;

                    continue;
                }


                if (
                    status
                    !=
                    ROBOT_STATUS_OK
                )
                {
                    return
                        status;
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


                status =
                    kinematics_add_solution_if_unique(
                        solutions,
                        &candidate
                    );


                if (
                    status
                    !=
                    ROBOT_STATUS_OK
                )
                {
                    return
                        status;
                }
            }
        }
    }


    /* =====================================================
     * Final Result
     * ===================================================== */

    if (
        solutions->count
        >
        0U
    )
    {
        return
            ROBOT_STATUS_OK;
    }


    if (
        singular_branch_count
        >
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_SINGULAR;
    }


    return
        ROBOT_STATUS_ERROR_NO_SOLUTION;
}


/* =========================================================
 * Legacy Solution Selection Placeholder
 * ========================================================= */

robot_status_t
kinematics_select_best_solution(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *reference,
    robot_joint_angles_t *selected
)
{
    if (
        solutions
        ==
        NULL
        ||
        reference
        ==
        NULL
        ||
        selected
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * 不在这里继续实现。
     *
     * 最优逆解属于 Motion Policy。
     *
     * 后续需要使用：
     *
     * robot_joint_positions_t
     *
     * 即 Continuous Position，
     * 而不是 Canonical Angle。
     */
    return
        ROBOT_STATUS_ERROR_NOT_IMPLEMENTED;
}