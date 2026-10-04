#include <stdint.h>

#include "kinematics_self_test.h"

#include "kinematics.h"
#include "kinematics_internal.h"

#include "robot_types.h"


/* =========================================================
 * Test Tolerance
 * ========================================================= */

#define KINEMATICS_TEST_ROTATION_TOLERANCE \
    0.001F

#define KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM \
    0.05F

#define KINEMATICS_TEST_ANGLE_TOLERANCE_RAD \
    0.001F


/* =========================================================
 * Known Angles
 * ========================================================= */

#define KINEMATICS_TEST_Q1_ZERO_BRANCH_A_RAD \
    (-2.8760488F)

#define KINEMATICS_TEST_Q1_ZERO_BRANCH_B_RAD \
    0.0F


#define KINEMATICS_TEST_Q1_60_BRANCH_A_RAD \
    (-1.8288512F)

#define KINEMATICS_TEST_Q1_60_BRANCH_B_RAD \
    1.0471976F


#define KINEMATICS_TEST_Q1_30_RAD \
    0.5235988F


#define KINEMATICS_TEST_Q5_POSITIVE_40_RAD \
    0.6981317F

#define KINEMATICS_TEST_Q5_NEGATIVE_40_RAD \
    (-0.6981317F)


#define KINEMATICS_TEST_Q6_NEGATIVE_10_RAD \
    (-0.1745329F)

#define KINEMATICS_TEST_Q6_POSITIVE_170_RAD \
    2.9670597F


#define KINEMATICS_TEST_Q3_POSITIVE_60_RAD \
    1.0471976F

#define KINEMATICS_TEST_Q3_NEGATIVE_60_RAD \
    (-1.0471976F)


#define KINEMATICS_TEST_Q3_FLIP_POSITIVE_RAD \
    1.5909281F

#define KINEMATICS_TEST_Q3_FLIP_NEGATIVE_RAD \
    (-1.5909281F)


#define KINEMATICS_TEST_Q2_ORIGINAL_POSITIVE_ELBOW_RAD \
    (-0.7853982F)

#define KINEMATICS_TEST_Q2_ORIGINAL_NEGATIVE_ELBOW_RAD \
    0.2155343F


#define KINEMATICS_TEST_Q2_FLIP_POSITIVE_ELBOW_RAD \
    (-0.8319115F)

#define KINEMATICS_TEST_Q2_FLIP_NEGATIVE_ELBOW_RAD \
    0.6777868F


#define KINEMATICS_TEST_Q4_ORIGINAL_POSITIVE_ELBOW_RAD \
    0.3490659F

#define KINEMATICS_TEST_Q4_ORIGINAL_NEGATIVE_ELBOW_RAD \
    1.4425279F


#define KINEMATICS_TEST_Q4_FLIP_POSITIVE_ELBOW_RAD \
    2.9929408F

#define KINEMATICS_TEST_Q4_FLIP_NEGATIVE_ELBOW_RAD \
    (-1.6175851F)


/* =========================================================
 * Helpers
 * ========================================================= */

static robot_real_t
test_abs(
    robot_real_t value
)
{
    if (
        value
        <
        0.0F
    )
    {
        return
            -value;
    }


    return
        value;
}


static uint8_t
scalar_is_close(
    robot_real_t actual,
    robot_real_t expected,
    robot_real_t tolerance
)
{
    if (
        test_abs(
            actual
            -
            expected
        )
        >
        tolerance
    )
    {
        return
            0U;
    }


    return
        1U;
}


/* =========================================================
 * Joint Compare
 * ========================================================= */

static uint8_t
joint_angles_are_equal(
    const robot_joint_angles_t *left,
    const robot_joint_angles_t *right
)
{
    uint32_t
        joint_index;


    if (
        left
        ==
        NULL
        ||
        right
        ==
        NULL
    )
    {
        return
            0U;
    }


    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        if (
            left->value[
                joint_index
            ]
            !=
            right->value[
                joint_index
            ]
        )
        {
            return
                0U;
        }
    }


    return
        1U;
}


/* =========================================================
 * Transform Compare
 * ========================================================= */

static uint8_t
transform_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
)
{
    uint32_t
        row;

    uint32_t
        column;


    for (
        row = 0U;
        row < 4U;
        row++
    )
    {
        for (
            column = 0U;
            column < 4U;
            column++
        )
        {
            robot_real_t
                tolerance;


            robot_real_t
                difference;


            if (
                column
                ==
                3U
                &&
                row
                <
                3U
            )
            {
                tolerance =
                    KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM;
            }
            else
            {
                tolerance =
                    KINEMATICS_TEST_ROTATION_TOLERANCE;
            }


            difference =
                test_abs(
                    actual->matrix[
                        row
                    ][
                        column
                    ]
                    -
                    expected->matrix[
                        row
                    ][
                        column
                    ]
                );


            if (
                difference
                >
                tolerance
            )
            {
                return
                    0U;
            }
        }
    }


    return
        1U;
}


/* =========================================================
 * FK Test
 * ========================================================= */

static robot_status_t
run_fk_test_case(
    const robot_joint_angles_t *joints,
    const robot_transform_t *expected
)
{
    robot_transform_t
        actual;


    if (
        kinematics_forward(
            joints,
            &actual
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !transform_is_close(
            &actual,
            expected
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * q1 Test
 * ========================================================= */

static robot_status_t
run_q1_test_case(
    const robot_transform_t *transform,
    robot_real_t expected_q1_a_rad,
    robot_real_t expected_q1_b_rad
)
{
    robot_real_t
        q1_candidates_rad[2];


    if (
        kinematics_internal_solve_q1_candidates(
            transform,
            q1_candidates_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q1_candidates_rad[0],
            expected_q1_a_rad,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q1_candidates_rad[1],
            expected_q1_b_rad,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Partial Analytic IK Chain
 * ========================================================= */

static robot_status_t
run_partial_ik_chain_test(
    const robot_joint_angles_t *joints
)
{
    robot_transform_t
        transform;


    robot_real_t
        q1_candidates_rad[2];

    robot_real_t
        selected_q1_rad;


    robot_real_t
        q5_candidates_rad[2];


    robot_real_t
        q6_original_rad;

    robot_real_t
        q6_flip_rad;


    robot_real_t
        q3_original_rad[2];

    robot_real_t
        q3_flip_rad[2];


    robot_real_t
        q2_original_positive_rad;

    robot_real_t
        q2_original_negative_rad;


    robot_real_t
        q2_flip_positive_rad;

    robot_real_t
        q2_flip_negative_rad;


    robot_real_t
        q4_original_positive_rad;

    robot_real_t
        q4_original_negative_rad;


    robot_real_t
        q4_flip_positive_rad;

    robot_real_t
        q4_flip_negative_rad;


    robot_real_t
        difference_0;

    robot_real_t
        difference_1;


    if (
        kinematics_forward(
            joints,
            &transform
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* q1 */

    if (
        kinematics_internal_solve_q1_candidates(
            &transform,
            q1_candidates_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    difference_0 =
        test_abs(
            q1_candidates_rad[0]
            -
            KINEMATICS_TEST_Q1_30_RAD
        );


    difference_1 =
        test_abs(
            q1_candidates_rad[1]
            -
            KINEMATICS_TEST_Q1_30_RAD
        );


    if (
        difference_0
        <
        difference_1
    )
    {
        selected_q1_rad =
            q1_candidates_rad[0];
    }
    else
    {
        selected_q1_rad =
            q1_candidates_rad[1];
    }


    if (
        !scalar_is_close(
            selected_q1_rad,
            KINEMATICS_TEST_Q1_30_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* q5 */

    if (
        kinematics_internal_solve_q5_candidates(
            &transform,
            selected_q1_rad,
            q5_candidates_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q5_candidates_rad[0],
            KINEMATICS_TEST_Q5_POSITIVE_40_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q5_candidates_rad[1],
            KINEMATICS_TEST_Q5_NEGATIVE_40_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* q6 */

    if (
        kinematics_internal_solve_q6(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[0],
            &q6_original_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q6_original_rad,
            KINEMATICS_TEST_Q6_NEGATIVE_10_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q6(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[1],
            &q6_flip_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q6_flip_rad,
            KINEMATICS_TEST_Q6_POSITIVE_170_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* q3 */

    if (
        kinematics_internal_solve_q3_candidates(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[0],
            q6_original_rad,
            q3_original_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q3_original_rad[0],
            KINEMATICS_TEST_Q3_POSITIVE_60_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q3_original_rad[1],
            KINEMATICS_TEST_Q3_NEGATIVE_60_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q3_candidates(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[1],
            q6_flip_rad,
            q3_flip_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q3_flip_rad[0],
            KINEMATICS_TEST_Q3_FLIP_POSITIVE_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q3_flip_rad[1],
            KINEMATICS_TEST_Q3_FLIP_NEGATIVE_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* q2 */

    if (
        kinematics_internal_solve_q2(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[0],
            q6_original_rad,
            q3_original_rad[0],
            &q2_original_positive_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q2_original_positive_rad,
            KINEMATICS_TEST_Q2_ORIGINAL_POSITIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q2(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[0],
            q6_original_rad,
            q3_original_rad[1],
            &q2_original_negative_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q2_original_negative_rad,
            KINEMATICS_TEST_Q2_ORIGINAL_NEGATIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q2(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[1],
            q6_flip_rad,
            q3_flip_rad[0],
            &q2_flip_positive_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q2_flip_positive_rad,
            KINEMATICS_TEST_Q2_FLIP_POSITIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q2(
            &transform,
            selected_q1_rad,
            q5_candidates_rad[1],
            q6_flip_rad,
            q3_flip_rad[1],
            &q2_flip_negative_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q2_flip_negative_rad,
            KINEMATICS_TEST_Q2_FLIP_NEGATIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* q4 */

    if (
        kinematics_internal_solve_q4(
            &transform,
            selected_q1_rad,
            q2_original_positive_rad,
            q3_original_rad[0],
            q5_candidates_rad[0],
            q6_original_rad,
            &q4_original_positive_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q4_original_positive_rad,
            KINEMATICS_TEST_Q4_ORIGINAL_POSITIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q4(
            &transform,
            selected_q1_rad,
            q2_original_negative_rad,
            q3_original_rad[1],
            q5_candidates_rad[0],
            q6_original_rad,
            &q4_original_negative_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q4_original_negative_rad,
            KINEMATICS_TEST_Q4_ORIGINAL_NEGATIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q4(
            &transform,
            selected_q1_rad,
            q2_flip_positive_rad,
            q3_flip_rad[0],
            q5_candidates_rad[1],
            q6_flip_rad,
            &q4_flip_positive_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q4_flip_positive_rad,
            KINEMATICS_TEST_Q4_FLIP_POSITIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_internal_solve_q4(
            &transform,
            selected_q1_rad,
            q2_flip_negative_rad,
            q3_flip_rad[1],
            q5_candidates_rad[1],
            q6_flip_rad,
            &q4_flip_negative_rad
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        !scalar_is_close(
            q4_flip_negative_rad,
            KINEMATICS_TEST_Q4_FLIP_NEGATIVE_ELBOW_RAD,
            KINEMATICS_TEST_ANGLE_TOLERANCE_RAD
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Public IK Solution Assembly Test
 * ========================================================= */

/**
 * @brief 验证公共 kinematics_inverse()。
 *
 * Known Joints：
 *
 * [30, -45, 60, 20, 40, -10]
 *
 * ->
 *
 * FK
 *
 * ->
 *
 * Target T_base_ee
 *
 * ->
 *
 * Public IK
 *
 * ->
 *
 * 1~8 Complete Solutions
 *
 * 并要求至少存在一组与原始
 * Canonical Joint Angles 完全一致。
 */
static robot_status_t
run_inverse_solution_assembly_test(
    const robot_joint_angles_t *original_joints
)
{
    robot_transform_t
        target_transform;


    kinematics_ik_solutions_t
        solutions;


    uint32_t
        solution_index;

    uint32_t
        joint_index;


    uint8_t
        original_solution_found =
            0U;


    robot_status_t
        status;


    if (
        original_joints
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Known Joints
     *
     * ->
     *
     * FK Target
     */
    status =
        kinematics_forward(
            original_joints,
            &target_transform
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Target
     *
     * ->
     *
     * Public IK
     */
    status =
        kinematics_inverse(
            &target_transform,
            &solutions
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 必须返回：
     *
     * 1 <= count <= 8
     */
    if (
        solutions.count
        ==
        0U
        ||
        solutions.count
        >
        KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 所有公共输出必须已经处于：
     *
     * [-18000, 17999]
     */
    for (
        solution_index = 0U;
        solution_index < solutions.count;
        solution_index++
    )
    {
        for (
            joint_index = 0U;
            joint_index < ROBOT_JOINT_COUNT;
            joint_index++
        )
        {
            int32_t
                raw;


            raw =
                (int32_t)
                solutions.solutions[
                    solution_index
                ].value[
                    joint_index
                ];


            if (
                raw
                <
                ROBOT_JOINT_ANGLE_MIN_RAW
                ||
                raw
                >
                ROBOT_JOINT_ANGLE_MAX_RAW
            )
            {
                return
                    ROBOT_STATUS_ERROR_INTERNAL;
            }
        }


        /*
         * 查找原始解。
         */
        if (
            joint_angles_are_equal(
                &solutions.solutions[
                    solution_index
                ],
                original_joints
            )
        )
        {
            original_solution_found =
                1U;
        }
    }


    /*
     * 对 FK 生成的 Target，
     * IK 必须至少找回原始 Joint Branch。
     */
    if (
        !original_solution_found
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Wrist Singularity Test
 * ========================================================= */

static robot_status_t
run_wrist_singularity_test(
    const robot_joint_angles_t *joints
)
{
    robot_transform_t
        transform;


    robot_real_t
        q6_rad;


    robot_status_t
        status;


    if (
        kinematics_forward(
            joints,
            &transform
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    status =
        kinematics_internal_solve_q6(
            &transform,
            0.0F,
            0.0F,
            &q6_rad
        );


    if (
        status
        !=
        ROBOT_STATUS_ERROR_SINGULAR
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Complete Self Test
 * ========================================================= */

robot_status_t
kinematics_self_test_run(void)
{
    /*
     * Zero Configuration
     */
    static const robot_joint_angles_t
        joints_zero =
    {
        .value =
        {
            0,
            0,
            0,
            0,
            0,
            0
        }
    };


    static const robot_transform_t
        expected_zero =
    {
        .matrix =
        {
            {
                0.0F,
                1.0F,
                0.0F,
                817.25F
            },

            {
                1.0F,
                0.0F,
                0.0F,
                191.45F
            },

            {
                0.0F,
                0.0F,
                -1.0F,
                -5.491F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    };


    /*
     * q1 = +60°
     */
    static const robot_joint_angles_t
        joints_q1_60 =
    {
        .value =
        {
            6000,
            0,
            0,
            0,
            0,
            0
        }
    };


    static const robot_transform_t
        expected_q1_60 =
    {
        .matrix =
        {
            {
                -0.866025F,
                0.5F,
                0.0F,
                242.82444F
            },

            {
                0.5F,
                0.866025F,
                0.0F,
                803.48425F
            },

            {
                0.0F,
                0.0F,
                -1.0F,
                -5.491F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    };


    /*
     * q2 = -90°
     */
    static const robot_joint_angles_t
        joints_q2_minus_90 =
    {
        .value =
        {
            0,
            -9000,
            0,
            0,
            0,
            0
        }
    };


    static const robot_transform_t
        expected_q2_minus_90 =
    {
        .matrix =
        {
            {
                0.0F,
                0.0F,
                1.0F,
                94.65F
            },

            {
                1.0F,
                0.0F,
                0.0F,
                191.45F
            },

            {
                0.0F,
                1.0F,
                0.0F,
                906.409F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    };


    /*
     * Main IK Validation：
     *
     * [
     *   30,
     *  -45,
     *   60,
     *   20,
     *   40,
     *  -10
     * ]
     */
    static const robot_joint_angles_t
        joints_ik_validation =
    {
        .value =
        {
            3000,
            -4500,
            6000,
            2000,
            4000,
            -1000
        }
    };


    /* =====================================================
     * FK
     * ===================================================== */

    if (
        run_fk_test_case(
            &joints_zero,
            &expected_zero
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_fk_test_case(
            &joints_q1_60,
            &expected_q1_60
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_fk_test_case(
            &joints_q2_minus_90,
            &expected_q2_minus_90
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* =====================================================
     * q1
     * ===================================================== */

    if (
        run_q1_test_case(
            &expected_zero,
            KINEMATICS_TEST_Q1_ZERO_BRANCH_A_RAD,
            KINEMATICS_TEST_Q1_ZERO_BRANCH_B_RAD
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_q1_test_case(
            &expected_q1_60,
            KINEMATICS_TEST_Q1_60_BRANCH_A_RAD,
            KINEMATICS_TEST_Q1_60_BRANCH_B_RAD
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* =====================================================
     * Analytic IK Individual Stages
     * ===================================================== */

    if (
        run_partial_ik_chain_test(
            &joints_ik_validation
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* =====================================================
     * Public IK Solution Assembly
     * ===================================================== */

    if (
        run_inverse_solution_assembly_test(
            &joints_ik_validation
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /* =====================================================
     * Wrist Singularity
     * ===================================================== */

    if (
        run_wrist_singularity_test(
            &joints_zero
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}