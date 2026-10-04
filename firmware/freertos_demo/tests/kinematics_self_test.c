#include <stdint.h>

#include "kinematics_self_test.h"

#include "kinematics.h"
#include "kinematics_internal.h"

#include "robot_types.h"


/* =========================================================
 * Test Tolerance
 * ========================================================= */

/*
 * 已知 FK Reference Matrix Test。
 *
 * 这些测试直接使用预先计算好的期望矩阵，
 * 因此保持较严格容差。
 */
#define KINEMATICS_TEST_ROTATION_TOLERANCE \
    0.001F

#define KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM \
    0.05F


/*
 * IK -> FK Round-Trip Test。
 *
 * kinematics_inverse() 输出最终会量化为：
 *
 * 0.01 degree
 *
 * 因此即使解析解在浮点域完全正确，
 * 转成 robot_joint_angles_t 后再执行 FK，
 * 末端位置仍然会产生少量量化误差。
 *
 * 所以 Round-Trip 使用独立容差，
 * 不与 Known FK Reference Test 混用。
 */
#define KINEMATICS_TEST_IK_ROUND_TRIP_ROTATION_TOLERANCE \
    0.002F

#define KINEMATICS_TEST_IK_ROUND_TRIP_TRANSLATION_TOLERANCE_MM \
    0.5F


#define KINEMATICS_TEST_ANGLE_TOLERANCE_RAD \
    0.001F


#define KINEMATICS_TEST_CANONICAL_MIN_VALUE \
    (-18000)

#define KINEMATICS_TEST_CANONICAL_MAX_VALUE \
    18000


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
 * Scalar Helpers
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
 * Joint Helpers
 * ========================================================= */

static uint8_t
joint_angles_are_equal(
    const robot_joint_angles_t *left,
    const robot_joint_angles_t *right
)
{
    uint32_t
        joint_index;


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


static uint8_t
joint_angles_are_canonical(
    const robot_joint_angles_t *joints
)
{
    uint32_t
        joint_index;


    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        if (
            joints->value[
                joint_index
            ]
            <
            KINEMATICS_TEST_CANONICAL_MIN_VALUE
            ||
            joints->value[
                joint_index
            ]
            >=
            KINEMATICS_TEST_CANONICAL_MAX_VALUE
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

/**
 * @brief 使用指定 Rotation / Translation Tolerance
 *        比较两个 4x4 Transform。
 */
static uint8_t
transform_is_close_with_tolerance(
    const robot_transform_t *actual,
    const robot_transform_t *expected,
    robot_real_t rotation_tolerance,
    robot_real_t translation_tolerance_mm
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


            /*
             * Homogeneous Transform：
             *
             * [ R R R Tx ]
             * [ R R R Ty ]
             * [ R R R Tz ]
             * [ 0 0 0  1 ]
             *
             * 只有前三行第四列
             * 使用 mm Translation Tolerance。
             */
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
                    translation_tolerance_mm;
            }
            else
            {
                tolerance =
                    rotation_tolerance;
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


/**
 * @brief Known FK Reference Matrix 的严格比较。
 */
static uint8_t
transform_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
)
{
    return
        transform_is_close_with_tolerance(
            actual,
            expected,
            KINEMATICS_TEST_ROTATION_TOLERANCE,
            KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM
        );
}


/**
 * @brief IK Solution Quantization 后的
 *        FK Round-Trip 比较。
 */
static uint8_t
transform_round_trip_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
)
{
    return
        transform_is_close_with_tolerance(
            actual,
            expected,
            KINEMATICS_TEST_IK_ROUND_TRIP_ROTATION_TOLERANCE,
            KINEMATICS_TEST_IK_ROUND_TRIP_TRANSLATION_TOLERANCE_MM
        );
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
 * Partial IK Chain Test
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


    /* =====================================================
     * q1
     * ===================================================== */

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


    /* =====================================================
     * q5
     * ===================================================== */

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


    /* =====================================================
     * q6
     * ===================================================== */

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


    /* =====================================================
     * q3
     * ===================================================== */

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


    /* =====================================================
     * q2
     * ===================================================== */

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


    /* =====================================================
     * q4
     * ===================================================== */

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

static robot_status_t
run_inverse_solution_assembly_test(
    const robot_joint_angles_t *original_joints
)
{
    robot_transform_t
        transform;


    kinematics_ik_solutions_t
        solutions;


    uint32_t
        solution_index;

    uint32_t
        compare_index;


    uint8_t
        original_solution_found =
            0U;


    /*
     * Known Joints
     * ->
     * FK Target
     */
    if (
        kinematics_forward(
            original_joints,
            &transform
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Target
     * ->
     * Public IK API
     */
    if (
        kinematics_inverse(
            &transform,
            &solutions
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 当前这组普通非奇异目标
     * 应得到完整 8 组解析解。
     */
    if (
        solutions.count
        !=
        KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 所有解必须：
     *
     * 1. Canonical；
     * 2. 不重复。
     */
    for (
        solution_index = 0U;
        solution_index < solutions.count;
        solution_index++
    )
    {
        if (
            !joint_angles_are_canonical(
                &solutions.solutions[
                    solution_index
                ]
            )
        )
        {
            return
                ROBOT_STATUS_ERROR_INTERNAL;
        }


        for (
            compare_index =
                solution_index
                +
                1U;

            compare_index < solutions.count;

            compare_index++
        )
        {
            if (
                joint_angles_are_equal(
                    &solutions.solutions[
                        solution_index
                    ],
                    &solutions.solutions[
                        compare_index
                    ]
                )
            )
            {
                return
                    ROBOT_STATUS_ERROR_INTERNAL;
            }
        }


        /*
         * 原始 Joint Configuration
         * 必须是解析解之一。
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
 * IK -> FK Round-Trip Test
 * ========================================================= */

/**
 * @brief 验证 Public IK 返回的每一组 Solution
 *        都能通过 FK 重建原始目标 Transform。
 *
 * 流程：
 *
 * Original Joints
 * ->
 * FK Target
 * ->
 * IK Solutions
 * ->
 * FK(each solution)
 * ->
 * Target Transform
 *
 * 这项测试不是只验证：
 *
 * “原始 Joint Configuration 是否被找回来”。
 *
 * 而是验证：
 *
 * “Public IK API 返回的每一组候选解，
 *  是否都真的描述相同末端位姿。”
 *
 * 这是后续重构 kinematics.c
 * 最重要的 Regression Safety Net。
 */
static robot_status_t
run_inverse_round_trip_test(
    const robot_joint_angles_t *original_joints
)
{
    robot_transform_t
        target_transform;


    robot_transform_t
        reconstructed_transform;


    kinematics_ik_solutions_t
        solutions;


    uint32_t
        solution_index;


    /*
     * Step 1：
     *
     * 已知有效关节姿态
     * ->
     * Target Transform。
     */
    if (
        kinematics_forward(
            original_joints,
            &target_transform
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Step 2：
     *
     * Target Transform
     * ->
     * Public IK Solutions。
     */
    if (
        kinematics_inverse(
            &target_transform,
            &solutions
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 当前测试目标必须至少存在一组解。
     */
    if (
        solutions.count
        ==
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * 防御性检查。
     *
     * Public API 不允许返回
     * 超过固定数组容量的 Solution Count。
     */
    if (
        solutions.count
        >
        KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Step 3：
     *
     * 对 Public IK 返回的每一组 Solution：
     *
     * Solution
     * ->
     * FK
     * ->
     * reconstructed_transform
     *
     * 必须重新得到同一个目标位姿。
     */
    for (
        solution_index = 0U;
        solution_index < solutions.count;
        solution_index++
    )
    {
        if (
            kinematics_forward(
                &solutions.solutions[
                    solution_index
                ],
                &reconstructed_transform
            )
            !=
            ROBOT_STATUS_OK
        )
        {
            return
                ROBOT_STATUS_ERROR_INTERNAL;
        }


        if (
            !transform_round_trip_is_close(
                &reconstructed_transform,
                &target_transform
            )
        )
        {
            return
                ROBOT_STATUS_ERROR_INTERNAL;
        }
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
     * IK Validation：
     *
     * [
     *   30,
     *  -45,
     *   60,
     *   20,
     *   40,
     *  -10
     * ]
     *
     * 这是一组普通非奇异姿态，
     * 当前解析 IK 应产生完整 8 解。
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
     * FK Reference Tests
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
     * q1 Analytic Branch Tests
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
     * Analytic Formula Chain
     *
     * q1
     * ->
     * q5
     * ->
     * q6
     * ->
     * q3
     * ->
     * q2
     * ->
     * q4
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
     * Public kinematics_inverse()
     *
     * Full 8-Solution Assembly
     *
     * 验证：
     *
     * 1. Solution Count；
     * 2. Canonical Angle；
     * 3. Duplicate Removal；
     * 4. Original Configuration Presence。
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
     * IK -> FK Round-Trip Regression
     *
     * 对 Public IK 返回的每一组 Solution
     * 重新执行 FK。
     *
     * 所有解都必须回到同一个 Target Transform。
     * ===================================================== */

    if (
        run_inverse_round_trip_test(
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