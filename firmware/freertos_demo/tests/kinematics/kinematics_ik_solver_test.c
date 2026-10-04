/*
 * 文件：kinematics_ik_solver_test.c
 *
 * 用途：
 * 验证 UR5 Analytic IK 各阶段数学求解器。
 *
 * 当前覆盖：
 *
 * q1
 * q5
 * q6
 * q3
 * q2
 * q4
 * Wrist Singularity
 */

#include "kinematics_ik_solver_test.h"

#include "kinematics.h"
#include "kinematics_internal.h"
#include "kinematics_test_support.h"


/* =========================================================
 * Known Expected Angles
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
 * q1 Reference Branch Test
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
 * Complete Analytic Formula Chain
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
        kinematics_test_abs_real(
            q1_candidates_rad[0]
            -
            KINEMATICS_TEST_Q1_30_RAD
        );


    difference_1 =
        kinematics_test_abs_real(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
        !kinematics_test_scalar_is_close(
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
 * Wrist Singularity
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
 * Complete Solver Test Suite
 * ========================================================= */

robot_status_t
kinematics_ik_solver_test_run(void)
{
    if (
        run_q1_test_case(
            kinematics_test_get_expected_zero(),
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
            kinematics_test_get_expected_q1_60(),
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


    if (
        run_partial_ik_chain_test(
            kinematics_test_get_joints_ik_validation()
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_wrist_singularity_test(
            kinematics_test_get_joints_zero()
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