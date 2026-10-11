/*
 * 文件：kinematics_fk_test.c
 *
 * 用途：
 * 验证 Public Forward Kinematics：
 *
 * 1. NULL Pointer Contract；
 * 2. Zero Configuration；
 * 3. q1 = +60°；
 * 4. q2 = -90°；
 * 5. 与已知参考 Transform 一致。
 */

#include <stddef.h>

#include "kinematics_fk_test.h"

#include "kinematics.h"
#include "kinematics_test_support.h"


/* =========================================================
 * Test Helpers
 * ========================================================= */

static robot_status_t run_fk_test_case(
    const robot_joint_angles_t *joints,
    const robot_transform_t *expected
)
{
    robot_transform_t actual;

    if (
        kinematics_forward(
            joints,
            &actual
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        !kinematics_test_transform_is_close(
            &actual,
            expected
        )
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Public API Contract
 * ========================================================= */

static robot_status_t run_fk_null_pointer_test(void)
{
    robot_joint_angles_t joints =
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

    robot_transform_t transform;

    if (
        kinematics_forward(
            NULL,
            &transform
        )
        != ROBOT_STATUS_ERROR_NULL_POINTER
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        kinematics_forward(
            &joints,
            NULL
        )
        != ROBOT_STATUS_ERROR_NULL_POINTER
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Complete FK Test Suite
 * ========================================================= */

robot_status_t kinematics_fk_test_run(void)
{
    if (
        run_fk_null_pointer_test()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_fk_test_case(
            kinematics_test_get_joints_zero(),
            kinematics_test_get_expected_zero()
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_fk_test_case(
            kinematics_test_get_joints_q1_60(),
            kinematics_test_get_expected_q1_60()
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_fk_test_case(
            kinematics_test_get_joints_q2_minus_90(),
            kinematics_test_get_expected_q2_minus_90()
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}