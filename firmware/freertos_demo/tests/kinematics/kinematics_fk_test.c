/*
 * 文件：kinematics_fk_test.c
 *
 * 用途：
 * 验证 Forward Kinematics 与已知参考矩阵一致。
 */

#include "kinematics_fk_test.h"

#include "kinematics.h"
#include "kinematics_test_support.h"


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
        !kinematics_test_transform_is_close(
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


robot_status_t
kinematics_fk_test_run(void)
{
    if (
        run_fk_test_case(
            kinematics_test_get_joints_zero(),
            kinematics_test_get_expected_zero()
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
            kinematics_test_get_joints_q1_60(),
            kinematics_test_get_expected_q1_60()
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
            kinematics_test_get_joints_q2_minus_90(),
            kinematics_test_get_expected_q2_minus_90()
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