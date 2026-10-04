/*
 * 文件：kinematics_self_test.c
 *
 * 用途：
 * 编排完整 Kinematics Startup Self Test。
 *
 * 本文件不实现具体测试逻辑。
 */

#include "kinematics_self_test.h"

#include "kinematics_fk_test.h"
#include "kinematics_ik_solver_test.h"
#include "kinematics_ik_test.h"


robot_status_t
kinematics_self_test_run(void)
{
    if (
        kinematics_fk_test_run()
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_ik_solver_test_run()
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        kinematics_ik_test_run()
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