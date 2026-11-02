/**
 * @file kinematics_self_test.c
 * @brief 回归测试或夹具：kinematics self。
 */

#include "kinematics_self_test.h"

#include "kinematics_fk_test.h"
#include "kinematics_ik_solver_test.h"
#include "kinematics_dedup_test.h"
#include "kinematics_ik_test.h"

robot_status_t kinematics_self_test_run(void)
{
    /*
     * 第一层：
     * 正运动学基础验证。
     */
    if (
        kinematics_fk_test_run()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 第二层：
     * 解析 IK 各个数学阶段验证。
     */
    if (
        kinematics_ik_solver_test_run()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 第三层：
     * 单独验证逆解容器的去重机制。
     *
     * 不依赖近奇异姿态的逆解数量。
     */
    if (
        kinematics_dedup_test_run()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 第四层：
     * Public IK 的完整集成验证。
     *
     * 包括：
     * - 8-Solution Assembly
     * - FK / IK Round-Trip
     * - Unreachable Target
     * - Singularity
     * - Near-Singularity Numerical Boundary
     */
    if (
        kinematics_ik_test_run()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}
