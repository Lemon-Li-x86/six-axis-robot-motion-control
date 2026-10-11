
/*
 * 文件：kinematics_dedup_test.c
 *
 * 用途：
 * 验证 IK Solution 去重机制。
 *
 * 设计原则：
 *
 * 去重属于 Solution Container 行为。
 *
 * 因此本测试不依赖 Analytic IK
 * 在近奇异位置计算出固定数量的解。
 *
 * 这样可以单独判断：
 * 去重机制是否正确。
 */

#include <stddef.h>
#include <stdint.h>

#include "kinematics_dedup_test.h"

/*
 * 测试代码允许访问运动学内部接口。
 * 正式 Application Code 不应依赖此头文件。
 */
#include "kinematics_private.h"


/* =========================================================
 * Joint Comparison
 * ========================================================= */

/**
 * @brief 比较两组量化后的关节角。
 *
 * 公共关节角单位为 0.01 degree，
 * 因此这里可以直接逐项比较整数值。
 */
static uint8_t joint_angles_equal(
    const robot_joint_angles_t *left,
    const robot_joint_angles_t *right
)
{
    uint32_t index;

    if (left == NULL || right == NULL)
    {
        return 0U;
    }

    for (index = 0U;
         index < ROBOT_JOINT_COUNT;
         index++)
    {
        if (left->value[index] != right->value[index])
        {
            return 0U;
        }
    }

    return 1U;
}


/* =========================================================
 * Deduplication Test
 * ========================================================= */

robot_status_t kinematics_dedup_test_run(void)
{
    /*
     * 使用零初始化，保证：
     *
     * solutions.count = 0
     *
     * 测试不读取未初始化的解。
     */
    kinematics_ik_solutions_t solutions = {0};

    robot_joint_angles_t candidate_positive_pi;
    robot_joint_angles_t candidate_negative_pi;
    robot_joint_angles_t candidate_different;

    robot_status_t status;

    /*
     * 第一组 Candidate：
     *
     * q3 = +180°。
     *
     * 规范化后应表示为 -180°。
     *
     * 其他关节使用普通非奇异角度，
     * 本测试不依赖复杂的 IK 求解过程。
     */
    kinematics_build_joint_solution(
        0.0F,
        0.0F,
        KINEMATICS_PI_F,
        -KINEMATICS_PI_F / 3.0F,
        KINEMATICS_PI_F / 4.0F,
        KINEMATICS_PI_F / 6.0F,
        &candidate_positive_pi
    );

    /*
     * 第二组 Candidate：
     *
     * q3 = -180°。
     *
     * 它与 +180° 代表相同关节位置。
     * 两者应被规范化为同一组关节角。
     */
    kinematics_build_joint_solution(
        0.0F,
        0.0F,
        -KINEMATICS_PI_F,
        -KINEMATICS_PI_F / 3.0F,
        KINEMATICS_PI_F / 4.0F,
        KINEMATICS_PI_F / 6.0F,
        &candidate_negative_pi
    );

    /*
     * 检查角度规范化：
     *
     * q3 的整数值应为 -18000，
     * 即 -180.00°。
     */
    if (
        candidate_positive_pi.value[2] != -18000
        ||
        candidate_negative_pi.value[2] != -18000
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 两组数学上等价的关节角，
     * 应转换成相同的 Public Solution。
     */
    if (
        !joint_angles_equal(
            &candidate_positive_pi,
            &candidate_negative_pi
        )
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 检查空指针处理。
     */
    status = kinematics_add_solution_if_unique(
        NULL,
        &candidate_positive_pi
    );

    if (status != ROBOT_STATUS_ERROR_NULL_POINTER)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    status = kinematics_add_solution_if_unique(
        &solutions,
        NULL
    );

    if (status != ROBOT_STATUS_ERROR_NULL_POINTER)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 插入第一组解。
     *
     * 预期 count：
     * 0 -> 1。
     */
    status = kinematics_add_solution_if_unique(
        &solutions,
        &candidate_positive_pi
    );

    if (
        status != ROBOT_STATUS_OK
        ||
        solutions.count != 1U
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 再插入数学上等价的第二组解。
     *
     * 去重机制应识别已有 Candidate。
     *
     * 预期 count：
     * 1 -> 1。
     */
    status = kinematics_add_solution_if_unique(
        &solutions,
        &candidate_negative_pi
    );

    if (
        status != ROBOT_STATUS_OK
        ||
        solutions.count != 1U
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 检查保存的第一组关节角没有被覆盖。
     */
    if (
        !joint_angles_equal(
            &solutions.solutions[0],
            &candidate_positive_pi
        )
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 构造真正不同的 Candidate。
     *
     * 仅将第一关节增加 1.00°。
     *
     * 原本：
     * q1 = 0.00°
     *
     * 修改：
     * q1 = 1.00°
     *
     * 这种差异远大于 0.01° 量化单位。
     */
    candidate_different = candidate_positive_pi;
    candidate_different.value[0] = 100;

    /*
     * 不同 Candidate 应被正常加入。
     *
     * 预期 count：
     * 1 -> 2。
     */
    status = kinematics_add_solution_if_unique(
        &solutions,
        &candidate_different
    );

    if (
        status != ROBOT_STATUS_OK
        ||
        solutions.count != 2U
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 再次插入相同 Candidate。
     *
     * 预期 count：
     * 2 -> 2。
     */
    status = kinematics_add_solution_if_unique(
        &solutions,
        &candidate_different
    );

    if (
        status != ROBOT_STATUS_OK
        ||
        solutions.count != 2U
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 检查两组不同解都被正确保存。
     */
    if (
        !joint_angles_equal(
            &solutions.solutions[0],
            &candidate_positive_pi
        )
        ||
        !joint_angles_equal(
            &solutions.solutions[1],
            &candidate_different
        )
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}
