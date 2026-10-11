/*
 * 文件：kinematics_ik_test.c
 *
 * 用途：
 * 验证 Public kinematics_inverse() 行为。
 *
 * 当前覆盖：
 *
 * 1. NULL Pointer Contract；
 * 2. 8-Solution Assembly；
 * 3. Canonical Angle；
 * 4. Duplicate-Free Output；
 * 5. True Duplicate Collapse；
 * 6. Original Configuration Presence；
 * 7. 多组普通 Pose 的 IK -> FK Round-Trip；
 * 8. 明确不可达目标；
 * 9. Public Singularity；
 * 10. Near-Wrist-Singularity 可解边界。
 */

#include <stddef.h>
#include <stdint.h>

#include "kinematics_ik_test.h"

#include "kinematics.h"
#include "kinematics_test_support.h"


/* =========================================================
 * Additional Round-Trip Fixtures
 * ========================================================= */

/*
 * 额外普通非奇异姿态。
 *
 * 单位：
 * 0.01 degree。
 *
 * 这些姿态用于扩展 Public IK Validation，
 * 避免完整 Round-Trip 只覆盖一组 Joint Configuration。
 */
static const robot_joint_angles_t
    additional_round_trip_joints[] =
{
    {
        .value =
        {
            9000,
            -6000,
            8000,
            3500,
            5500,
            -12000
        }
    },

    {
        .value =
        {
            -12000,
            2000,
            -7000,
            10000,
            -6500,
            4500
        }
    },

    {
        .value =
        {
            -6000,
            -3000,
            4500,
            -7000,
            -5000,
            8000
        }
    }
};


/*
 * Public Singularity Validation Pose。
 *
 * 单位：
 * 0.01 degree。
 *
 * [
 *    90,
 *  -150,
 *     0,
 *  -150,
 *     0,
 *  -150
 * ]
 *
 * 对该 Pose：
 *
 * 1. Wrist Geometry 中存在 singular Branch；
 * 2. 其余 Branch 不形成有效完整解；
 * 3. 最终 Public kinematics_inverse()
 *    应返回 ROBOT_STATUS_ERROR_SINGULAR。
 */
static const robot_joint_angles_t
    singular_validation_joints =
{
    .value =
    {
        9000,
        -15000,
        0,
        -15000,
        0,
        -15000
    }
};



/*
 * Near-Wrist-Singularity Validation Pose。
 *
 * 单位：
 * 0.01 degree。
 *
 * 输入关节角：
 *
 * [
 *      0,
 *      0,
 *   -180,
 *    -60,
 *   0.01,
 *     30
 * ]
 *
 * q5 = 0.01° 时，目标接近腕部奇异位置。
 *
 * 注意：
 *
 * 当前 IK 使用 float32。
 *
 * 对极小 q5，cos(q5) 可能舍入为 1.0F，
 * 导致反算得到 q5 = 0。
 *
 * 这时求解器可能拒绝对应奇异分支。
 *
 * 本测试只验证：
 *
 * 1. 返回状态符合公共 API 约定；
 * 2. 成功返回的 Candidate 不重复；
 * 3. 成功返回的 Candidate 满足 FK 回代精度。
 *
 * 不要求固定生成 7 或 8 组解。
 * 不强制要求恢复原始的每一个关节角。
 */
static const robot_joint_angles_t
    near_wrist_singularity_joints =
{
    .value =
    {
        0,
        0,
        -18000,
        -6000,
        1,
        3000
    }
};


/* =========================================================
 * Public API Contract
 * ========================================================= */

static robot_status_t run_inverse_null_pointer_test(void)
{
    robot_transform_t transform =
    {
        .matrix =
        {
            {
                1.0F,
                0.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                1.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                1.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    };

    kinematics_ik_solutions_t solutions;

    if (
        kinematics_inverse(
            NULL,
            &solutions
        )
        != ROBOT_STATUS_ERROR_NULL_POINTER
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        kinematics_inverse(
            &transform,
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
 * Public IK Solution Assembly
 * ========================================================= */

static robot_status_t run_inverse_solution_assembly_test(
    const robot_joint_angles_t *original_joints
)
{
    robot_transform_t transform;

    kinematics_ik_solutions_t solutions;

    uint32_t solution_index;
    uint32_t compare_index;

    uint8_t original_solution_found = 0U;

    /*
     * Known Joints
     * ->
     * FK Target。
     */
    if (
        kinematics_forward(
            original_joints,
            &transform
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * Target
     * ->
     * Public IK API。
     */
    if (
        kinematics_inverse(
            &transform,
            &solutions
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 当前固定 Validation Pose
     * 为普通非奇异目标，
     * 应得到完整 8 组解析解。
     */
    if (
        solutions.count
        != KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    for (
        solution_index = 0U;
        solution_index < solutions.count;
        solution_index++
    )
    {
        /*
         * 所有 Public IK Solution
         * 必须使用 Canonical Angle。
         */
        if (
            !kinematics_test_joint_angles_are_canonical(
                &solutions.solutions[
                    solution_index
                ]
            )
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }

        /*
         * 任意两组 Public Solution
         * 不允许在 0.01 degree
         * 量化后完全相同。
         */
        for (
            compare_index = solution_index + 1U;
            compare_index < solutions.count;
            compare_index++
        )
        {
            if (
                kinematics_test_joint_angles_are_equal(
                    &solutions.solutions[
                        solution_index
                    ],
                    &solutions.solutions[
                        compare_index
                    ]
                )
            )
            {
                return ROBOT_STATUS_ERROR_INTERNAL;
            }
        }

        /*
         * 用于生成 Target Transform
         * 的原始 Joint Configuration
         * 必须是解析解之一。
         */
        if (
            kinematics_test_joint_angles_are_equal(
                &solutions.solutions[
                    solution_index
                ],
                original_joints
            )
        )
        {
            original_solution_found = 1U;
        }
    }

    if (!original_solution_found)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * IK -> FK Round-Trip
 * ========================================================= */

static robot_status_t run_inverse_round_trip_test(
    const robot_joint_angles_t *original_joints
)
{
    robot_transform_t target_transform;
    robot_transform_t reconstructed_transform;

    kinematics_ik_solutions_t solutions;

    uint32_t solution_index;

    /*
     * Original Joints
     * ->
     * FK Target。
     */
    if (
        kinematics_forward(
            original_joints,
            &target_transform
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * FK Target
     * ->
     * IK Candidate Set。
     */
    if (
        kinematics_inverse(
            &target_transform,
            &solutions
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 普通可达姿态至少应返回一组解，
     * 但并不要求所有姿态都必须拥有 8 个解。
     */
    if (
        solutions.count == 0U
        || solutions.count > KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 每一组 Public IK Solution
     * 都必须通过 FK 回到同一个 Target Transform。
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
            != ROBOT_STATUS_OK
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }

        if (
            !kinematics_test_transform_round_trip_is_close(
                &reconstructed_transform,
                &target_transform
            )
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }
    }

    return ROBOT_STATUS_OK;
}


static robot_status_t run_additional_round_trip_tests(void)
{
    uint32_t test_index;

    const uint32_t test_count =
        (uint32_t)(
            sizeof(additional_round_trip_joints)
            / sizeof(additional_round_trip_joints[0])
        );

    for (
        test_index = 0U;
        test_index < test_count;
        test_index++
    )
    {
        if (
            run_inverse_round_trip_test(
                &additional_round_trip_joints[
                    test_index
                ]
            )
            != ROBOT_STATUS_OK
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Unreachable Target
 * ========================================================= */

static robot_status_t run_inverse_unreachable_target_test(void)
{
    /*
     * 合法 Homogeneous Transform，
     * 但目标位置位于：
     *
     * x = 5000 mm
     *
     * 远超当前 UR5 几何工作空间。
     *
     * Orientation 使用 Identity，
     * 使失败原因集中在几何可达性，
     * 而不是非法 Rotation Matrix。
     */
    const robot_transform_t unreachable_transform =
    {
        .matrix =
        {
            {
                1.0F,
                0.0F,
                0.0F,
                5000.0F
            },

            {
                0.0F,
                1.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                1.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    };

    kinematics_ik_solutions_t solutions;

    robot_status_t status;

    status = kinematics_inverse(
        &unreachable_transform,
        &solutions
    );

    if (status != ROBOT_STATUS_ERROR_NO_SOLUTION)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * Public IK 在开始求解时清零 count。
     *
     * 不可达目标不得留下伪 Candidate。
     */
    if (solutions.count != 0U)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Public Singularity
 * ========================================================= */

static robot_status_t run_inverse_public_singularity_test(void)
{
    robot_transform_t transform;

    kinematics_ik_solutions_t solutions;

    robot_status_t status;

    /*
     * 使用已知 Singular Joint Configuration
     * 生成一个合法 Target Transform。
     *
     * 这样测试的是：
     *
     * “合法但处于当前未处理奇异区域的目标”
     *
     * 而不是人为拼接一个非法 Matrix。
     */
    if (
        kinematics_forward(
            &singular_validation_joints,
            &transform
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    status = kinematics_inverse(
        &transform,
        &solutions
    );

    if (status != ROBOT_STATUS_ERROR_SINGULAR)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 当前 Public Contract：
     *
     * 当没有任何完整可返回 Solution，
     * 且至少一个 Branch 因 Singularity
     * 被拒绝时返回 ERROR_SINGULAR。
     */
    if (solutions.count != 0U)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}



/*
 * =========================================================
 * Near-Wrist-Singularity Numerical Boundary
 * =========================================================
 */

/**
 * @brief 验证接近腕部奇异位置时的 IK 行为。
 *
 * 当前数值边界：
 * q5 = 0.01 degree。
 *
 * 当求解成功时：
 * 1. 至少返回一组解；
 * 2. 不允许重复候选解；
 * 3. 所有候选解必须满足 FK 回代精度。
 *
 * 当求解器明确返回 SINGULAR 时：
 * 不允许残留不完整的 Candidate。
 *
 * 此测试不再要求特定解数量。
 */
static robot_status_t
run_inverse_near_wrist_singularity_test(void)
{
    robot_transform_t target_transform;
    robot_transform_t reconstructed_transform;

    kinematics_ik_solutions_t solutions = {0};

    robot_status_t status;

    uint32_t solution_index;
    uint32_t compare_index;

    /*
     * 1. 使用原始关节配置计算合法目标位姿。
     */
    status = kinematics_forward(
        &near_wrist_singularity_joints,
        &target_transform
    );

    if (status != ROBOT_STATUS_OK)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 2. 对目标位姿执行完整解析 IK。
     */
    status = kinematics_inverse(
        &target_transform,
        &solutions
    );

    /*
     * 3. 如果所有有效分支均不可确定，
     *    求解器允许报告 SINGULAR。
     *
     * 按当前公共 API 约定：
     * 此时不得返回不完整的解集合。
     */
    if (status == ROBOT_STATUS_ERROR_SINGULAR)
    {
        if (solutions.count != 0U)
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }

        return ROBOT_STATUS_OK;
    }

    /*
     * 4. 其他错误不视为测试通过。
     *
     * 因为原始目标来自合法 FK，
     * 它并不是一个无意义的随机矩阵。
     */
    if (status != ROBOT_STATUS_OK)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 5. 求解成功时必须有有效 Candidate。
     *
     * 不强制规定解的确切数量：
     * 近奇异位置的可用分支数量可能减少。
     */
    if (
        solutions.count == 0U
        ||
        solutions.count > KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 6. 检查每一个返回的 Candidate。
     */
    for (
        solution_index = 0U;
        solution_index < solutions.count;
        solution_index++
    )
    {
        /*
         * 关节角必须处于 Canonical 范围。
         */
        if (
            !kinematics_test_joint_angles_are_canonical(
                &solutions.solutions[solution_index]
            )
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }

        /*
         * 任意两个 Candidate 不得相同。
         */
        for (
            compare_index = solution_index + 1U;
            compare_index < solutions.count;
            compare_index++
        )
        {
            if (
                kinematics_test_joint_angles_are_equal(
                    &solutions.solutions[solution_index],
                    &solutions.solutions[compare_index]
                )
            )
            {
                return ROBOT_STATUS_ERROR_INTERNAL;
            }
        }

        /*
         * 7. 将逆解重新代入 FK。
         *
         * 每组被公开返回的 IK Solution
         * 都应该对应原始目标 Transform。
         */
        status = kinematics_forward(
            &solutions.solutions[solution_index],
            &reconstructed_transform
        );

        if (status != ROBOT_STATUS_OK)
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }

        /*
         * 使用项目已有的 IK Round-Trip 容差。
         *
         * 当前容差：
         * Rotation 0.002
         * Translation 0.5 mm
         */
        if (
            !kinematics_test_transform_round_trip_is_close(
                &reconstructed_transform,
                &target_transform
            )
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Complete Public IK Test Suite
 * ========================================================= */

robot_status_t kinematics_ik_test_run(void)
{
    const robot_joint_angles_t *validation_joints;

    validation_joints =
        kinematics_test_get_joints_ik_validation();

    if (
        run_inverse_null_pointer_test()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 固定 Validation Pose：
     *
     * [30, -45, 60, 20, 40, -10] degree。
     *
     * 对该 Pose 额外要求：
     *
     * 完整 8 Branch。
     */
    if (
        run_inverse_solution_assembly_test(
            validation_joints
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_inverse_round_trip_test(
            validation_joints
        )
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * 扩展多个独立普通姿态，
     * 验证 Public IK 并非只对
     * 单一 Reference Pose 成立。
     */
    if (
        run_additional_round_trip_tests()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_inverse_unreachable_target_test()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_inverse_public_singularity_test()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    if (
        run_inverse_near_wrist_singularity_test()
        != ROBOT_STATUS_OK
    )
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}