/*
 * 文件：kinematics_ik_test.c
 *
 * 用途：
 * 验证 Public kinematics_inverse() 行为。
 *
 * 当前覆盖：
 *
 * 1. 8-Solution Assembly；
 * 2. Canonical Angle；
 * 3. Duplicate Removal；
 * 4. Original Configuration Presence；
 * 5. 所有 IK Solution 的 FK Round-Trip。
 */

#include <stdint.h>

#include "kinematics_ik_test.h"

#include "kinematics.h"
#include "kinematics_test_support.h"


/* =========================================================
 * Public IK Solution Assembly
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
     * 当前普通非奇异目标
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


    for (
        solution_index = 0U;
        solution_index < solutions.count;
        solution_index++
    )
    {
        /*
         * 所有解必须为 Canonical Angle。
         */
        if (
            !kinematics_test_joint_angles_are_canonical(
                &solutions.solutions[
                    solution_index
                ]
            )
        )
        {
            return
                ROBOT_STATUS_ERROR_INTERNAL;
        }


        /*
         * 任意两组 Solution
         * 不允许在 0.01 degree 量化后完全相同。
         */
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
                return
                    ROBOT_STATUS_ERROR_INTERNAL;
            }
        }


        /*
         * 原始 Joint Configuration
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
 * IK -> FK Round-Trip
 * ========================================================= */

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
     * Original Joints
     * ->
     * FK Target。
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
     * FK Target
     * ->
     * IK Solutions。
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


    if (
        solutions.count
        ==
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


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
            !=
            ROBOT_STATUS_OK
        )
        {
            return
                ROBOT_STATUS_ERROR_INTERNAL;
        }


        if (
            !kinematics_test_transform_round_trip_is_close(
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
 * Complete Public IK Test Suite
 * ========================================================= */

robot_status_t
kinematics_ik_test_run(void)
{
    const robot_joint_angles_t
        *validation_joints;


    validation_joints =
        kinematics_test_get_joints_ik_validation();


    if (
        run_inverse_solution_assembly_test(
            validation_joints
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_inverse_round_trip_test(
            validation_joints
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