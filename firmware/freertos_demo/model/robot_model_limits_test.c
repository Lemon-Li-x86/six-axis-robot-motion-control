
/*
 * 文件：robot_model_limits_test.c
 *
 * 用途：
 * 在 Host 环境检查 UR5 Robot Model 中的关节限位：
 *
 * 1. Active Robot Model 不为空；
 * 2. 六轴限位与当前 URDF 一致；
 * 3. 每个关节的下限小于上限；
 * 4. 关节零位位于限位之内。
 *
 * 本文件独立运行，不依赖 FreeRTOS 或 QEMU。
 * 当前不测试 IK 最优解选择和运动控制限位执行。
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "robot_model.h"


/* =========================================================
 * Expected Joint Limits
 * ========================================================= */

/*
 * 参考来源：当前项目的 UR5 URDF。
 *
 * 单位：
 * 0.01 degree。
 */
static const robot_joint_positions_t expected_min_positions =
{
    .value =
    {
        -36000,
        -36000,
        -18000,
        -36000,
        -36000,
        -36000
    }
};

static const robot_joint_positions_t expected_max_positions =
{
    .value =
    {
        36000,
        36000,
        18000,
        36000,
        36000,
        36000
    }
};


/* =========================================================
 * Joint Limit Validation
 * ========================================================= */

static int run_joint_limit_test(void)
{
    const robot_model_t *model;
    uint32_t joint_index;

    model = robot_model_get_active();

    if (model == NULL)
    {
        puts("FAIL: Active Robot Model is NULL");
        return 1;
    }

    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        int32_t lower;
        int32_t upper;

        lower = model->joint_min_positions.value[joint_index];
        upper = model->joint_max_positions.value[joint_index];

        if (
            lower != expected_min_positions.value[joint_index]
            || upper != expected_max_positions.value[joint_index]
        )
        {
            printf(
                "FAIL: J%lu limit does not match URDF\n",
                (unsigned long)(joint_index + 1U)
            );
            return 1;
        }

        if (lower >= upper)
        {
            printf(
                "FAIL: J%lu lower limit >= upper limit\n",
                (unsigned long)(joint_index + 1U)
            );
            return 1;
        }

        if (lower > 0 || upper < 0)
        {
            printf(
                "FAIL: J%lu zero position is out of range\n",
                (unsigned long)(joint_index + 1U)
            );
            return 1;
        }

        printf(
            "J%lu: [%.2f, %.2f] deg\n",
            (unsigned long)(joint_index + 1U),
            (double)lower * (double)ROBOT_JOINT_ANGLE_UNIT_DEG,
            (double)upper * (double)ROBOT_JOINT_ANGLE_UNIT_DEG
        );
    }

    return 0;
}


/* =========================================================
 * Test Entry
 * ========================================================= */

int main(void)
{
    if (run_joint_limit_test() != 0)
    {
        return 1;
    }

    puts("PASS: UR5 joint limits match current URDF");
    return 0;
}
