#include <stdint.h>

#include "kinematics_self_test.h"

#include "kinematics.h"
#include "robot_types.h"


/* =========================================================
 * 测试误差
 * ========================================================= */

/*
 * Rotation Matrix 无量纲。
 */
#define KINEMATICS_TEST_ROTATION_TOLERANCE \
    0.001F


/*
 * Translation 单位 mm。
 */
#define KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM \
    0.05F


/* =========================================================
 * Float Absolute Value
 * ========================================================= */

static robot_real_t test_abs(
    robot_real_t value
)
{
    if (
        value < 0.0F
    )
    {
        return -value;
    }


    return value;
}


/* =========================================================
 * Transform Compare
 * ========================================================= */

static uint8_t transform_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
)
{
    uint32_t row;
    uint32_t column;


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
            robot_real_t tolerance;
            robot_real_t difference;


            /*
             * 第 4 列前三个元素是 Translation：
             *
             * Tx
             * Ty
             * Tz
             *
             * 单位 mm。
             */
            if (
                column == 3U
                &&
                row < 3U
            )
            {
                tolerance =
                    KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM;
            }
            else
            {
                tolerance =
                    KINEMATICS_TEST_ROTATION_TOLERANCE;
            }


            difference =
                test_abs(
                    actual->matrix[row][column]
                    -
                    expected->matrix[row][column]
                );


            if (
                difference > tolerance
            )
            {
                return 0U;
            }
        }
    }


    return 1U;
}


/* =========================================================
 * Test Case Runner
 * ========================================================= */

static robot_status_t run_test_case(
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
 * Complete Self Test
 * ========================================================= */

robot_status_t kinematics_self_test_run(void)
{
    /*
     * -----------------------------------------------------
     * Test 1
     *
     * q =
     *
     * [0, 0, 0, 0, 0, 0]
     * -----------------------------------------------------
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
     * -----------------------------------------------------
     * Test 2
     *
     * q =
     *
     * [60°, 0, 0, 0, 0, 0]
     *
     * 60° = 6000 × 0.01°
     * -----------------------------------------------------
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
     * -----------------------------------------------------
     * Test 3
     *
     * q =
     *
     * [0, -90°, 0, 0, 0, 0]
     *
     * -90° = -9000 × 0.01°
     * -----------------------------------------------------
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


    /* =====================================================
     * Execute
     * ===================================================== */

    if (
        run_test_case(
            &joints_zero,
            &expected_zero
        )
        != ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_test_case(
            &joints_q1_60,
            &expected_q1_60
        )
        != ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    if (
        run_test_case(
            &joints_q2_minus_90,
            &expected_q2_minus_90
        )
        != ROBOT_STATUS_OK
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}