/*
 * 文件：kinematics_test_support.c
 *
 * 用途：
 * 实现 Kinematics Test Suites 共用辅助逻辑和固定测试数据。
 */

#include <stddef.h>
#include <stdint.h>

#include "kinematics_test_support.h"


/* =========================================================
 * Shared Test Fixtures
 * ========================================================= */

/*
 * Zero Configuration。
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
 * q1 = +60 degree。
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
 * q2 = -90 degree。
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


/*
 * 普通非奇异 IK Validation Pose：
 *
 * [
 *   30,
 *  -45,
 *   60,
 *   20,
 *   40,
 *  -10
 * ]
 */
static const robot_joint_angles_t
    joints_ik_validation =
{
    .value =
    {
        3000,
        -4500,
        6000,
        2000,
        4000,
        -1000
    }
};


/* =========================================================
 * Scalar Helpers
 * ========================================================= */

robot_real_t
kinematics_test_abs_real(
    robot_real_t value
)
{
    if (
        value
        <
        0.0F
    )
    {
        return
            -value;
    }


    return
        value;
}


uint8_t
kinematics_test_scalar_is_close(
    robot_real_t actual,
    robot_real_t expected,
    robot_real_t tolerance
)
{
    if (
        kinematics_test_abs_real(
            actual
            -
            expected
        )
        >
        tolerance
    )
    {
        return
            0U;
    }


    return
        1U;
}


/* =========================================================
 * Joint Helpers
 * ========================================================= */

uint8_t
kinematics_test_joint_angles_are_equal(
    const robot_joint_angles_t *left,
    const robot_joint_angles_t *right
)
{
    uint32_t
        joint_index;


    if (
        left
        ==
        NULL
        ||
        right
        ==
        NULL
    )
    {
        return
            0U;
    }


    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        if (
            left->value[
                joint_index
            ]
            !=
            right->value[
                joint_index
            ]
        )
        {
            return
                0U;
        }
    }


    return
        1U;
}


uint8_t
kinematics_test_joint_angles_are_canonical(
    const robot_joint_angles_t *joints
)
{
    uint32_t
        joint_index;


    if (
        joints
        ==
        NULL
    )
    {
        return
            0U;
    }


    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        if (
            joints->value[
                joint_index
            ]
            <
            ROBOT_JOINT_ANGLE_MIN_RAW
            ||
            joints->value[
                joint_index
            ]
            >
            ROBOT_JOINT_ANGLE_MAX_RAW
        )
        {
            return
                0U;
        }
    }


    return
        1U;
}


/* =========================================================
 * Transform Helpers
 * ========================================================= */

static uint8_t
kinematics_test_transform_is_close_with_tolerance(
    const robot_transform_t *actual,
    const robot_transform_t *expected,
    robot_real_t rotation_tolerance,
    robot_real_t translation_tolerance_mm
)
{
    uint32_t
        row;

    uint32_t
        column;


    if (
        actual
        ==
        NULL
        ||
        expected
        ==
        NULL
    )
    {
        return
            0U;
    }


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
            robot_real_t
                tolerance;


            robot_real_t
                difference;


            /*
             * Homogeneous Transform：
             *
             * [ R R R Tx ]
             * [ R R R Ty ]
             * [ R R R Tz ]
             * [ 0 0 0  1 ]
             *
             * 只有前三行第四列使用 mm Translation Tolerance。
             */
            if (
                column
                ==
                3U
                &&
                row
                <
                3U
            )
            {
                tolerance =
                    translation_tolerance_mm;
            }
            else
            {
                tolerance =
                    rotation_tolerance;
            }


            difference =
                kinematics_test_abs_real(
                    actual->matrix[
                        row
                    ][
                        column
                    ]
                    -
                    expected->matrix[
                        row
                    ][
                        column
                    ]
                );


            if (
                difference
                >
                tolerance
            )
            {
                return
                    0U;
            }
        }
    }


    return
        1U;
}


uint8_t
kinematics_test_transform_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
)
{
    return
        kinematics_test_transform_is_close_with_tolerance(
            actual,
            expected,
            KINEMATICS_TEST_ROTATION_TOLERANCE,
            KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM
        );
}


uint8_t
kinematics_test_transform_round_trip_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
)
{
    return
        kinematics_test_transform_is_close_with_tolerance(
            actual,
            expected,
            KINEMATICS_TEST_IK_ROUND_TRIP_ROTATION_TOLERANCE,
            KINEMATICS_TEST_IK_ROUND_TRIP_TRANSLATION_TOLERANCE_MM
        );
}


/* =========================================================
 * Shared Fixture Access
 * ========================================================= */

const robot_joint_angles_t *
kinematics_test_get_joints_zero(void)
{
    return
        &joints_zero;
}


const robot_transform_t *
kinematics_test_get_expected_zero(void)
{
    return
        &expected_zero;
}


const robot_joint_angles_t *
kinematics_test_get_joints_q1_60(void)
{
    return
        &joints_q1_60;
}


const robot_transform_t *
kinematics_test_get_expected_q1_60(void)
{
    return
        &expected_q1_60;
}


const robot_joint_angles_t *
kinematics_test_get_joints_q2_minus_90(void)
{
    return
        &joints_q2_minus_90;
}


const robot_transform_t *
kinematics_test_get_expected_q2_minus_90(void)
{
    return
        &expected_q2_minus_90;
}


const robot_joint_angles_t *
kinematics_test_get_joints_ik_validation(void)
{
    return
        &joints_ik_validation;
}