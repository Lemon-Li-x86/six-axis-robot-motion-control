/*
 * 文件：kinematics_common.c
 *
 * 用途：
 * 实现 Kinematics Module 内部真正共享的基础逻辑。
 */

#include <stddef.h>
#include <stdint.h>

#include "kinematics_private.h"
#include "matrix4.h"


robot_real_t
kinematics_abs_real(
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


robot_real_t
kinematics_normalize_angle_rad(
    robot_real_t angle_rad
)
{
    while (
        angle_rad
        >=
        KINEMATICS_PI_F
    )
    {
        angle_rad -=
            KINEMATICS_TWO_PI_F;
    }


    while (
        angle_rad
        <
        -KINEMATICS_PI_F
    )
    {
        angle_rad +=
            KINEMATICS_TWO_PI_F;
    }


    return
        angle_rad;
}


robot_real_t
kinematics_sqrt_nonnegative(
    robot_real_t value
)
{
    robot_real_t
        estimate;


    uint32_t
        iteration;


    if (
        value
        <=
        0.0F
    )
    {
        return
            0.0F;
    }


    estimate =
        (
            value
            >=
            1.0F
        )
        ?
        value
        :
        1.0F;


    for (
        iteration = 0U;
        iteration < 16U;
        iteration++
    )
    {
        estimate =
            0.5F
            *
            (
                estimate
                +
                value
                /
                estimate
            );
    }


    return
        estimate;
}


/* =========================================================
 * Joint Angle Conversion
 * ========================================================= */

static int16_t
kinematics_angle_rad_to_joint_value(
    robot_real_t angle_rad
)
{
    robot_real_t
        angle_deg;


    robot_real_t
        scaled_value;


    int32_t
        rounded_value;


    angle_rad =
        kinematics_normalize_angle_rad(
            angle_rad
        );


    angle_deg =
        angle_rad
        *
        KINEMATICS_RAD_TO_DEG_F;


    scaled_value =
        angle_deg
        /
        ROBOT_JOINT_ANGLE_UNIT_DEG;


    if (
        scaled_value
        >=
        0.0F
    )
    {
        rounded_value =
            (int32_t)
            (
                scaled_value
                +
                0.5F
            );
    }
    else
    {
        rounded_value =
            (int32_t)
            (
                scaled_value
                -
                0.5F
            );
    }


    while (
        rounded_value
        >=
        ROBOT_JOINT_HALF_TURN_RAW
    )
    {
        rounded_value -=
            ROBOT_JOINT_FULL_TURN_RAW;
    }


    while (
        rounded_value
        <
        -ROBOT_JOINT_HALF_TURN_RAW
    )
    {
        rounded_value +=
            ROBOT_JOINT_FULL_TURN_RAW;
    }


    return
        (int16_t)
        rounded_value;
}


/* =========================================================
 * Solution Helpers
 * ========================================================= */

void
kinematics_build_joint_solution(
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q4_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_joint_angles_t *solution
)
{
    if (
        solution
        ==
        NULL
    )
    {
        return;
    }


    solution->value[0] =
        kinematics_angle_rad_to_joint_value(
            q1_rad
        );

    solution->value[1] =
        kinematics_angle_rad_to_joint_value(
            q2_rad
        );

    solution->value[2] =
        kinematics_angle_rad_to_joint_value(
            q3_rad
        );

    solution->value[3] =
        kinematics_angle_rad_to_joint_value(
            q4_rad
        );

    solution->value[4] =
        kinematics_angle_rad_to_joint_value(
            q5_rad
        );

    solution->value[5] =
        kinematics_angle_rad_to_joint_value(
            q6_rad
        );
}


static uint8_t
kinematics_joint_solution_equal(
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


robot_status_t
kinematics_add_solution_if_unique(
    kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *candidate
)
{
    uint32_t
        solution_index;


    if (
        solutions
        ==
        NULL
        ||
        candidate
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    for (
        solution_index = 0U;
        solution_index < solutions->count;
        solution_index++
    )
    {
        if (
            kinematics_joint_solution_equal(
                &solutions->solutions[
                    solution_index
                ],
                candidate
            )
        )
        {
            return
                ROBOT_STATUS_OK;
        }
    }


    if (
        solutions->count
        >=
        KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return
            ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }


    solutions->solutions[
        solutions->count
    ] =
        *candidate;


    solutions->count++;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Public Transform -> Standard DH Transform
 * ========================================================= */

robot_status_t
kinematics_public_to_dh_transform(
    const robot_model_t *model,
    const robot_transform_t *public_transform,
    robot_transform_t *dh_transform
)
{
    robot_transform_t
        converted;


    robot_status_t
        status;


    if (
        model
        ==
        NULL
        ||
        public_transform
        ==
        NULL
        ||
        dh_transform
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        matrix4_multiply(
            &model->base_frame_conversion_inverse,
            public_transform,
            &converted
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    return
        matrix4_multiply(
            &converted,
            &model->tool_frame_conversion_inverse,
            dh_transform
        );
}