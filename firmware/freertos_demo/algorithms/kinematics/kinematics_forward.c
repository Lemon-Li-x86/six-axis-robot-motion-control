/*
 * 文件：kinematics_forward.c
 *
 * 用途：
 * 实现 Forward Kinematics。
 */

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "kinematics.h"
#include "kinematics_private.h"

#include "robot_model.h"
#include "matrix4.h"


static robot_status_t
kinematics_build_dh_transform(
    robot_real_t theta_rad,
    robot_real_t d_mm,
    robot_real_t a_mm,
    robot_real_t alpha_rad,
    robot_transform_t *transform
)
{
    robot_real_t
        cos_theta;

    robot_real_t
        sin_theta;

    robot_real_t
        cos_alpha;

    robot_real_t
        sin_alpha;


    if (
        transform
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    cos_theta =
        cosf(
            theta_rad
        );

    sin_theta =
        sinf(
            theta_rad
        );

    cos_alpha =
        cosf(
            alpha_rad
        );

    sin_alpha =
        sinf(
            alpha_rad
        );


    transform->matrix[0][0] =
        cos_theta;

    transform->matrix[0][1] =
        -sin_theta
        *
        cos_alpha;

    transform->matrix[0][2] =
        sin_theta
        *
        sin_alpha;

    transform->matrix[0][3] =
        a_mm
        *
        cos_theta;


    transform->matrix[1][0] =
        sin_theta;

    transform->matrix[1][1] =
        cos_theta
        *
        cos_alpha;

    transform->matrix[1][2] =
        -cos_theta
        *
        sin_alpha;

    transform->matrix[1][3] =
        a_mm
        *
        sin_theta;


    transform->matrix[2][0] =
        0.0F;

    transform->matrix[2][1] =
        sin_alpha;

    transform->matrix[2][2] =
        cos_alpha;

    transform->matrix[2][3] =
        d_mm;


    transform->matrix[3][0] =
        0.0F;

    transform->matrix[3][1] =
        0.0F;

    transform->matrix[3][2] =
        0.0F;

    transform->matrix[3][3] =
        1.0F;


    return
        ROBOT_STATUS_OK;
}


robot_status_t
kinematics_forward(
    const robot_joint_angles_t *joints,
    robot_transform_t *transform
)
{
    const robot_model_t
        *model;


    robot_transform_t
        dh_total;

    robot_transform_t
        dh_joint;

    robot_transform_t
        converted;


    robot_status_t
        status;


    uint32_t
        joint_index;


    if (
        joints
        ==
        NULL
        ||
        transform
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    model =
        robot_model_get_active();


    status =
        matrix4_identity(
            &dh_total
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


    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        robot_real_t
            angle_deg;

        robot_real_t
            angle_rad;


        angle_deg =
            (robot_real_t)
            joints->value[
                joint_index
            ]
            *
            ROBOT_JOINT_ANGLE_UNIT_DEG;


        angle_rad =
            angle_deg
            *
            KINEMATICS_DEG_TO_RAD_F;


        status =
            kinematics_build_dh_transform(
                angle_rad,
                model->dh_d_mm[
                    joint_index
                ],
                model->dh_a_mm[
                    joint_index
                ],
                model->dh_alpha_rad[
                    joint_index
                ],
                &dh_joint
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


        status =
            matrix4_multiply(
                &dh_total,
                &dh_joint,
                &dh_total
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
    }


    status =
        matrix4_multiply(
            &model->base_frame_conversion,
            &dh_total,
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
            &model->tool_frame_conversion,
            transform
        );
}