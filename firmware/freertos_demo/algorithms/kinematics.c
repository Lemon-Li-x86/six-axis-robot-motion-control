/*
 * 文件：kinematics.c
 *
 * 用途：
 * 实现六轴 UR5 机器人运动学算法。
 *
 * 当前已实现：
 *
 * 1. UR5 Standard DH 参数；
 * 2. Standard DH 单关节变换；
 * 3. Forward Kinematics；
 * 4. DH 坐标系到项目公共 URDF / PyBullet
 *    坐标系的固定转换。
 *
 * 当前尚未实现：
 *
 * 1. Inverse Kinematics；
 * 2. IK Solution Selection。
 */

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "kinematics.h"
#include "matrix4.h"


/* =========================================================
 * 数学常量
 * ========================================================= */

#define KINEMATICS_PI_F \
    3.14159265358979323846F


#define KINEMATICS_DEG_TO_RAD_F \
    (KINEMATICS_PI_F / 180.0F)


/* =========================================================
 * UR5 Standard DH 参数
 * ========================================================= */

/*
 * 本项目使用：
 *
 * Universal Robots UR5
 * Legacy / CB-Series Geometry
 *
 * Standard DH：
 *
 * A_i =
 *
 * Rz(theta_i)
 * *
 * Tz(d_i)
 * *
 * Tx(a_i)
 * *
 * Rx(alpha_i)
 *
 *
 * 长度单位：
 *
 * mm
 *
 * 角度单位：
 *
 * rad
 */


/*
 * a_i
 *
 * i = 1 ... 6
 */
static const robot_real_t
    ur5_dh_a_mm[ROBOT_JOINT_COUNT] =
{
    0.0F,
    -425.0F,
    -392.25F,
    0.0F,
    0.0F,
    0.0F
};


/*
 * d_i
 *
 * i = 1 ... 6
 */
static const robot_real_t
    ur5_dh_d_mm[ROBOT_JOINT_COUNT] =
{
    89.159F,
    0.0F,
    0.0F,
    109.15F,
    94.65F,
    82.3F
};


/*
 * alpha_i
 *
 * i = 1 ... 6
 */
static const robot_real_t
    ur5_dh_alpha_rad[ROBOT_JOINT_COUNT] =
{
    KINEMATICS_PI_F / 2.0F,
    0.0F,
    0.0F,
    KINEMATICS_PI_F / 2.0F,
    -KINEMATICS_PI_F / 2.0F,
    0.0F
};


/* =========================================================
 * DH -> URDF 固定坐标转换
 * ========================================================= */

/*
 * Standard DH 使用的 Base Frame
 * 与当前 URDF base_link
 * 方向定义不同。
 *
 * 左侧固定转换：
 *
 * Rz(pi)
 *
 * [ -1  0  0  0 ]
 * [  0 -1  0  0 ]
 * [  0  0  1  0 ]
 * [  0  0  0  1 ]
 */
static const robot_transform_t
    ur5_base_frame_conversion =
{
    .matrix =
    {
        {
            -1.0F,
            0.0F,
            0.0F,
            0.0F
        },

        {
            0.0F,
            -1.0F,
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


/*
 * Standard DH Frame {6}
 * 与当前 URDF ee_link
 * 方向定义不同。
 *
 * 右侧固定转换：
 *
 * [ 0 -1  0  0 ]
 * [ 0  0 -1  0 ]
 * [ 1  0  0  0 ]
 * [ 0  0  0  1 ]
 */
static const robot_transform_t
    ur5_tool_frame_conversion =
{
    .matrix =
    {
        {
            0.0F,
            -1.0F,
            0.0F,
            0.0F
        },

        {
            0.0F,
            0.0F,
            -1.0F,
            0.0F
        },

        {
            1.0F,
            0.0F,
            0.0F,
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


/* =========================================================
 * Standard DH Transform
 * ========================================================= */

/**
 * @brief 根据 Standard DH 参数构造单关节变换矩阵。
 *
 * 计算：
 *
 * A_i =
 *
 * Rz(theta)
 * *
 * Tz(d)
 * *
 * Tx(a)
 * *
 * Rx(alpha)
 *
 * @param theta_rad
 * 当前关节角，单位 rad。
 *
 * @param d_mm
 * DH 参数 d，单位 mm。
 *
 * @param a_mm
 * DH 参数 a，单位 mm。
 *
 * @param alpha_rad
 * DH 参数 alpha，单位 rad。
 *
 * @param transform
 * 输出单关节 4 × 4 变换矩阵。
 */
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
        transform == NULL
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


    /*
     * Standard DH：
     *
     * [ ct  -st*ca   st*sa   a*ct ]
     * [ st   ct*ca  -ct*sa   a*st ]
     * [  0      sa      ca      d ]
     * [  0       0       0      1 ]
     */


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


/* =========================================================
 * Forward Kinematics
 * ========================================================= */

robot_status_t kinematics_forward(
    const robot_joint_angles_t *joints,
    robot_transform_t *transform
)
{
    robot_transform_t
        dh_total;


    robot_transform_t
        joint_transform;


    robot_transform_t
        converted;


    uint32_t
        joint_index;


    robot_status_t
        status;


    if (
        joints == NULL
        ||
        transform == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * 初始：
     *
     * T = I
     */
    status =
        matrix4_identity(
            &dh_total
        );


    if (
        status
        != ROBOT_STATUS_OK
    )
    {
        return status;
    }


    /*
     * 依次计算：
     *
     * T =
     *
     * A1
     * *
     * A2
     * *
     * A3
     * *
     * A4
     * *
     * A5
     * *
     * A6
     */
    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        robot_real_t
            joint_angle_deg;


        robot_real_t
            joint_angle_rad;


        /*
         * Protocol / Public Type：
         *
         * 1 raw unit
         * =
         * 0.01 degree
         */
        joint_angle_deg =
            (robot_real_t)
            joints->value[
                joint_index
            ]
            *
            ROBOT_JOINT_ANGLE_UNIT_DEG;


        /*
         * degree -> radian
         */
        joint_angle_rad =
            joint_angle_deg
            *
            KINEMATICS_DEG_TO_RAD_F;


        /*
         * 当前关节：
         *
         * theta_i
         * d_i
         * a_i
         * alpha_i
         *
         * ->
         *
         * A_i
         */
        status =
            kinematics_build_dh_transform(
                joint_angle_rad,

                ur5_dh_d_mm[
                    joint_index
                ],

                ur5_dh_a_mm[
                    joint_index
                ],

                ur5_dh_alpha_rad[
                    joint_index
                ],

                &joint_transform
            );


        if (
            status
            != ROBOT_STATUS_OK
        )
        {
            return status;
        }


        /*
         * 累积：
         *
         * T = T × A_i
         *
         * matrix4_multiply()
         * 支持 result 与 left 相同。
         */
        status =
            matrix4_multiply(
                &dh_total,
                &joint_transform,
                &dh_total
            );


        if (
            status
            != ROBOT_STATUS_OK
        )
        {
            return status;
        }
    }


    /*
     * 此时得到：
     *
     * dh_total
     * =
     * Standard DH T_0_6
     *
     *
     * 但项目公共坐标使用：
     *
     * URDF / PyBullet
     *
     * base_link -> ee_link
     *
     *
     * 因此：
     *
     * T_public
     *
     * =
     *
     * C_base
     * *
     * T_DH
     * *
     * C_tool
     */


    /*
     * converted =
     *
     * C_base × T_DH
     */
    status =
        matrix4_multiply(
            &ur5_base_frame_conversion,
            &dh_total,
            &converted
        );


    if (
        status
        != ROBOT_STATUS_OK
    )
    {
        return status;
    }


    /*
     * transform =
     *
     * converted × C_tool
     */
    status =
        matrix4_multiply(
            &converted,
            &ur5_tool_frame_conversion,
            transform
        );


    if (
        status
        != ROBOT_STATUS_OK
    )
    {
        return status;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Inverse Kinematics
 * ========================================================= */

robot_status_t kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions
)
{
    if (
        transform == NULL
        ||
        solutions == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    solutions->count =
        0U;


    return
        ROBOT_STATUS_ERROR_NOT_IMPLEMENTED;
}


/* =========================================================
 * IK Solution Selection
 * ========================================================= */

robot_status_t kinematics_select_best_solution(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *reference,
    robot_joint_angles_t *selected
)
{
    if (
        solutions == NULL
        ||
        reference == NULL
        ||
        selected == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        solutions->count
        == 0U
    )
    {
        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    if (
        solutions->count
        > KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        return
            ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }


    return
        ROBOT_STATUS_ERROR_NOT_IMPLEMENTED;
}