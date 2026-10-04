/*
 * 文件：kinematics.c
 *
 * 用途：
 * 实现 UR5 六轴机器人运动学算法。
 *
 * 当前已实现：
 *
 * 1. Standard DH 单关节变换；
 * 2. UR5 Forward Kinematics；
 * 3. DH / Public 坐标系转换；
 * 4. IK 第一阶段：
 *    根据目标 T_0_6 求两个 q1 Shoulder 候选。
 *
 * 尚未完成：
 *
 * 1. q5；
 * 2. q6；
 * 3. q3；
 * 4. q2；
 * 5. q4；
 * 6. 完整 IK Solution Assembly；
 * 7. Joint Limit Filtering；
 * 8. IK Solution Selection。
 */


#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "kinematics.h"
#include "matrix4.h"


/* =========================================================
 * Mathematical Constants
 * ========================================================= */

#define KINEMATICS_PI_F \
    3.14159265358979323846F

#define KINEMATICS_TWO_PI_F \
    (2.0F * KINEMATICS_PI_F)

#define KINEMATICS_DEG_TO_RAD_F \
    (KINEMATICS_PI_F / 180.0F)


/*
 * 用于浮点数几何判断。
 */
#define KINEMATICS_GEOMETRY_EPSILON_MM \
    0.001F


/* =========================================================
 * UR5 Standard DH Parameters
 * =========================================================
 *
 * Legacy / CB-series UR5。
 *
 * Standard DH：
 *
 * A_i =
 * Rz(theta_i)
 * Tz(d_i)
 * Tx(a_i)
 * Rx(alpha_i)
 *
 * 单位：
 *
 * a, d：
 * mm
 *
 * alpha：
 * rad
 * ========================================================= */

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
 * Coordinate Conversion
 * =========================================================
 *
 * 内部 Standard DH：
 *
 * T_0_6
 *
 * 公共接口：
 *
 * T_base_ee
 *
 * Forward：
 *
 * T_base_ee =
 * C_base
 * ×
 * T_0_6
 * ×
 * C_tool
 *
 *
 * Inverse：
 *
 * T_0_6 =
 * C_base^-1
 * ×
 * T_base_ee
 * ×
 * C_tool^-1
 *
 * ========================================================= */


/*
 * C_base
 *
 * 当前矩阵：
 *
 * Rz(pi)
 *
 * 因此：
 *
 * C_base^-1 = C_base
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
 * C_tool
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


/*
 * C_tool^-1
 *
 * C_tool 只有 Rotation，
 * 所以：
 *
 * inverse(R) = transpose(R)
 */
static const robot_transform_t
    ur5_tool_frame_conversion_inverse =
{
    .matrix =
    {
        {
            0.0F,
            0.0F,
            1.0F,
            0.0F
        },

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
            0.0F,
            1.0F
        }
    }
};


/* =========================================================
 * Internal Angle Helpers
 * ========================================================= */

/**
 * @brief 将 radian 角规范化到 [-pi, pi)。
 */
static robot_real_t
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


    return angle_rad;
}


/* =========================================================
 * Standard DH Transform
 * ========================================================= */

/**
 * @brief 构造单个 Standard DH 变换矩阵。
 *
 * Standard DH：
 *
 * A_i =
 *
 * Rz(theta)
 * Tz(d)
 * Tx(a)
 * Rx(alpha)
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
        transform
        == NULL
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


/* =========================================================
 * Public -> Standard DH Coordinate Conversion
 * ========================================================= */

/**
 * @brief 将公共 T_base_ee 转换成内部 Standard DH T_0_6。
 *
 * Forward 中：
 *
 * T_base_ee =
 *
 * C_base
 * ×
 * T_0_6
 * ×
 * C_tool
 *
 * 因此：
 *
 * T_0_6 =
 *
 * C_base^-1
 * ×
 * T_base_ee
 * ×
 * C_tool^-1
 *
 * 当前：
 *
 * C_base^-1 = C_base
 */
static robot_status_t
kinematics_public_to_dh_transform(
    const robot_transform_t *public_transform,
    robot_transform_t *dh_transform
)
{
    robot_transform_t
        converted;


    robot_status_t
        status;


    if (
        public_transform
        == NULL
        ||
        dh_transform
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * converted =
     *
     * C_base^-1
     * ×
     * T_base_ee
     *
     * 当前 C_base^-1 = C_base。
     */
    status =
        matrix4_multiply(
            &ur5_base_frame_conversion,
            public_transform,
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
     * T_0_6 =
     *
     * converted
     * ×
     * C_tool^-1
     */
    status =
        matrix4_multiply(
            &converted,
            &ur5_tool_frame_conversion_inverse,
            dh_transform
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
 * IK Stage 1
 *
 * Solve q1 Shoulder Candidates
 * ========================================================= */

/**
 * @brief 根据 Standard DH T_0_6 求两个 q1 候选。
 *
 * 先求 Wrist Point p5：
 *
 * p5 =
 *
 * p6
 * -
 * d6 * z6
 *
 * 其中：
 *
 * z6 是 T_0_6 Rotation Matrix
 * 第三列。
 *
 *
 * XY Plane：
 *
 * radius =
 *
 * sqrt(
 *     x5^2
 *     +
 *     y5^2
 * )
 *
 *
 * phi =
 *
 * atan2(
 *     y5,
 *     x5
 * )
 *
 *
 * alpha =
 *
 * asin(
 *     d4 / radius
 * )
 *
 *
 * 两个 Shoulder 解：
 *
 * q1[0] =
 *
 * phi
 * +
 * alpha
 *
 *
 * q1[1] =
 *
 * phi
 * +
 * pi
 * -
 * alpha
 *
 *
 * 两个结果均规范化到：
 *
 * [-pi, pi)
 */
static robot_status_t
kinematics_solve_q1_candidates(
    const robot_transform_t *dh_transform,
    robot_real_t q1_candidates_rad[2]
)
{
    robot_real_t
        p6_x;

    robot_real_t
        p6_y;


    robot_real_t
        z6_x;

    robot_real_t
        z6_y;


    robot_real_t
        wrist_x;

    robot_real_t
        wrist_y;


    robot_real_t
        radius_squared;

    robot_real_t
        radius;


    robot_real_t
        d4;

    robot_real_t
        d6;


    robot_real_t
        ratio;

    robot_real_t
        phi;

    robot_real_t
        alpha;


    if (
        dh_transform
        == NULL
        ||
        q1_candidates_rad
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * T_0_6 Translation：
     *
     * p6 =
     *
     * [ px ]
     * [ py ]
     * [ pz ]
     */
    p6_x =
        dh_transform->matrix[0][3];

    p6_y =
        dh_transform->matrix[1][3];


    /*
     * Rotation Matrix 第三列：
     *
     * z6 =
     *
     * [ r02 ]
     * [ r12 ]
     * [ r22 ]
     */
    z6_x =
        dh_transform->matrix[0][2];

    z6_y =
        dh_transform->matrix[1][2];


    /*
     * UR5：
     *
     * d4 = 109.15 mm
     * d6 = 82.3 mm
     */
    d4 =
        ur5_dh_d_mm[3];

    d6 =
        ur5_dh_d_mm[5];


    /*
     * Wrist Point：
     *
     * p5 =
     *
     * p6
     * -
     * d6 * z6
     *
     * q1 只需要 XY 分量。
     */
    wrist_x =
        p6_x
        -
        d6
        *
        z6_x;


    wrist_y =
        p6_y
        -
        d6
        *
        z6_y;


    radius_squared =
        wrist_x
        *
        wrist_x
        +
        wrist_y
        *
        wrist_y;


    radius =
        sqrtf(
            radius_squared
        );


    /*
     * 几何可达性：
     *
     * radius >= |d4|
     *
     * 否则：
     *
     * asin(d4 / radius)
     *
     * 不存在实数解。
     */
    if (
        radius
        +
        KINEMATICS_GEOMETRY_EPSILON_MM
        <
        fabsf(
            d4
        )
    )
    {
        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    /*
     * 理论上 ratio 位于 [-1, 1]。
     *
     * 由于浮点误差，
     * 接近边界时可能出现：
     *
     * 1.0000001
     *
     * 因此在调用 asinf() 前进行 Clamp。
     */
    ratio =
        d4
        /
        radius;


    if (
        ratio
        >
        1.0F
    )
    {
        ratio =
            1.0F;
    }


    if (
        ratio
        <
        -1.0F
    )
    {
        ratio =
            -1.0F;
    }


    phi =
        atan2f(
            wrist_y,
            wrist_x
        );


    alpha =
        asinf(
            ratio
        );


    /*
     * Shoulder Branch 1。
     */
    q1_candidates_rad[0] =
        kinematics_normalize_angle_rad(
            phi
            +
            alpha
        );


    /*
     * Shoulder Branch 2。
     */
    q1_candidates_rad[1] =
        kinematics_normalize_angle_rad(
            phi
            +
            KINEMATICS_PI_F
            -
            alpha
        );


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
        dh_joint;


    robot_transform_t
        converted;


    robot_status_t
        status;


    uint32_t
        joint_index;


    if (
        joints
        == NULL
        ||
        transform
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


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
     * Standard DH：
     *
     * T_0_6 =
     *
     * A1
     * ×
     * A2
     * ×
     * A3
     * ×
     * A4
     * ×
     * A5
     * ×
     * A6
     */
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
                ur5_dh_d_mm[
                    joint_index
                ],
                ur5_dh_a_mm[
                    joint_index
                ],
                ur5_dh_alpha_rad[
                    joint_index
                ],
                &dh_joint
            );


        if (
            status
            != ROBOT_STATUS_OK
        )
        {
            return status;
        }


        status =
            matrix4_multiply(
                &dh_total,
                &dh_joint,
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
     * Standard DH
     *
     * ->
     *
     * Public base_link Frame
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
     * DH Tool Frame
     *
     * ->
     *
     * URDF ee_link Frame
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
    robot_transform_t
        dh_transform;


    robot_real_t
        q1_candidates_rad[2];


    robot_status_t
        status;


    if (
        transform
        == NULL
        ||
        solutions
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * 当前完整 IK 尚未完成，
     * 因此不向调用者暴露任何半成品解。
     */
    solutions->count =
        0U;


    /*
     * Step 1：
     *
     * Public：
     *
     * T_base_ee
     *
     * ->
     *
     * Internal Standard DH：
     *
     * T_0_6
     */
    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
        );


    if (
        status
        != ROBOT_STATUS_OK
    )
    {
        return status;
    }


    /*
     * Step 2：
     *
     * 求两个 Shoulder q1 候选。
     *
     * 当前仅完成到这里。
     */
    status =
        kinematics_solve_q1_candidates(
            &dh_transform,
            q1_candidates_rad
        );


    if (
        status
        != ROBOT_STATUS_OK
    )
    {
        return status;
    }


    /*
     * q1_candidates_rad 当前已经包含：
     *
     * q1[0]
     * q1[1]
     *
     * 但 q2 ~ q6 尚未求解，
     * 因此现在不能构造完整 IK Solution。
     *
     * 暂时显式标记为未使用。
     */
    (void)q1_candidates_rad;


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
        solutions
        == NULL
        ||
        reference
        == NULL
        ||
        selected
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    return
        ROBOT_STATUS_ERROR_NOT_IMPLEMENTED;
}