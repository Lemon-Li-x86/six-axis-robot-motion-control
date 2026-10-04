/*
 * 文件：kinematics.c
 *
 * 用途：
 * 实现 UR5 六轴机器人运动学算法。
 *
 * 当前已实现：
 *
 * 1. Standard DH Forward Kinematics；
 * 2. Public / DH 坐标转换；
 *
 * Analytic IK：
 *
 * 3. q1 Shoulder x 2；
 * 4. q5 Wrist x 2；
 * 5. q6；
 * 6. Wrist Singularity Detection；
 * 7. q3 Elbow x 2；
 * 8. q2；
 * 9. q4；
 * 10. 最多 8 个完整六轴 Branch；
 * 11. radian -> Canonical 0.01 degree；
 * 12. kinematics_ik_solutions_t Assembly。
 *
 * 尚未完成：
 *
 * 1. Joint Limit Filtering；
 * 2. Duplicate Removal；
 * 3. 全部 IK 解 FK Round-Trip Validation；
 * 4. IK Solution Selection。
 */


#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "kinematics.h"
#include "kinematics_internal.h"
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

#define KINEMATICS_RAD_TO_DEG_F \
    (180.0F / KINEMATICS_PI_F)


#define KINEMATICS_GEOMETRY_EPSILON_MM_SQUARED \
    0.01F


#define KINEMATICS_TRIG_DOMAIN_EPSILON \
    0.00001F


#define KINEMATICS_SINGULARITY_EPSILON \
    0.0001F


#define KINEMATICS_ORIENTATION_EPSILON_SQUARED \
    0.00000001F


/* =========================================================
 * UR5 Standard DH Parameters
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
 * ========================================================= */

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
 * Internal Math Helpers
 * ========================================================= */

static robot_real_t
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


    return
        angle_rad;
}


/*
 * 将 integer centi-degree 规范化到：
 *
 * [-18000, 17999]
 */
static int32_t
kinematics_normalize_joint_raw(
    int32_t raw
)
{
    while (
        raw
        >
        ROBOT_JOINT_ANGLE_MAX_RAW
    )
    {
        raw -=
            ROBOT_JOINT_FULL_TURN_RAW;
    }


    while (
        raw
        <
        ROBOT_JOINT_ANGLE_MIN_RAW
    )
    {
        raw +=
            ROBOT_JOINT_FULL_TURN_RAW;
    }


    return
        raw;
}


/**
 * @brief radian -> Canonical Joint Raw。
 *
 * 输出：
 *
 * 1 unit = 0.01 degree
 *
 * Range：
 *
 * [-18000, 17999]
 */
static robot_joint_angle_t
kinematics_angle_rad_to_joint_raw(
    robot_real_t angle_rad
)
{
    robot_real_t
        normalized_rad;


    robot_real_t
        degree;


    robot_real_t
        scaled_raw;


    int32_t
        rounded_raw;


    normalized_rad =
        kinematics_normalize_angle_rad(
            angle_rad
        );


    degree =
        normalized_rad
        *
        KINEMATICS_RAD_TO_DEG_F;


    scaled_raw =
        degree
        /
        ROBOT_JOINT_ANGLE_UNIT_DEG;


    /*
     * 不调用 roundf()，
     * 避免额外 C Library 依赖。
     *
     * C integer cast 向 0 截断，
     * 因此：
     *
     * positive：
     * +0.5
     *
     * negative：
     * -0.5
     */
    if (
        scaled_raw
        >=
        0.0F
    )
    {
        rounded_raw =
            (int32_t)
            (
                scaled_raw
                +
                0.5F
            );
    }
    else
    {
        rounded_raw =
            (int32_t)
            (
                scaled_raw
                -
                0.5F
            );
    }


    rounded_raw =
        kinematics_normalize_joint_raw(
            rounded_raw
        );


    return
        (robot_joint_angle_t)
        rounded_raw;
}


/*
 * Freestanding Square Root。
 */
static robot_real_t
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


    if (
        value
        >=
        1.0F
    )
    {
        estimate =
            value;
    }
    else
    {
        estimate =
            1.0F;
    }


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
 * Solution Assembly Helper
 * ========================================================= */

/**
 * @brief 将一组完整 radian IK Branch
 *        写入公共 Solution Array。
 */
static robot_status_t
kinematics_append_solution(
    kinematics_ik_solutions_t *solutions,
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q4_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad
)
{
    robot_joint_angles_t
        *solution;


    if (
        solutions
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
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


    solution =
        &solutions->solutions[
            solutions->count
        ];


    solution->value[0] =
        kinematics_angle_rad_to_joint_raw(
            q1_rad
        );


    solution->value[1] =
        kinematics_angle_rad_to_joint_raw(
            q2_rad
        );


    solution->value[2] =
        kinematics_angle_rad_to_joint_raw(
            q3_rad
        );


    solution->value[3] =
        kinematics_angle_rad_to_joint_raw(
            q4_rad
        );


    solution->value[4] =
        kinematics_angle_rad_to_joint_raw(
            q5_rad
        );


    solution->value[5] =
        kinematics_angle_rad_to_joint_raw(
            q6_rad
        );


    solutions->count++;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Standard DH Transform
 * ========================================================= */

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


/* =========================================================
 * Public -> Standard DH
 * ========================================================= */

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
            &ur5_base_frame_conversion,
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


    status =
        matrix4_multiply(
            &converted,
            &ur5_tool_frame_conversion_inverse,
            dh_transform
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
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Shared IK Geometry
 * ========================================================= */

static robot_status_t
kinematics_compute_p13(
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q6_rad,
    robot_real_t *p13_x,
    robot_real_t *p13_y
)
{
    robot_real_t
        sin_q1;

    robot_real_t
        cos_q1;


    robot_real_t
        sin_q6;

    robot_real_t
        cos_q6;


    robot_real_t
        d1;

    robot_real_t
        d5;

    robot_real_t
        d6;


    if (
        dh_transform
        ==
        NULL
        ||
        p13_x
        ==
        NULL
        ||
        p13_y
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    sin_q1 =
        sinf(
            q1_rad
        );


    cos_q1 =
        cosf(
            q1_rad
        );


    sin_q6 =
        sinf(
            q6_rad
        );


    cos_q6 =
        cosf(
            q6_rad
        );


    d1 =
        ur5_dh_d_mm[0];

    d5 =
        ur5_dh_d_mm[4];

    d6 =
        ur5_dh_d_mm[5];


    *p13_x =
        d5
        *
        (
            sin_q6
            *
            (
                dh_transform->matrix[0][0]
                *
                cos_q1
                +
                dh_transform->matrix[1][0]
                *
                sin_q1
            )
            +
            cos_q6
            *
            (
                dh_transform->matrix[0][1]
                *
                cos_q1
                +
                dh_transform->matrix[1][1]
                *
                sin_q1
            )
        )
        -
        d6
        *
        (
            dh_transform->matrix[0][2]
            *
            cos_q1
            +
            dh_transform->matrix[1][2]
            *
            sin_q1
        )
        +
        dh_transform->matrix[0][3]
        *
        cos_q1
        +
        dh_transform->matrix[1][3]
        *
        sin_q1;


    *p13_y =
        dh_transform->matrix[2][3]
        -
        d1
        -
        d6
        *
        dh_transform->matrix[2][2]
        +
        d5
        *
        (
            dh_transform->matrix[2][1]
            *
            cos_q6
            +
            dh_transform->matrix[2][0]
            *
            sin_q6
        );


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * IK Stage 1
 * q1
 * ========================================================= */

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
        d4_squared;


    robot_real_t
        radial_component_squared;

    robot_real_t
        radial_component;


    robot_real_t
        d4;

    robot_real_t
        d6;


    robot_real_t
        phi;

    robot_real_t
        alpha;


    if (
        dh_transform
        ==
        NULL
        ||
        q1_candidates_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    p6_x =
        dh_transform->matrix[0][3];

    p6_y =
        dh_transform->matrix[1][3];


    z6_x =
        dh_transform->matrix[0][2];

    z6_y =
        dh_transform->matrix[1][2];


    d4 =
        ur5_dh_d_mm[3];

    d6 =
        ur5_dh_d_mm[5];


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


    d4_squared =
        d4
        *
        d4;


    if (
        radius_squared
        +
        KINEMATICS_GEOMETRY_EPSILON_MM_SQUARED
        <
        d4_squared
    )
    {
        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    radial_component_squared =
        radius_squared
        -
        d4_squared;


    if (
        radial_component_squared
        <
        0.0F
    )
    {
        radial_component_squared =
            0.0F;
    }


    radial_component =
        kinematics_sqrt_nonnegative(
            radial_component_squared
        );


    phi =
        atan2f(
            wrist_y,
            wrist_x
        );


    alpha =
        atan2f(
            d4,
            radial_component
        );


    q1_candidates_rad[0] =
        kinematics_normalize_angle_rad(
            phi
            +
            alpha
        );


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
 * IK Stage 2
 * q5
 * ========================================================= */

static robot_status_t
kinematics_solve_q5_candidates(
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
)
{
    robot_real_t
        p6_x;

    robot_real_t
        p6_y;


    robot_real_t
        d4;

    robot_real_t
        d6;


    robot_real_t
        sin_q1;

    robot_real_t
        cos_q1;


    robot_real_t
        cos_q5;

    robot_real_t
        sin_q5_squared;

    robot_real_t
        sin_q5_absolute;


    if (
        dh_transform
        ==
        NULL
        ||
        q5_candidates_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    p6_x =
        dh_transform->matrix[0][3];

    p6_y =
        dh_transform->matrix[1][3];


    d4 =
        ur5_dh_d_mm[3];

    d6 =
        ur5_dh_d_mm[5];


    sin_q1 =
        sinf(
            q1_rad
        );


    cos_q1 =
        cosf(
            q1_rad
        );


    cos_q5 =
        (
            p6_x
            *
            sin_q1
            -
            p6_y
            *
            cos_q1
            -
            d4
        )
        /
        d6;


    if (
        cos_q5
        >
        1.0F
        +
        KINEMATICS_TRIG_DOMAIN_EPSILON
        ||
        cos_q5
        <
        -1.0F
        -
        KINEMATICS_TRIG_DOMAIN_EPSILON
    )
    {
        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    if (
        cos_q5
        >
        1.0F
    )
    {
        cos_q5 =
            1.0F;
    }


    if (
        cos_q5
        <
        -1.0F
    )
    {
        cos_q5 =
            -1.0F;
    }


    sin_q5_squared =
        1.0F
        -
        cos_q5
        *
        cos_q5;


    if (
        sin_q5_squared
        <
        0.0F
    )
    {
        sin_q5_squared =
            0.0F;
    }


    sin_q5_absolute =
        kinematics_sqrt_nonnegative(
            sin_q5_squared
        );


    q5_candidates_rad[0] =
        kinematics_normalize_angle_rad(
            atan2f(
                sin_q5_absolute,
                cos_q5
            )
        );


    q5_candidates_rad[1] =
        kinematics_normalize_angle_rad(
            atan2f(
                -sin_q5_absolute,
                cos_q5
            )
        );


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * IK Stage 3
 * q6
 * ========================================================= */

static robot_status_t
kinematics_solve_q6(
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
)
{
    robot_real_t
        sin_q1;

    robot_real_t
        cos_q1;

    robot_real_t
        sin_q5;


    robot_real_t
        sin_q5_sign;


    robot_real_t
        atan_y;

    robot_real_t
        atan_x;


    if (
        dh_transform
        ==
        NULL
        ||
        q6_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    sin_q5 =
        sinf(
            q5_rad
        );


    if (
        kinematics_abs_real(
            sin_q5
        )
        <
        KINEMATICS_SINGULARITY_EPSILON
    )
    {
        return
            ROBOT_STATUS_ERROR_SINGULAR;
    }


    sin_q1 =
        sinf(
            q1_rad
        );


    cos_q1 =
        cosf(
            q1_rad
        );


    if (
        sin_q5
        >
        0.0F
    )
    {
        sin_q5_sign =
            1.0F;
    }
    else
    {
        sin_q5_sign =
            -1.0F;
    }


    atan_y =
        sin_q5_sign
        *
        (
            -dh_transform->matrix[0][1]
            *
            sin_q1
            +
            dh_transform->matrix[1][1]
            *
            cos_q1
        );


    atan_x =
        sin_q5_sign
        *
        (
            dh_transform->matrix[0][0]
            *
            sin_q1
            -
            dh_transform->matrix[1][0]
            *
            cos_q1
        );


    *q6_rad =
        kinematics_normalize_angle_rad(
            atan2f(
                atan_y,
                atan_x
            )
        );


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * IK Stage 4
 * q3
 * ========================================================= */

static robot_status_t
kinematics_solve_q3_candidates(
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
)
{
    robot_real_t
        p13_x;

    robot_real_t
        p13_y;


    robot_real_t
        a2;

    robot_real_t
        a3;


    robot_real_t
        cos_q3;

    robot_real_t
        sin_q3_squared;

    robot_real_t
        sin_q3_absolute;


    robot_status_t
        status;


    (void)q5_rad;


    if (
        dh_transform
        ==
        NULL
        ||
        q3_candidates_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_compute_p13(
            dh_transform,
            q1_rad,
            q6_rad,
            &p13_x,
            &p13_y
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


    a2 =
        ur5_dh_a_mm[1];

    a3 =
        ur5_dh_a_mm[2];


    cos_q3 =
        (
            p13_x
            *
            p13_x
            +
            p13_y
            *
            p13_y
            -
            a2
            *
            a2
            -
            a3
            *
            a3
        )
        /
        (
            2.0F
            *
            a2
            *
            a3
        );


    if (
        cos_q3
        >
        1.0F
        +
        KINEMATICS_TRIG_DOMAIN_EPSILON
        ||
        cos_q3
        <
        -1.0F
        -
        KINEMATICS_TRIG_DOMAIN_EPSILON
    )
    {
        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    if (
        cos_q3
        >
        1.0F
    )
    {
        cos_q3 =
            1.0F;
    }


    if (
        cos_q3
        <
        -1.0F
    )
    {
        cos_q3 =
            -1.0F;
    }


    sin_q3_squared =
        1.0F
        -
        cos_q3
        *
        cos_q3;


    if (
        sin_q3_squared
        <
        0.0F
    )
    {
        sin_q3_squared =
            0.0F;
    }


    sin_q3_absolute =
        kinematics_sqrt_nonnegative(
            sin_q3_squared
        );


    q3_candidates_rad[0] =
        kinematics_normalize_angle_rad(
            atan2f(
                sin_q3_absolute,
                cos_q3
            )
        );


    q3_candidates_rad[1] =
        kinematics_normalize_angle_rad(
            atan2f(
                -sin_q3_absolute,
                cos_q3
            )
        );


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * IK Stage 5
 * q2
 * ========================================================= */

static robot_status_t
kinematics_solve_q2(
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
)
{
    robot_real_t
        p13_x;

    robot_real_t
        p13_y;


    robot_real_t
        a2;

    robot_real_t
        a3;


    robot_real_t
        cos_q3;

    robot_real_t
        sin_q3;


    robot_real_t
        coefficient_a;

    robot_real_t
        coefficient_b;


    robot_real_t
        atan_y;

    robot_real_t
        atan_x;


    robot_real_t
        radius_squared;


    robot_status_t
        status;


    (void)q5_rad;


    if (
        dh_transform
        ==
        NULL
        ||
        q2_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_compute_p13(
            dh_transform,
            q1_rad,
            q6_rad,
            &p13_x,
            &p13_y
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


    radius_squared =
        p13_x
        *
        p13_x
        +
        p13_y
        *
        p13_y;


    if (
        radius_squared
        <
        KINEMATICS_GEOMETRY_EPSILON_MM_SQUARED
    )
    {
        return
            ROBOT_STATUS_ERROR_SINGULAR;
    }


    a2 =
        ur5_dh_a_mm[1];

    a3 =
        ur5_dh_a_mm[2];


    cos_q3 =
        cosf(
            q3_rad
        );


    sin_q3 =
        sinf(
            q3_rad
        );


    coefficient_a =
        a2
        +
        a3
        *
        cos_q3;


    coefficient_b =
        a3
        *
        sin_q3;


    atan_y =
        coefficient_a
        *
        p13_y
        -
        coefficient_b
        *
        p13_x;


    atan_x =
        coefficient_a
        *
        p13_x
        +
        coefficient_b
        *
        p13_y;


    *q2_rad =
        kinematics_normalize_angle_rad(
            atan2f(
                atan_y,
                atan_x
            )
        );


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * IK Stage 6
 * q4
 * ========================================================= */

static robot_status_t
kinematics_solve_q4(
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t *q4_rad
)
{
    robot_real_t
        sin_q1;

    robot_real_t
        cos_q1;


    robot_real_t
        sin_q5;

    robot_real_t
        cos_q5;


    robot_real_t
        sin_q6;

    robot_real_t
        cos_q6;


    robot_real_t
        q23_rad;

    robot_real_t
        sin_q23;

    robot_real_t
        cos_q23;


    robot_real_t
        x04_x;

    robot_real_t
        x04_y;


    robot_real_t
        atan_y;

    robot_real_t
        atan_x;


    robot_real_t
        orientation_squared;


    if (
        dh_transform
        ==
        NULL
        ||
        q4_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    sin_q1 =
        sinf(
            q1_rad
        );


    cos_q1 =
        cosf(
            q1_rad
        );


    sin_q5 =
        sinf(
            q5_rad
        );


    cos_q5 =
        cosf(
            q5_rad
        );


    sin_q6 =
        sinf(
            q6_rad
        );


    cos_q6 =
        cosf(
            q6_rad
        );


    x04_x =
        -sin_q5
        *
        (
            dh_transform->matrix[0][2]
            *
            cos_q1
            +
            dh_transform->matrix[1][2]
            *
            sin_q1
        )
        -
        cos_q5
        *
        (
            sin_q6
            *
            (
                dh_transform->matrix[0][1]
                *
                cos_q1
                +
                dh_transform->matrix[1][1]
                *
                sin_q1
            )
            -
            cos_q6
            *
            (
                dh_transform->matrix[0][0]
                *
                cos_q1
                +
                dh_transform->matrix[1][0]
                *
                sin_q1
            )
        );


    x04_y =
        cos_q5
        *
        (
            dh_transform->matrix[2][0]
            *
            cos_q6
            -
            dh_transform->matrix[2][1]
            *
            sin_q6
        )
        -
        dh_transform->matrix[2][2]
        *
        sin_q5;


    orientation_squared =
        x04_x
        *
        x04_x
        +
        x04_y
        *
        x04_y;


    if (
        orientation_squared
        <
        KINEMATICS_ORIENTATION_EPSILON_SQUARED
    )
    {
        return
            ROBOT_STATUS_ERROR_SINGULAR;
    }


    q23_rad =
        q2_rad
        +
        q3_rad;


    sin_q23 =
        sinf(
            q23_rad
        );


    cos_q23 =
        cosf(
            q23_rad
        );


    atan_y =
        cos_q23
        *
        x04_y
        -
        sin_q23
        *
        x04_x;


    atan_x =
        x04_x
        *
        cos_q23
        +
        x04_y
        *
        sin_q23;


    *q4_rad =
        kinematics_normalize_angle_rad(
            atan2f(
                atan_y,
                atan_x
            )
        );


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Internal Validation API
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q1_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_candidates_rad[2]
)
{
    robot_transform_t
        dh_transform;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        q1_candidates_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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
        kinematics_solve_q1_candidates(
            &dh_transform,
            q1_candidates_rad
        );
}


robot_status_t
kinematics_internal_solve_q5_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
)
{
    robot_transform_t
        dh_transform;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        q5_candidates_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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
        kinematics_solve_q5_candidates(
            &dh_transform,
            q1_rad,
            q5_candidates_rad
        );
}


robot_status_t
kinematics_internal_solve_q6(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
)
{
    robot_transform_t
        dh_transform;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        q6_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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
        kinematics_solve_q6(
            &dh_transform,
            q1_rad,
            q5_rad,
            q6_rad
        );
}


robot_status_t
kinematics_internal_solve_q3_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
)
{
    robot_transform_t
        dh_transform;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        q3_candidates_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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
        kinematics_solve_q3_candidates(
            &dh_transform,
            q1_rad,
            q5_rad,
            q6_rad,
            q3_candidates_rad
        );
}


robot_status_t
kinematics_internal_solve_q2(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
)
{
    robot_transform_t
        dh_transform;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        q2_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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
        kinematics_solve_q2(
            &dh_transform,
            q1_rad,
            q5_rad,
            q6_rad,
            q3_rad,
            q2_rad
        );
}


robot_status_t
kinematics_internal_solve_q4(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t *q4_rad
)
{
    robot_transform_t
        dh_transform;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        q4_rad
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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
        kinematics_solve_q4(
            &dh_transform,
            q1_rad,
            q2_rad,
            q3_rad,
            q5_rad,
            q6_rad,
            q4_rad
        );
}


/* =========================================================
 * Forward Kinematics
 * ========================================================= */

robot_status_t
kinematics_forward(
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
            &ur5_base_frame_conversion,
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


    status =
        matrix4_multiply(
            &converted,
            &ur5_tool_frame_conversion,
            transform
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
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Inverse Kinematics
 * ========================================================= */

robot_status_t
kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions
)
{
    robot_transform_t
        dh_transform;


    robot_real_t
        q1_candidates_rad[2];


    robot_real_t
        q5_candidates_rad[2][2];


    robot_real_t
        q6_candidates_rad[2][2];


    robot_real_t
        q3_candidates_rad[2][2][2];


    robot_real_t
        q2_candidates_rad[2][2][2];


    robot_real_t
        q4_candidates_rad[2][2][2];


    uint8_t
        q5_branch_valid[2] =
        {
            0U,
            0U
        };


    uint8_t
        q6_branch_valid[2][2] =
        {
            {
                0U,
                0U
            },

            {
                0U,
                0U
            }
        };


    uint8_t
        q3_pair_valid[2][2] =
        {
            {
                0U,
                0U
            },

            {
                0U,
                0U
            }
        };


    uint8_t
        q2_branch_valid[2][2][2] =
        {
            {
                {
                    0U,
                    0U
                },

                {
                    0U,
                    0U
                }
            },

            {
                {
                    0U,
                    0U
                },

                {
                    0U,
                    0U
                }
            }
        };


    uint8_t
        q4_branch_valid[2][2][2] =
        {
            {
                {
                    0U,
                    0U
                },

                {
                    0U,
                    0U
                }
            },

            {
                {
                    0U,
                    0U
                },

                {
                    0U,
                    0U
                }
            }
        };


    uint32_t
        shoulder_index;

    uint32_t
        wrist_index;

    uint32_t
        elbow_index;


    uint32_t
        valid_q4_branch_count =
            0U;


    uint32_t
        singular_branch_count =
            0U;


    robot_status_t
        status;


    if (
        transform
        ==
        NULL
        ||
        solutions
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    solutions->count =
        0U;


    /* =====================================================
     * Public -> Standard DH
     * ===================================================== */

    status =
        kinematics_public_to_dh_transform(
            transform,
            &dh_transform
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


    /* =====================================================
     * Stage 1
     * q1 x 2
     * ===================================================== */

    status =
        kinematics_solve_q1_candidates(
            &dh_transform,
            q1_candidates_rad
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


    /* =====================================================
     * Stage 2
     * q5 x 2
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        status =
            kinematics_solve_q5_candidates(
                &dh_transform,
                q1_candidates_rad[
                    shoulder_index
                ],
                q5_candidates_rad[
                    shoulder_index
                ]
            );


        if (
            status
            ==
            ROBOT_STATUS_ERROR_NO_SOLUTION
        )
        {
            continue;
        }


        if (
            status
            !=
            ROBOT_STATUS_OK
        )
        {
            return
                status;
        }


        q5_branch_valid[
            shoulder_index
        ] =
            1U;
    }


    /* =====================================================
     * Stage 3
     * q6
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        if (
            !q5_branch_valid[
                shoulder_index
            ]
        )
        {
            continue;
        }


        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            status =
                kinematics_solve_q6(
                    &dh_transform,
                    q1_candidates_rad[
                        shoulder_index
                    ],
                    q5_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],
                    &q6_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ]
                );


            if (
                status
                ==
                ROBOT_STATUS_ERROR_SINGULAR
            )
            {
                singular_branch_count++;

                continue;
            }


            if (
                status
                !=
                ROBOT_STATUS_OK
            )
            {
                return
                    status;
            }


            q6_branch_valid[
                shoulder_index
            ][
                wrist_index
            ] =
                1U;
        }
    }


    /* =====================================================
     * Stage 4
     * q3 x 2
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            if (
                !q6_branch_valid[
                    shoulder_index
                ][
                    wrist_index
                ]
            )
            {
                continue;
            }


            status =
                kinematics_solve_q3_candidates(
                    &dh_transform,
                    q1_candidates_rad[
                        shoulder_index
                    ],
                    q5_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],
                    q6_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ],
                    q3_candidates_rad[
                        shoulder_index
                    ][
                        wrist_index
                    ]
                );


            if (
                status
                ==
                ROBOT_STATUS_ERROR_NO_SOLUTION
            )
            {
                continue;
            }


            if (
                status
                !=
                ROBOT_STATUS_OK
            )
            {
                return
                    status;
            }


            q3_pair_valid[
                shoulder_index
            ][
                wrist_index
            ] =
                1U;
        }
    }


    /* =====================================================
     * Stage 5
     * q2
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            if (
                !q3_pair_valid[
                    shoulder_index
                ][
                    wrist_index
                ]
            )
            {
                continue;
            }


            for (
                elbow_index = 0U;
                elbow_index < 2U;
                elbow_index++
            )
            {
                status =
                    kinematics_solve_q2(
                        &dh_transform,
                        q1_candidates_rad[
                            shoulder_index
                        ],
                        q5_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ],
                        q6_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ],
                        q3_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ],
                        &q2_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ]
                    );


                if (
                    status
                    ==
                    ROBOT_STATUS_ERROR_SINGULAR
                )
                {
                    singular_branch_count++;

                    continue;
                }


                if (
                    status
                    !=
                    ROBOT_STATUS_OK
                )
                {
                    return
                        status;
                }


                q2_branch_valid[
                    shoulder_index
                ][
                    wrist_index
                ][
                    elbow_index
                ] =
                    1U;
            }
        }
    }


    /* =====================================================
     * Stage 6
     * q4
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            for (
                elbow_index = 0U;
                elbow_index < 2U;
                elbow_index++
            )
            {
                if (
                    !q2_branch_valid[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ]
                )
                {
                    continue;
                }


                status =
                    kinematics_solve_q4(
                        &dh_transform,

                        q1_candidates_rad[
                            shoulder_index
                        ],

                        q2_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ],

                        q3_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ],

                        q5_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ],

                        q6_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ],

                        &q4_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ]
                    );


                if (
                    status
                    ==
                    ROBOT_STATUS_ERROR_SINGULAR
                )
                {
                    singular_branch_count++;

                    continue;
                }


                if (
                    status
                    !=
                    ROBOT_STATUS_OK
                )
                {
                    return
                        status;
                }


                q4_branch_valid[
                    shoulder_index
                ][
                    wrist_index
                ][
                    elbow_index
                ] =
                    1U;


                valid_q4_branch_count++;
            }
        }
    }


    /*
     * 一个完整非奇异 Branch 都没有。
     */
    if (
        valid_q4_branch_count
        ==
        0U
    )
    {
        if (
            singular_branch_count
            >
            0U
        )
        {
            return
                ROBOT_STATUS_ERROR_SINGULAR;
        }


        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    /* =====================================================
     * Stage 7
     * Complete Solution Assembly
     *
     * float radian branch
     *
     * ->
     *
     * Canonical 0.01 degree
     *
     * ->
     *
     * solutions[]
     * ===================================================== */

    for (
        shoulder_index = 0U;
        shoulder_index < 2U;
        shoulder_index++
    )
    {
        for (
            wrist_index = 0U;
            wrist_index < 2U;
            wrist_index++
        )
        {
            for (
                elbow_index = 0U;
                elbow_index < 2U;
                elbow_index++
            )
            {
                if (
                    !q4_branch_valid[
                        shoulder_index
                    ][
                        wrist_index
                    ][
                        elbow_index
                    ]
                )
                {
                    continue;
                }


                status =
                    kinematics_append_solution(
                        solutions,

                        q1_candidates_rad[
                            shoulder_index
                        ],

                        q2_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ],

                        q3_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ],

                        q4_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ][
                            elbow_index
                        ],

                        q5_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ],

                        q6_candidates_rad[
                            shoulder_index
                        ][
                            wrist_index
                        ]
                    );


                if (
                    status
                    !=
                    ROBOT_STATUS_OK
                )
                {
                    solutions->count =
                        0U;

                    return
                        status;
                }
            }
        }
    }


    /*
     * 理论保护。
     */
    if (
        solutions->count
        ==
        0U
    )
    {
        if (
            singular_branch_count
            >
            0U
        )
        {
            return
                ROBOT_STATUS_ERROR_SINGULAR;
        }


        return
            ROBOT_STATUS_ERROR_NO_SOLUTION;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * IK Solution Selection
 * ========================================================= */

robot_status_t
kinematics_select_best_solution(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *reference,
    robot_joint_angles_t *selected
)
{
    if (
        solutions
        ==
        NULL
        ||
        reference
        ==
        NULL
        ||
        selected
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    return
        ROBOT_STATUS_ERROR_NOT_IMPLEMENTED;
}