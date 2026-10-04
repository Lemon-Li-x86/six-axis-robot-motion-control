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
 * 3. Public / DH 坐标转换；
 *
 * IK：
 *
 * 4. q1 Shoulder 两个候选；
 * 5. q5 Wrist 两个候选；
 * 6. q6 Wrist；
 * 7. Wrist Singularity 检测；
 * 8. q3 Elbow 两个候选；
 * 9. q2 Shoulder Lift；
 * 10. q4 Wrist 1；
 * 11. 最多 8 组完整 IK Solution Assembly；
 * 12. [-180°, 180°) Canonical Angle；
 * 13. 0.01° int16_t 量化；
 * 14. Duplicate Solution Removal。
 *
 * 尚未完成：
 *
 * 1. Joint Limit Filtering；
 * 2. IK -> FK Round-Trip Verification；
 * 3. 更完整的奇异位形策略；
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


/*
 * Canonical Joint Angle：
 *
 * [-180°, 180°)
 *
 * robot_joint_angles_t：
 *
 * 0.01 degree / unit
 */
#define KINEMATICS_HALF_TURN_JOINT_UNITS \
    18000L

#define KINEMATICS_FULL_TURN_JOINT_UNITS \
    36000L


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


/**
 * @brief 将 radian 规范化到 [-pi, pi)。
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


    return
        angle_rad;
}


/**
 * @brief 非负平方根。
 *
 * 使用 Newton-Raphson，
 * 避免额外 sqrtf / errno 依赖。
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
 * Joint Angle Conversion
 * ========================================================= */

/**
 * @brief
 * radian
 *
 * ->
 *
 * [-180°, 180°)
 *
 * ->
 *
 * 0.01 degree / unit
 *
 * ->
 *
 * int16_t
 */
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


    /*
     * 避免 roundf()。
     *
     * 正数：
     *
     * +0.5 后截断。
     *
     * 负数：
     *
     * -0.5 后截断。
     */
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


    /*
     * 浮点量化可能把：
     *
     * 179.99999°
     *
     * 四舍五入成：
     *
     * +18000
     *
     * 但 Canonical Range 要求：
     *
     * [-18000, 18000)
     *
     * 因此再次整数规范化。
     */
    while (
        rounded_value
        >=
        KINEMATICS_HALF_TURN_JOINT_UNITS
    )
    {
        rounded_value -=
            KINEMATICS_FULL_TURN_JOINT_UNITS;
    }


    while (
        rounded_value
        <
        -KINEMATICS_HALF_TURN_JOINT_UNITS
    )
    {
        rounded_value +=
            KINEMATICS_FULL_TURN_JOINT_UNITS;
    }


    return
        (int16_t)
        rounded_value;
}


/* =========================================================
 * Solution Helpers
 * ========================================================= */

static void
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


/**
 * @brief 将 Candidate 加入 Solution Set。
 *
 * 如果 Candidate 在 0.01° 量化后
 * 已经与现有解完全一致，
 * 则视为 Duplicate 并忽略。
 */
static robot_status_t
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
            /*
             * Duplicate：
             *
             * 不报错，
             * 只是不要重复保存。
             */
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
 * p13
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
 * q1 Shoulder
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
 * q5 Wrist
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
 * q6 Wrist
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
 * q3 Elbow
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
 * q2 Shoulder Lift
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
 * q4 Wrist 1
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


    uint32_t
        shoulder_index;

    uint32_t
        wrist_index;

    uint32_t
        elbow_index;


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


    /*
     * 每次调用从空集合开始。
     */
    solutions->count =
        0U;


    /* =====================================================
     * Public T_base_ee
     *
     * ->
     *
     * Standard DH T_0_6
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
     *
     * +
     *
     * Complete Solution Assembly
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
                robot_real_t
                    q4_rad;


                robot_joint_angles_t
                    candidate;


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

                        &q4_rad
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


                /*
                 * 完整 Branch：
                 *
                 * [
                 *   q1,
                 *   q2,
                 *   q3,
                 *   q4,
                 *   q5,
                 *   q6
                 * ]
                 *
                 * radian
                 *
                 * ->
                 *
                 * Canonical 0.01°
                 */
                kinematics_build_joint_solution(
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

                    q4_rad,

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

                    &candidate
                );


                /*
                 * 添加到最终 Solution Set。
                 *
                 * 量化后相同的解自动去重。
                 */
                status =
                    kinematics_add_solution_if_unique(
                        solutions,
                        &candidate
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
        }
    }


    /* =====================================================
     * Final Result
     * ===================================================== */

    if (
        solutions->count
        >
        0U
    )
    {
        /*
         * 即使某些 Branch Singular，
         * 只要还有正常合法 Branch，
         * 公共接口就返回正常解集合。
         */
        return
            ROBOT_STATUS_OK;
    }


    /*
     * 没有普通解，
     * 但至少发现了奇异 Branch。
     */
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