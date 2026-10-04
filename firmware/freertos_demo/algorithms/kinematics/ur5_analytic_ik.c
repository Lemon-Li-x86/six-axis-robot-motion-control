/*
 * 文件：ur5_analytic_ik.c
 *
 * 用途：
 * 实现 UR5 Analytic IK 的各阶段数学求解器。
 *
 * 本文件只负责：
 *
 * q1
 * q5
 * q6
 * q3
 * q2
 * q4
 *
 * 不负责最终 Branch Assembly。
 */

#include <stddef.h>
#include <math.h>

#include "kinematics_private.h"
#include "kinematics_internal.h"

#include "robot_model.h"


#define KINEMATICS_GEOMETRY_EPSILON_MM_SQUARED \
    0.01F

#define KINEMATICS_TRIG_DOMAIN_EPSILON \
    0.00001F

#define KINEMATICS_SINGULARITY_EPSILON \
    0.0001F

#define KINEMATICS_ORIENTATION_EPSILON_SQUARED \
    0.00000001F


/* =========================================================
 * Shared UR5 IK Geometry
 * ========================================================= */

static robot_status_t
ur5_analytic_ik_compute_p13(
    const robot_model_t *model,
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
        model
        ==
        NULL
        ||
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
        model->dh_d_mm[0];

    d5 =
        model->dh_d_mm[4];

    d6 =
        model->dh_d_mm[5];


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
 * q1 Shoulder
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q1_candidates(
    const robot_model_t *model,
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
        model
        ==
        NULL
        ||
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
        model->dh_d_mm[3];

    d6 =
        model->dh_d_mm[5];


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
 * q5 Wrist
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q5_candidates(
    const robot_model_t *model,
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
        model
        ==
        NULL
        ||
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
        model->dh_d_mm[3];

    d6 =
        model->dh_d_mm[5];


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
 * q6 Wrist
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q6(
    const robot_model_t *model,
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


    (void)model;


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


    sin_q5_sign =
        (
            sin_q5
            >
            0.0F
        )
        ?
        1.0F
        :
        -1.0F;


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
 * q3 Elbow
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q3_candidates(
    const robot_model_t *model,
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
        model
        ==
        NULL
        ||
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
        ur5_analytic_ik_compute_p13(
            model,
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
        model->dh_a_mm[1];

    a3 =
        model->dh_a_mm[2];


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
 * q2 Shoulder Lift
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q2(
    const robot_model_t *model,
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
        model
        ==
        NULL
        ||
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
        ur5_analytic_ik_compute_p13(
            model,
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
        model->dh_a_mm[1];

    a3 =
        model->dh_a_mm[2];


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
 * q4 Wrist 1
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q4(
    const robot_model_t *model,
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


    (void)model;


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
 * Internal Validation Wrappers
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q1_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_candidates_rad[2]
)
{
    const robot_model_t
        *model;


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


    model =
        robot_model_get_active();


    status =
        kinematics_public_to_dh_transform(
            model,
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
        ur5_analytic_ik_solve_q1_candidates(
            model,
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
    const robot_model_t
        *model;


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


    model =
        robot_model_get_active();


    status =
        kinematics_public_to_dh_transform(
            model,
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
        ur5_analytic_ik_solve_q5_candidates(
            model,
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
    const robot_model_t
        *model;


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


    model =
        robot_model_get_active();


    status =
        kinematics_public_to_dh_transform(
            model,
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
        ur5_analytic_ik_solve_q6(
            model,
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
    const robot_model_t
        *model;


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


    model =
        robot_model_get_active();


    status =
        kinematics_public_to_dh_transform(
            model,
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
        ur5_analytic_ik_solve_q3_candidates(
            model,
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
    const robot_model_t
        *model;


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


    model =
        robot_model_get_active();


    status =
        kinematics_public_to_dh_transform(
            model,
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
        ur5_analytic_ik_solve_q2(
            model,
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
    const robot_model_t
        *model;


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


    model =
        robot_model_get_active();


    status =
        kinematics_public_to_dh_transform(
            model,
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
        ur5_analytic_ik_solve_q4(
            model,
            &dh_transform,
            q1_rad,
            q2_rad,
            q3_rad,
            q5_rad,
            q6_rad,
            q4_rad
        );
}