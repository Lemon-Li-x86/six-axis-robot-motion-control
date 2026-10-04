/*
 * 文件：kinematics_private.h
 *
 * 用途：
 * 定义 Kinematics Module 内部共享接口。
 *
 * Public Application Code
 * 不应 include 本文件。
 */

#ifndef KINEMATICS_PRIVATE_H
#define KINEMATICS_PRIVATE_H

#include <stdint.h>

#include "kinematics.h"
#include "robot_model.h"


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


/* =========================================================
 * Shared Math Helpers
 * ========================================================= */

robot_real_t
kinematics_abs_real(
    robot_real_t value
);


robot_real_t
kinematics_normalize_angle_rad(
    robot_real_t angle_rad
);


robot_real_t
kinematics_sqrt_nonnegative(
    robot_real_t value
);


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
);


robot_status_t
kinematics_add_solution_if_unique(
    kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *candidate
);


/* =========================================================
 * Coordinate Conversion
 * ========================================================= */

robot_status_t
kinematics_public_to_dh_transform(
    const robot_model_t *model,
    const robot_transform_t *public_transform,
    robot_transform_t *dh_transform
);


/* =========================================================
 * UR5 Analytic IK Solver
 * ========================================================= */

robot_status_t
ur5_analytic_ik_solve_q1_candidates(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_candidates_rad[2]
);


robot_status_t
ur5_analytic_ik_solve_q5_candidates(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
);


robot_status_t
ur5_analytic_ik_solve_q6(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
);


robot_status_t
ur5_analytic_ik_solve_q3_candidates(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
);


robot_status_t
ur5_analytic_ik_solve_q2(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
);


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
);


#endif