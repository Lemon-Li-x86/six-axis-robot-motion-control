/*
 * 文件：kinematics_internal.h
 *
 * 用途：
 * 暴露 Analytic IK Stage Validation API。
 *
 * 本文件不属于正式 Public API。
 *
 * 当前仅供：
 *
 * tests/kinematics_self_test.c
 *
 * 使用。
 */

#ifndef KINEMATICS_INTERNAL_H
#define KINEMATICS_INTERNAL_H

#include "robot_types.h"
#include "error_code.h"


robot_status_t
kinematics_internal_solve_q1_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_candidates_rad[2]
);


robot_status_t
kinematics_internal_solve_q5_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
);


robot_status_t
kinematics_internal_solve_q6(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
);


robot_status_t
kinematics_internal_solve_q3_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
);


robot_status_t
kinematics_internal_solve_q2(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
);


robot_status_t
kinematics_internal_solve_q4(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t *q4_rad
);


#endif