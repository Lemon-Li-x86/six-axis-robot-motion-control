/*
 * 文件：kinematics_internal.h
 *
 * 用途：
 * 定义 Kinematics 模块内部验证接口。
 *
 * 本文件不属于公共运动学 API。
 *
 * 主要供：
 *
 * 1. Kinematics 内部实现；
 * 2. Self Test
 *
 * 使用。
 */

#ifndef KINEMATICS_INTERNAL_H
#define KINEMATICS_INTERNAL_H


#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * IK Stage 1
 * q1 Shoulder
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q1_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_candidates_rad[2]
);


/* =========================================================
 * IK Stage 2
 * q5 Wrist
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q5_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
);


/* =========================================================
 * IK Stage 3
 * q6 Wrist
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q6(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
);


/* =========================================================
 * IK Stage 4
 * q3 Elbow
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q3_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
);


/* =========================================================
 * IK Stage 5
 * q2 Shoulder Lift
 * ========================================================= */

robot_status_t
kinematics_internal_solve_q2(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
);


/* =========================================================
 * IK Stage 6
 * q4 Wrist 1
 * ========================================================= */

/**
 * @brief 已知 q1/q2/q3/q5/q6 求唯一 q4。
 *
 * @param transform
 * 输入：
 * T_base_ee
 *
 * @param q1_rad
 * q1。
 *
 * @param q2_rad
 * q2。
 *
 * @param q3_rad
 * q3。
 *
 * @param q5_rad
 * q5。
 *
 * @param q6_rad
 * q6。
 *
 * @param q4_rad
 * 输出 q4。
 *
 * 单位：
 * radian
 *
 * 范围：
 * [-pi, pi)
 */
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