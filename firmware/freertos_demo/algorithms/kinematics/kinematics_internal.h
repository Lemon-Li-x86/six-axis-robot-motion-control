/*
 * 文件：kinematics_internal.h
 *
 * 用途：
 * 暴露 Analytic IK Stage Validation API。
 *
 * 本文件不属于正式 Public API。
 *
 * 主要供：
 *
 * tests/kinematics/
 *
 * 中的 IK Solver Test 使用。
 *
 * Application、Motion、Control 等正式模块
 * 不应 include 本文件。
 */

#ifndef KINEMATICS_INTERNAL_H
#define KINEMATICS_INTERNAL_H

#include "robot_types.h"
#include "error_code.h"


/**
 * @brief 求解 UR5 Analytic IK 的两个 q1 Shoulder Candidate。
 *
 * @param[in] transform
 * 公共 T_base_ee。
 *
 * Translation 单位：
 * mm。
 *
 * @param[out] q1_candidates_rad
 * 两个 q1 Candidate。
 *
 * 单位：
 * radian。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 求解成功。
 *
 * ROBOT_STATUS_ERROR_NO_SOLUTION：
 * Wrist Geometry 不可达。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出 Pointer 为 NULL。
 */
robot_status_t kinematics_internal_solve_q1_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_candidates_rad[2]
);


/**
 * @brief 在给定 q1 Branch 下求解两个 q5 Wrist Candidate。
 *
 * @param[in] transform
 * 公共 T_base_ee。
 *
 * @param[in] q1_rad
 * 已选择的 q1，单位 radian。
 *
 * @param[out] q5_candidates_rad
 * 两个 q5 Candidate，单位 radian。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 求解成功。
 *
 * ROBOT_STATUS_ERROR_NO_SOLUTION：
 * 三角函数几何条件不可满足。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出 Pointer 为 NULL。
 */
robot_status_t kinematics_internal_solve_q5_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
);


/**
 * @brief 在给定 q1 和 q5 Branch 下求解 q6。
 *
 * @param[in] transform
 * 公共 T_base_ee。
 *
 * @param[in] q1_rad
 * q1，单位 radian。
 *
 * @param[in] q5_rad
 * q5，单位 radian。
 *
 * @param[out] q6_rad
 * q6，单位 radian。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 求解成功。
 *
 * ROBOT_STATUS_ERROR_SINGULAR：
 * sin(q5) 接近 0，
 * 当前 Wrist Branch 无法唯一确定 q6。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出 Pointer 为 NULL。
 */
robot_status_t kinematics_internal_solve_q6(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
);


/**
 * @brief 在给定 Shoulder / Wrist Branch 下
 *        求解两个 q3 Elbow Candidate。
 *
 * @param[in] transform
 * 公共 T_base_ee。
 *
 * @param[in] q1_rad
 * q1，单位 radian。
 *
 * @param[in] q5_rad
 * q5，单位 radian。
 *
 * @param[in] q6_rad
 * q6，单位 radian。
 *
 * @param[out] q3_candidates_rad
 * 两个 q3 Candidate，单位 radian。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 求解成功。
 *
 * ROBOT_STATUS_ERROR_NO_SOLUTION：
 * Elbow Geometry 不可达。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出 Pointer 为 NULL。
 */
robot_status_t kinematics_internal_solve_q3_candidates(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
);


/**
 * @brief 在给定 q1、q5、q6、q3 Branch 下求解 q2。
 *
 * @param[in] transform
 * 公共 T_base_ee。
 *
 * @param[in] q1_rad
 * q1，单位 radian。
 *
 * @param[in] q5_rad
 * q5，单位 radian。
 *
 * @param[in] q6_rad
 * q6，单位 radian。
 *
 * @param[in] q3_rad
 * q3，单位 radian。
 *
 * @param[out] q2_rad
 * q2，单位 radian。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 求解成功。
 *
 * ROBOT_STATUS_ERROR_SINGULAR：
 * Shoulder-Elbow Geometry 退化。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出 Pointer 为 NULL。
 */
robot_status_t kinematics_internal_solve_q2(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
);


/**
 * @brief 根据前五个已确定 Joint Branch 求解 q4。
 *
 * @param[in] transform
 * 公共 T_base_ee。
 *
 * @param[in] q1_rad
 * q1，单位 radian。
 *
 * @param[in] q2_rad
 * q2，单位 radian。
 *
 * @param[in] q3_rad
 * q3，单位 radian。
 *
 * @param[in] q5_rad
 * q5，单位 radian。
 *
 * @param[in] q6_rad
 * q6，单位 radian。
 *
 * @param[out] q4_rad
 * q4，单位 radian。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 求解成功。
 *
 * ROBOT_STATUS_ERROR_SINGULAR：
 * 当前 Orientation 无法唯一确定 q4。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出 Pointer 为 NULL。
 */
robot_status_t kinematics_internal_solve_q4(
    const robot_transform_t *transform,
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t *q4_rad
);


#endif