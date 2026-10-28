/*
 * 文件：kinematics_private.h
 *
 * 用途：
 * 定义 Kinematics Module 内部共享接口。
 *
 * 本文件只允许：
 *
 * Kinematics Module Implementation
 *
 * 使用。
 *
 * Public Application Code
 * 不应 include 本文件。
 */

#ifndef KINEMATICS_PRIVATE_H
#define KINEMATICS_PRIVATE_H

#include <stdint.h>

#include "kinematics.h"
#include "robot_model.h"
#include "math_constants.h"


/* =========================================================
 * Mathematical Constants
 * ========================================================= */

#define KINEMATICS_PI_F ROBOT_PI_F
#define KINEMATICS_TWO_PI_F ROBOT_TWO_PI_F
#define KINEMATICS_DEG_TO_RAD_F ROBOT_DEG_TO_RAD_F
#define KINEMATICS_RAD_TO_DEG_F ROBOT_RAD_TO_DEG_F


/* Near-wrist singularity conditioning policy for OPTIONAL seeded IK.
 * These are numeric-reliability thresholds, not geometry tolerances.
 * They do not relax KINEMATICS_TRIG_DOMAIN_EPSILON in ur5_analytic_ik.c.
 *
 * sin(q5)<0.001: small projections lead to unstable q6 from binary32 SO(3).
 * Reference q1 must agree within 0.005 rad; reference q5 must share sign.
 * A seeded q6 substitutes only if its deviation from orientation-derived q6
 * is <=0.05 rad. FK verification independently prevents false candidates.
 */
#define KINEMATICS_SEEDED_WRIST_SIN_THRESHOLD 0.001F
#define KINEMATICS_SEEDED_SHOULDER_MAX_ERROR_RAD 0.005F
#define KINEMATICS_SEEDED_Q6_MAX_ERROR_RAD 0.05F

/* =========================================================
 * Shared Math Helpers
 * ========================================================= */

/**
 * @brief 计算单精度浮点数绝对值。
 *
 * @param[in] value
 * 输入值。
 *
 * @return
 * value 的非负绝对值。
 */
robot_real_t kinematics_abs_real(
    robot_real_t value
);


/**
 * @brief 将 Radian Angle 规范化到 [-pi, pi)。
 *
 * @param[in] angle_rad
 * 输入角度，单位 radian。
 *
 * @return
 * Canonical Radian Angle，
 * 范围 [-pi, pi)。
 */
robot_real_t kinematics_normalize_angle_rad(
    robot_real_t angle_rad
);


/**
 * @brief 计算非负输入的平方根。
 *
 * @param[in] value
 * 输入值。
 *
 * @return
 * value > 0 时返回平方根。
 *
 * value <= 0 时返回 0。
 *
 * @note
 * 本函数用于 Kinematics 内部几何计算。
 * 数值算法实现细节位于 kinematics_common.c。
 */
robot_real_t kinematics_sqrt_nonnegative(
    robot_real_t value
);


/* =========================================================
 * Solution Helpers
 * ========================================================= */

/**
 * @brief 将六个 Radian Joint Angle
 *        转换为公共 Canonical Joint Solution。
 *
 * @param[in] q1_rad
 * Joint 1，单位 radian。
 *
 * @param[in] q2_rad
 * Joint 2，单位 radian。
 *
 * @param[in] q3_rad
 * Joint 3，单位 radian。
 *
 * @param[in] q4_rad
 * Joint 4，单位 radian。
 *
 * @param[in] q5_rad
 * Joint 5，单位 radian。
 *
 * @param[in] q6_rad
 * Joint 6，单位 radian。
 *
 * @param[out] solution
 * 输出公共 Joint Solution。
 *
 * 单位：
 * 0.01 degree。
 *
 * 范围：
 * [-180°, 180°)。
 *
 * @note
 * 输出过程中会执行：
 *
 * 1. Radian Canonical Normalization；
 * 2. Radian -> Degree；
 * 3. 0.01 degree Quantization；
 * 4. Integer Canonical Normalization。
 */
void kinematics_build_joint_solution(
    robot_real_t q1_rad,
    robot_real_t q2_rad,
    robot_real_t q3_rad,
    robot_real_t q4_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_joint_angles_t *solution
);


/**
 * @brief 将 Candidate 添加到 IK Solution Set，
 *        并删除量化后完全重复的解。
 *
 * @param[in,out] solutions
 * 当前 IK Solution Set。
 *
 * @param[in] candidate
 * 待加入的 Canonical Candidate。
 *
 * @return
 * ROBOT_STATUS_OK：
 * Candidate 已加入，
 * 或 Candidate 与已有 Solution 完全重复。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * Pointer 参数为 NULL。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * Solution Set 已达到
 * KINEMATICS_MAX_IK_SOLUTIONS。
 */
robot_status_t kinematics_add_solution_if_unique(
    kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *candidate
);


/* =========================================================
 * Coordinate Conversion
 * ========================================================= */

/**
 * @brief 将公共 T_base_ee 转换为
 *        Standard DH T_0_6。
 *
 * @param[in] model
 * 当前 Robot Model。
 *
 * @param[in] public_transform
 * 公共 T_base_ee。
 *
 * @param[out] dh_transform
 * Standard DH T_0_6。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 转换成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * Pointer 参数为 NULL。
 *
 * 其他错误码：
 * Matrix Operation 失败。
 *
 * @note
 * 转换关系：
 *
 * T_0_6
 *
 * =
 *
 * base_frame_conversion_inverse
 * ×
 * T_base_ee
 * ×
 * tool_frame_conversion_inverse
 */
robot_status_t kinematics_public_to_dh_transform(
    const robot_model_t *model,
    const robot_transform_t *public_transform,
    robot_transform_t *dh_transform
);


/* =========================================================
 * UR5 Analytic IK Solver
 * ========================================================= */

/**
 * @brief 求解 Standard DH Transform 对应的
 *        两个 q1 Candidate。
 *
 * @param[in] model
 * UR5 Robot Model。
 *
 * @param[in] dh_transform
 * Standard DH T_0_6。
 *
 * @param[out] q1_candidates_rad
 * 两个 q1 Candidate，单位 radian。
 *
 * @return
 * Solver Status。
 */
robot_status_t ur5_analytic_ik_solve_q1_candidates(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_candidates_rad[2]
);


/**
 * @brief 在指定 q1 Branch 下求解
 *        两个 q5 Candidate。
 *
 * @param[in] model
 * UR5 Robot Model。
 *
 * @param[in] dh_transform
 * Standard DH T_0_6。
 *
 * @param[in] q1_rad
 * q1，单位 radian。
 *
 * @param[out] q5_candidates_rad
 * 两个 q5 Candidate，单位 radian。
 *
 * @return
 * Solver Status。
 */
robot_status_t ur5_analytic_ik_solve_q5_candidates(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_candidates_rad[2]
);


/**
 * @brief 在指定 q1 / q5 Branch 下求解 q6。
 *
 * @param[in] model
 * UR5 Robot Model。
 *
 * @param[in] dh_transform
 * Standard DH T_0_6。
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
 * Solver Status。
 */
robot_status_t ur5_analytic_ik_solve_q6(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t *q6_rad
);


/**
 * @brief 在指定 Shoulder / Wrist Branch 下
 *        求解两个 q3 Candidate。
 *
 * @param[in] model
 * UR5 Robot Model。
 *
 * @param[in] dh_transform
 * Standard DH T_0_6。
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
 * Solver Status。
 */
robot_status_t ur5_analytic_ik_solve_q3_candidates(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_candidates_rad[2]
);


/**
 * @brief 在指定 q1 / q5 / q6 / q3 Branch 下求解 q2。
 *
 * @param[in] model
 * UR5 Robot Model。
 *
 * @param[in] dh_transform
 * Standard DH T_0_6。
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
 * Solver Status。
 */
robot_status_t ur5_analytic_ik_solve_q2(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q5_rad,
    robot_real_t q6_rad,
    robot_real_t q3_rad,
    robot_real_t *q2_rad
);


/**
 * @brief 根据其余已确定 Branch 求解 q4。
 *
 * @param[in] model
 * UR5 Robot Model。
 *
 * @param[in] dh_transform
 * Standard DH T_0_6。
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
 * Solver Status。
 */
robot_status_t ur5_analytic_ik_solve_q4(
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