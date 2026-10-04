/*
 * 文件：kinematics_test_support.h
 *
 * 用途：
 * 提供 Kinematics Test Suites 共用的：
 *
 * 1. 数值比较；
 * 2. Joint Angle 比较；
 * 3. Transform 比较；
 * 4. 固定测试姿态；
 * 5. 固定参考 Transform。
 *
 * 本模块只属于测试代码。
 */

#ifndef KINEMATICS_TEST_SUPPORT_H
#define KINEMATICS_TEST_SUPPORT_H

#include <stdint.h>

#include "robot_types.h"


/* =========================================================
 * Test Tolerance
 * ========================================================= */

/*
 * 已知 FK Reference Matrix Test。
 */
#define KINEMATICS_TEST_ROTATION_TOLERANCE \
    0.001F

#define KINEMATICS_TEST_TRANSLATION_TOLERANCE_MM \
    0.05F


/*
 * IK Solution 已量化到 0.01 degree。
 *
 * 因此 IK -> FK Round-Trip 使用独立容差。
 */
#define KINEMATICS_TEST_IK_ROUND_TRIP_ROTATION_TOLERANCE \
    0.002F

#define KINEMATICS_TEST_IK_ROUND_TRIP_TRANSLATION_TOLERANCE_MM \
    0.5F


#define KINEMATICS_TEST_ANGLE_TOLERANCE_RAD \
    0.001F


/* =========================================================
 * Scalar Helpers
 * ========================================================= */

robot_real_t
kinematics_test_abs_real(
    robot_real_t value
);


uint8_t
kinematics_test_scalar_is_close(
    robot_real_t actual,
    robot_real_t expected,
    robot_real_t tolerance
);


/* =========================================================
 * Joint Helpers
 * ========================================================= */

uint8_t
kinematics_test_joint_angles_are_equal(
    const robot_joint_angles_t *left,
    const robot_joint_angles_t *right
);


uint8_t
kinematics_test_joint_angles_are_canonical(
    const robot_joint_angles_t *joints
);


/* =========================================================
 * Transform Helpers
 * ========================================================= */

uint8_t
kinematics_test_transform_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
);


uint8_t
kinematics_test_transform_round_trip_is_close(
    const robot_transform_t *actual,
    const robot_transform_t *expected
);


/* =========================================================
 * Shared Test Fixtures
 * ========================================================= */

const robot_joint_angles_t *
kinematics_test_get_joints_zero(void);


const robot_transform_t *
kinematics_test_get_expected_zero(void);


const robot_joint_angles_t *
kinematics_test_get_joints_q1_60(void);


const robot_transform_t *
kinematics_test_get_expected_q1_60(void);


const robot_joint_angles_t *
kinematics_test_get_joints_q2_minus_90(void);


const robot_transform_t *
kinematics_test_get_expected_q2_minus_90(void);


const robot_joint_angles_t *
kinematics_test_get_joints_ik_validation(void);


#endif