/*
 * 文件：kinematics_self_test.h
 *
 * 用途：
 * 定义完整 Kinematics Startup Self Test 入口。
 */

#ifndef TESTS_KINEMATICS_KINEMATICS_SELF_TEST_H
#define TESTS_KINEMATICS_KINEMATICS_SELF_TEST_H

#include "error_code.h"


/**
 * @brief 执行完整 Kinematics Self Test。
 *
 * 当前测试由三个独立 Test Suite 组成：
 *
 * 1. Forward Kinematics；
 * 2. UR5 Analytic IK Solver；
 * 3. Public Inverse Kinematics。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 所有测试通过。
 *
 * ROBOT_STATUS_ERROR_INTERNAL：
 * 至少一个测试失败。
 */
robot_status_t
kinematics_self_test_run(void);


#endif