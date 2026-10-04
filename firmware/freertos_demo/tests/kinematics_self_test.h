#ifndef KINEMATICS_SELF_TEST_H
#define KINEMATICS_SELF_TEST_H

#include "error_code.h"


/**
 * @brief 执行 UR5 Forward Kinematics 自检。
 *
 * 使用若干已知关节姿态和对应期望矩阵，
 * 验证 kinematics_forward() 的数值结果。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 所有测试通过。
 *
 * ROBOT_STATUS_ERROR_INTERNAL：
 * 至少一个测试失败。
 */
robot_status_t kinematics_self_test_run(void);


#endif