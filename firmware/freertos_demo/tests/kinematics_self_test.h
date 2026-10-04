#ifndef KINEMATICS_SELF_TEST_H
#define KINEMATICS_SELF_TEST_H

#include "error_code.h"


/**
 * @brief 执行 UR5 运动学自检。
 *
 * 当前验证：
 *
 * 1. 已知关节姿态的 Forward Kinematics；
 * 2. Analytic IK 各阶段公式链；
 * 3. Public IK 8-Solution Assembly；
 * 4. IK Solution Canonical Angle；
 * 5. IK Duplicate Solution Detection；
 * 6. 每组 IK Solution 的 FK Round-Trip；
 * 7. Wrist Singularity Detection。
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