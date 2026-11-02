/**
 * @file kinematics_dedup_test.h
 * @brief 测试共享声明/夹具：kinematics dedup。
 */

#ifndef KINEMATICS_DEDUP_TEST_H
#define KINEMATICS_DEDUP_TEST_H

#include "error_code.h"

/**
 * @brief 验证逆解去重逻辑。
 *
 * 验证内容：
 * 1. NULL Pointer Contract；
 * 2. +180° / -180° 规范化；
 * 3. 相同 Candidate 不重复加入；
 * 4. 不同 Candidate 正常加入；
 * 5. 已保存的 Candidate 不被覆盖。
 *
 * @return ROBOT_STATUS_OK 表示全部通过。
 */
robot_status_t kinematics_dedup_test_run(void);

#endif /* KINEMATICS_DEDUP_TEST_H */
