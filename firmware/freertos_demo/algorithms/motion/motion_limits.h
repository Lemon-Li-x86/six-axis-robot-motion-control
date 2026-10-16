/*
 * 文件：motion_limits.h
 *
 * 用途：
 * 定义关节连续位置限位检查与角度展开接口。
 *
 * 本模块属于 Motion Algorithm Layer：
 *
 * 1. 从 Robot Model 读取静态关节限位；
 * 2. 判断六轴连续位置是否合法；
 * 3. 将 Canonical Angle 展开为最近的合法连续位置。
 *
 * 本模块不执行电机驱动，也不修改当前机器人状态。
 */

#ifndef MOTION_LIMITS_H
#define MOTION_LIMITS_H

#include "robot_types.h"
#include "error_code.h"


/**
 * @brief 检查六轴连续位置是否位于当前模型限位内。
 *
 * @param[in] positions
 * 六轴连续位置，单位 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：全部关节合法；
 * ROBOT_STATUS_ERROR_NULL_POINTER：输入为空；
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：至少一个关节越限；
 * ROBOT_STATUS_ERROR_INTERNAL：机器人模型配置异常。
 */
robot_status_t motion_limits_check_positions(
    const robot_joint_positions_t *positions
);


/**
 * @brief 将六轴规范化角度展开为最近的合法连续位置。
 *
 * 以 current_positions 作为参考，
 * 对每个关节寻找 candidate + 36000 * k，
 * 选择满足模型限位且距离当前关节最近的连续位置。
 *
 * 如果两个候选距离相等，优先选择数值较大的连续位置。
 *
 * 函数只计算目标，不检查运动过程或碰撞。
 * 失败时不修改输出对象。
 *
 * @param[in] canonical_angles
 * 六轴 Canonical Angle，范围 [-18000, 18000)，
 * 单位 0.01 degree。
 *
 * @param[in] current_positions
 * 当前六轴连续位置，必须处于模型限位内，
 * 单位 0.01 degree。
 *
 * @param[out] target_positions
 * 展开后的目标连续位置，单位 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：展开成功；
 * ROBOT_STATUS_ERROR_NULL_POINTER：任意输入输出指针为空；
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：规范化角度非法；
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：当前连续位置越限；
 * ROBOT_STATUS_ERROR_NO_SOLUTION：不存在合法等效连续角；
 * ROBOT_STATUS_ERROR_INTERNAL：机器人模型配置异常。
 */
robot_status_t motion_limits_unwrap_nearest(
    const robot_joint_angles_t *canonical_angles,
    const robot_joint_positions_t *current_positions,
    robot_joint_positions_t *target_positions
);


#endif
