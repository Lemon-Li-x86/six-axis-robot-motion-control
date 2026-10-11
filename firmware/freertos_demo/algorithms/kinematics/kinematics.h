/*
 * 文件：kinematics.h
 *
 * 用途：
 * 定义机器人运动学模块的公共接口。
 *
 * Kinematics Layer：
 *
 * 1. 不依赖 UART；
 * 2. 不依赖 Protocol；
 * 3. 不依赖 FreeRTOS；
 * 4. 不依赖 Motor Driver；
 * 5. 不负责 Trajectory；
 * 6. 不负责 Motion Policy；
 * 7. 不负责 Control。
 *
 * 当前使用 Active Robot Model。
 *
 * 当前 Active Model：
 *
 * UR5。
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


/*
 * UR5 Analytic IK：
 *
 * Shoulder × Wrist × Elbow
 *
 * 2 × 2 × 2
 *
 * =
 *
 * 8。
 */
#define KINEMATICS_MAX_IK_SOLUTIONS 8U


/**
 * @brief 保存一次逆运动学求解得到的候选解集合。
 *
 * count：
 * 当前有效 Solution 数量。
 *
 * solutions：
 * 最多保存 KINEMATICS_MAX_IK_SOLUTIONS
 * 组 Canonical Joint Angles。
 */
typedef struct
{
    uint8_t count;

    robot_joint_angles_t solutions[
        KINEMATICS_MAX_IK_SOLUTIONS
    ];

} kinematics_ik_solutions_t;


/**
 * @brief 计算机器人 Forward Kinematics。
 *
 * @param[in] joints
 * 六轴 Canonical Joint Angles。
 *
 * 单位：
 * 0.01 degree。
 *
 * 范围：
 * [-180°, 180°)。
 *
 * @param[out] transform
 * 计算得到的公共齐次变换矩阵 T_base_ee。
 *
 * Translation 单位：
 * mm。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 计算成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * joints 或 transform 为 NULL。
 *
 * @note
 * 输入和输出均使用项目公共坐标约定。
 *
 * Standard DH Frame 仅属于 Kinematics
 * 模块内部实现细节。
 */
robot_status_t kinematics_forward(
    const robot_joint_angles_t *joints,
    robot_transform_t *transform
);


/**
 * @brief 计算当前 Active Robot Model 的解析逆运动学。
 *
 * @param[in] transform
 * 目标公共齐次变换矩阵 T_base_ee。
 *
 * Translation 单位：
 * mm。
 *
 * @param[out] solutions
 * 返回所有当前能够求得的几何逆解。
 *
 * 每组解均为 Canonical Joint Angles，
 * 单位为 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 至少得到一组有效逆解。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * transform 或 solutions 为 NULL。
 *
 * ROBOT_STATUS_ERROR_NO_SOLUTION：
 * 当前目标不存在几何逆解。
 *
 * ROBOT_STATUS_ERROR_SINGULAR：
 * 当前所有可达 Branch 均落入
 * 尚未处理的奇异状态。
 *
 * 其他错误码：
 * 底层矩阵运算或内部求解失败。
 *
 * @note
 * 当前实现返回的候选解满足：
 *
 * 1. 几何有效；
 * 2. 非当前未处理奇异 Branch；
 * 3. 已规范化为 Canonical Angle；
 * 4. 已量化为 0.01 degree；
 * 5. 已删除量化后完全重复的 Solution。
 */
robot_status_t kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions
);


#endif