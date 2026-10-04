/*
 * 文件：kinematics.h
 *
 * 用途：
 * 定义机器人运动学公共接口。
 *
 * Kinematics Layer：
 *
 * 1. 不依赖 UART；
 * 2. 不依赖 Protocol；
 * 3. 不依赖 FreeRTOS；
 * 4. 不依赖 Motor Driver；
 * 5. 不负责 Trajectory；
 * 6. 不负责 Control。
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


typedef struct
{
    uint8_t count;

    robot_joint_angles_t solutions[
        KINEMATICS_MAX_IK_SOLUTIONS
    ];

} kinematics_ik_solutions_t;


/**
 * @brief 计算 Forward Kinematics。
 *
 * 输入：
 *
 * Canonical Joint Angles
 * [-180°, 180°)
 *
 * 单位：
 *
 * 0.01 degree。
 *
 * 输出：
 *
 * T_base_ee
 *
 * 平移单位：
 *
 * mm。
 */
robot_status_t kinematics_forward(
    const robot_joint_angles_t *joints,
    robot_transform_t *transform
);


/**
 * @brief 计算当前 UR5 模型的解析逆运动学。
 *
 * 返回：
 *
 * 1. 几何有效；
 * 2. 非当前未处理奇异 Branch；
 * 3. Canonical；
 * 4. 0.01 degree 量化；
 * 5. 已去除量化后重复解；
 *
 * 的候选解集合。
 *
 * 本函数只负责回答：
 *
 * “有哪些几何逆解？”
 *
 * 不负责：
 *
 * 1. Joint Limit Policy；
 * 2. 当前机器人连续位置；
 * 3. Trajectory Continuity；
 * 4. 最优运动距离；
 * 5. Motion Planning。
 */
robot_status_t kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions
);


/**
 * @brief 旧接口预留。
 *
 * 当前仍保持 API 兼容，
 * 但 Solution Selection 不应继续在
 * Kinematics Layer 中实现。
 *
 * 后续应迁移到 Motion Layer，
 * 并基于 Continuous Joint Position
 * 和 Robot Model Limits 进行选择。
 */
robot_status_t kinematics_select_best_solution(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *reference,
    robot_joint_angles_t *selected
);


#endif