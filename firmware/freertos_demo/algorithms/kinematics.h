/*
 * 文件：kinematics.h
 *
 * 用途：
 * 定义六轴机器人运动学算法层的公共接口。
 *
 * 本文件当前只冻结 API，
 * 正运动学和逆运动学具体算法
 * 将在下一阶段实现。
 *
 * Algorithm Layer 不依赖：
 *
 * UART
 * Protocol
 * FreeRTOS
 * Motor Driver
 *
 * 因此运动学算法可以独立测试。
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * 逆运动学配置
 * ========================================================= */

/*
 * UR5 球形腕结构
 * 理论上最多存在 8 组解析逆解。
 */
#define KINEMATICS_MAX_IK_SOLUTIONS 8U


/*
 * 逆运动学解集合。
 *
 * count：
 * 当前有效解数量。
 *
 * solutions：
 * 有效关节角解。
 */
typedef struct
{
    uint8_t count;

    robot_joint_angles_t solutions[
        KINEMATICS_MAX_IK_SOLUTIONS
    ];

} kinematics_ik_solutions_t;


/* =========================================================
 * Forward Kinematics
 * ========================================================= */

/**
 * @brief 计算六轴机器人正运动学。
 *
 * @param joints
 * 输入六轴关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param transform
 * 输出 Base Frame 到 Tool Frame
 * 的 4 × 4 齐次变换矩阵。
 *
 * 平移单位：
 * mm。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 计算成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出为空。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * 关节角超过允许范围。
 *
 * ROBOT_STATUS_ERROR_NOT_IMPLEMENTED：
 * 当前算法尚未实现。
 */
robot_status_t kinematics_forward(
    const robot_joint_angles_t *joints,
    robot_transform_t *transform
);


/* =========================================================
 * Inverse Kinematics
 * ========================================================= */

/**
 * @brief 计算六轴机器人逆运动学全部可行解析解。
 *
 * @param transform
 * 输入 Base Frame 到 Tool Frame
 * 的目标齐次变换矩阵。
 *
 * @param solutions
 * 输出逆运动学解集合。
 *
 * 最多包含：
 * KINEMATICS_MAX_IK_SOLUTIONS
 * 组解。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 至少得到一组有效解。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出为空。
 *
 * ROBOT_STATUS_ERROR_NO_SOLUTION：
 * 当前目标位姿无可行解。
 *
 * ROBOT_STATUS_ERROR_SINGULAR：
 * 当前位姿处于运动学奇异状态。
 *
 * ROBOT_STATUS_ERROR_NOT_IMPLEMENTED：
 * 当前算法尚未实现。
 */
robot_status_t kinematics_inverse(
    const robot_transform_t *transform,
    kinematics_ik_solutions_t *solutions
);


/* =========================================================
 * IK Solution Selection
 * ========================================================= */

/**
 * @brief 从多组逆运动学解中选择最合适的一组。
 *
 * 当前预留的选择依据包括：
 *
 * 1. 与当前关节位置的总运动距离；
 * 2. 关节限位；
 * 3. 奇异点风险。
 *
 * @param solutions
 * 逆运动学候选解集合。
 *
 * @param reference
 * 当前机器人关节角，
 * 作为选择最接近解的参考。
 *
 * @param selected
 * 输出最终选中的关节角。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 成功选出有效解。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 参数为空。
 *
 * ROBOT_STATUS_ERROR_NO_SOLUTION：
 * 输入候选解为空。
 *
 * ROBOT_STATUS_ERROR_NOT_IMPLEMENTED：
 * 当前筛选算法尚未实现。
 */
robot_status_t kinematics_select_best_solution(
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *reference,
    robot_joint_angles_t *selected
);


#endif