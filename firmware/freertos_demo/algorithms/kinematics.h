/*
 * 文件：kinematics.h
 *
 * 用途：
 * 定义六轴机器人运动学算法层的公共接口。
 *
 * Algorithm Layer 不依赖：
 *
 * UART
 * Protocol
 * FreeRTOS
 * Motor Driver
 *
 * 因此运动学算法可以独立测试。
 *
 *
 * =========================================================
 * 公共坐标系约定
 * =========================================================
 *
 * Kinematics 对外统一使用
 * 当前 UR5 URDF / PyBullet 坐标系：
 *
 * Base Frame：
 *
 * base_link
 *
 * End Frame：
 *
 * ee_link
 *
 * 因此：
 *
 * robot_transform_t
 *
 * 在本模块公共接口中的含义为：
 *
 * T_base_ee
 *
 * 即 ee_link 相对于 base_link
 * 的齐次变换。
 *
 *
 * Standard DH 坐标系：
 *
 * {0} ... {6}
 *
 * 仅作为本模块内部实现细节。
 *
 * Forward Kinematics 内部可以：
 *
 * Joint Angles
 * ->
 * Standard DH T_0_6
 * ->
 * Coordinate Conversion
 * ->
 * Public T_base_ee
 *
 * Inverse Kinematics 则执行逆过程。
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
 *
 * 每组解使用 Canonical Angle：
 *
 * [-180°, 180°)
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
 * Canonical Range：
 *
 * [-180°, 180°)
 *
 * @param transform
 * 输出：
 *
 * T_base_ee
 *
 * 即 ee_link 相对于 base_link
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
 * 输入数据超过算法允许范围。
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
 * 输入目标：
 *
 * T_base_ee
 *
 * 即 ee_link 相对于 base_link
 * 的目标齐次变换矩阵。
 *
 * @param solutions
 * 输出逆运动学解集合。
 *
 * 最多包含：
 * KINEMATICS_MAX_IK_SOLUTIONS
 * 组解。
 *
 * 每组关节角规范化到：
 *
 * [-180°, 180°)
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
 * 当前机器人 Canonical Joint Angles，
 * 作为选择最接近解的参考。
 *
 * @param selected
 * 输出最终选中的关节角。
 *
 * 输出采用：
 *
 * [-180°, 180°)
 *
 * Canonical Angle。
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
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * 解数量异常。
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