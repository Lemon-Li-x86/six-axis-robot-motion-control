/**
 * @file robot_model.h
 * @brief Immutable robot geometry, transforms and joint position limits.
 *
 * The model describes static robot properties, not measured joint states,
 * controller state or trajectory runtime state.
 */

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "robot_types.h"

/** @brief UR5 静态几何、坐标变换和连续关节位置限位。 */
typedef struct
{
    /* DH: a/d 单位 mm，alpha 单位 rad。 */

    robot_real_t dh_a_mm[
        ROBOT_JOINT_COUNT
    ];

    robot_real_t dh_d_mm[
        ROBOT_JOINT_COUNT
    ];

    robot_real_t dh_alpha_rad[
        ROBOT_JOINT_COUNT
    ];

    /* 公共基座与 DH 基座坐标变换及其逆矩阵。 */

    robot_transform_t
        base_frame_conversion;

    robot_transform_t
        base_frame_conversion_inverse;

    /* DH 工具坐标系与公开末端坐标系之间的变换。 */

    robot_transform_t
        tool_frame_conversion;

    robot_transform_t
        tool_frame_conversion_inverse;

    /* 连续关节角上下限，单位 0.01°，端点包含。 */

    robot_joint_positions_t
        joint_min_positions;

    robot_joint_positions_t
        joint_max_positions;

} robot_model_t;

/**
 * @brief 获取当前启用的机器人静态模型（UR5）。
 * @return 模型只读指针。
 */
const robot_model_t *
robot_model_get_active(void);

#endif
