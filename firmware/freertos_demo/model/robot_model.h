/*
 * 文件：robot_model.h
 *
 * 用途：
 * 定义机器人本体的静态几何模型。
 *
 * Robot Model 描述机器人“是什么”，
 * 而不是机器人“现在处于什么状态”。
 *
 * 当前保存：
 *
 * 1. Standard DH 参数；
 * 2. Public Base Frame 与 DH Base Frame 的转换；
 * 3. DH Tool Frame 与 Public End Frame 的转换。
 *
 * 后续可以继续扩展：
 *
 * 1. Joint Limit；
 * 2. Maximum Velocity；
 * 3. Maximum Acceleration；
 * 4. 其他机器人固有参数。
 *
 * 不应在本模块中保存：
 *
 * 1. 当前关节位置；
 * 2. 当前速度；
 * 3. 当前目标；
 * 4. PID 状态；
 * 5. Trajectory Runtime State。
 */

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "robot_types.h"


typedef struct
{
    /*
     * Standard DH Parameters。
     *
     * 单位：
     *
     * a：mm
     * d：mm
     * alpha：radian
     */
    robot_real_t dh_a_mm[
        ROBOT_JOINT_COUNT
    ];

    robot_real_t dh_d_mm[
        ROBOT_JOINT_COUNT
    ];

    robot_real_t dh_alpha_rad[
        ROBOT_JOINT_COUNT
    ];


    /*
     * Forward Kinematics：
     *
     * T_base_ee
     *
     * =
     *
     * base_frame_conversion
     * ×
     * T_0_6
     * ×
     * tool_frame_conversion
     */
    robot_transform_t
        base_frame_conversion;

    robot_transform_t
        base_frame_conversion_inverse;


    robot_transform_t
        tool_frame_conversion;

    robot_transform_t
        tool_frame_conversion_inverse;

} robot_model_t;


/**
 * @brief 获取当前固件使用的机器人模型。
 *
 * 当前：
 *
 * UR5。
 *
 * 后续如果增加其他机器人，
 * 模型选择逻辑可以在 Model Layer 内部扩展，
 * 上层算法无需直接引用具体型号全局变量。
 */
const robot_model_t *
robot_model_get_active(void);


#endif