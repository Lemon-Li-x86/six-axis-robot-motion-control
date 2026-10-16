
/*
 * 文件：robot_model.h
 *
 * 用途：
 * 定义机器人本体的静态几何与关节限位模型。
 *
 * Robot Model 描述机器人“是什么”，
 * 而不是机器人“现在处于什么状态”。
 *
 * 当前保存：
 *
 * 1. Standard DH 参数；
 * 2. Public Base Frame 与 DH Base Frame 的转换；
 * 3. DH Tool Frame 与 Public End Frame 的转换；
 * 4. Joint Position Limits。
 *
 * 后续可以继续扩展：
 *
 * 1. Maximum Velocity；
 * 2. Maximum Acceleration；
 * 3. 其他机器人固有参数。
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


    /* =========================================================
     * Joint Position Limits
     * ========================================================= */

    /*
     * 六轴关节的连续位置下限和上限。
     *
     * 数据类型：
     * robot_joint_positions_t。
     *
     * 单位：
     * 0.01 degree。
     *
     * 最小值和最大值均为包含边界。
     *
     * 注意：
     * 本限位使用 Continuous Joint Position，
     * 而非 [-180°, 180°) 的 Canonical Angle。
     *
     * IK 返回的 Canonical Angle
     * 需先展开为连续位置，再判断是否越界。
     */
    robot_joint_positions_t
        joint_min_positions;

    robot_joint_positions_t
        joint_max_positions;

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
