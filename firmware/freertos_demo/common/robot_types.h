/*
 * 文件：robot_types.h
 *
 * 用途：
 * 定义六轴机器人各模块共同使用的核心数据类型。
 *
 * 本文件统一：
 *
 * 1. 关节数量；
 * 2. 关节角数据格式、单位与精度；
 * 3. 关节速度数据格式；
 * 4. 笛卡尔位姿数据格式；
 * 5. 坐标系与姿态角约定；
 * 6. 4 × 4 齐次变换矩阵格式。
 *
 * Driver、Communication、Algorithm、Control 等模块
 * 应优先使用这里定义的公共类型。
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>


/* =========================================================
 * 六轴机器人基本配置
 * ========================================================= */

#define ROBOT_JOINT_COUNT 6U


/* =========================================================
 * 通用计算标量
 * ========================================================= */

/*
 * 运动学、轨迹规划和 PID 内部计算
 * 统一使用单精度浮点数。
 */
typedef float robot_real_t;


/* =========================================================
 * 关节角数据
 * ========================================================= */

/*
 * 外部关节角统一采用 int16_t 定点表示：
 *
 * 1 unit = 0.01 degree
 *
 * 例如：
 *
 * 6000  =  60.00°
 * -9000 = -90.00°
 */
typedef int16_t robot_joint_angle_t;


typedef struct
{
    robot_joint_angle_t value[
        ROBOT_JOINT_COUNT
    ];

} robot_joint_angles_t;


/*
 * 原始关节角转 degree：
 *
 * degree =
 * raw × ROBOT_JOINT_ANGLE_UNIT_DEG
 */
#define ROBOT_JOINT_ANGLE_UNIT_DEG 0.01F


/* =========================================================
 * 关节速度
 * ========================================================= */

/*
 * 单位：
 *
 * degree / second
 */
typedef robot_real_t robot_joint_velocity_t;


typedef struct
{
    robot_joint_velocity_t value[
        ROBOT_JOINT_COUNT
    ];

} robot_joint_velocities_t;


/* =========================================================
 * 笛卡尔位姿
 * ========================================================= */

/*
 * robot_pose_t 坐标约定：
 *
 * Position：
 *
 * x_mm
 * y_mm
 * z_mm
 *
 * 单位：
 *
 * mm
 *
 * Orientation：
 *
 * rx_deg = Roll  about X
 * ry_deg = Pitch about Y
 * rz_deg = Yaw   about Z
 *
 * 单位：
 *
 * degree
 *
 * 姿态组合约定：
 *
 * R =
 * Rz(rz)
 * ×
 * Ry(ry)
 * ×
 * Rx(rx)
 *
 * 即常用 Roll-Pitch-Yaw 表示。
 *
 * 位姿默认描述：
 *
 * Tool Frame
 * 相对于
 * Robot Base Frame
 * 的位置和姿态。
 */
typedef struct
{
    robot_real_t x_mm;

    robot_real_t y_mm;

    robot_real_t z_mm;


    robot_real_t rx_deg;

    robot_real_t ry_deg;

    robot_real_t rz_deg;

} robot_pose_t;


/* =========================================================
 * 齐次变换矩阵
 * ========================================================= */

/*
 * 4 × 4 Homogeneous Transform。
 *
 * 使用 Row-Major 存储：
 *
 * matrix[row][column]
 *
 * 形式：
 *
 * [ R00 R01 R02 Tx ]
 * [ R10 R11 R12 Ty ]
 * [ R20 R21 R22 Tz ]
 * [  0   0   0   1 ]
 *
 * 其中：
 *
 * R：
 * 3 × 3 Rotation Matrix
 *
 * T：
 * Translation，单位 mm
 *
 * 当前默认含义：
 *
 * Base Frame
 * ->
 * Tool Frame
 *
 * 即：
 *
 * T_base_tool
 */
typedef struct
{
    robot_real_t matrix[4][4];

} robot_transform_t;


#endif