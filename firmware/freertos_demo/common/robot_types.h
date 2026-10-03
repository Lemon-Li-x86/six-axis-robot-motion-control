/*
 * 文件：robot_types.h
 *
 * 用途：
 * 定义六轴机器人各模块共同使用的核心数据类型。
 *
 * 本文件用于统一：
 *
 * 1. 关节数量；
 * 2. 关节角数据格式；
 * 3. 关节角单位与精度；
 * 4. 笛卡尔位姿数据格式；
 * 5. 位姿单位。
 *
 * Driver、Communication、Algorithm、Control 等模块
 * 应优先使用这里定义的公共类型，
 * 避免各模块自行定义不同的数据表示。
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>


/* =========================================================
 * 六轴机器人基本配置
 * ========================================================= */

/*
 * 当前机器人自由度：
 *
 * 6 DOF
 */
#define ROBOT_JOINT_COUNT 6U


/* =========================================================
 * 关节角数据
 * ========================================================= */

/*
 * 当前关节角采用有符号 16 bit 定点表示：
 *
 * 1 unit = 0.01 degree
 *
 * 例如：
 *
 * 6000 = 60.00°
 * -9000 = -90.00°
 *
 * 当前通信协议、控制目标和状态反馈
 * 统一使用该表示。
 */
typedef int16_t robot_joint_angle_t;


/*
 * 六轴关节角集合。
 *
 * value[0] -> Joint 1
 * value[1] -> Joint 2
 * ...
 * value[5] -> Joint 6
 */
typedef struct
{
    robot_joint_angle_t value[
        ROBOT_JOINT_COUNT
    ];

} robot_joint_angles_t;


/*
 * 关节角单位换算：
 *
 * raw value × 0.01
 * =
 * degree
 */
#define ROBOT_JOINT_ANGLE_UNIT_DEG 0.01F


/* =========================================================
 * 通用浮点数类型
 * ========================================================= */

/*
 * 后续运动学、轨迹规划和 PID
 * 统一使用单精度浮点数作为计算标量。
 */
typedef float robot_real_t;


/* =========================================================
 * 笛卡尔位姿
 * ========================================================= */

/*
 * 笛卡尔位姿表示。
 *
 * 位置：
 *
 * x_mm
 * y_mm
 * z_mm
 *
 * 单位：mm
 *
 * 姿态：
 *
 * rx_deg
 * ry_deg
 * rz_deg
 *
 * 单位：degree
 *
 * 具体坐标系定义
 * 将由后续 Kinematics API 明确规定。
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


#endif