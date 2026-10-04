/*
 * 文件：matrix4.h
 *
 * 用途：
 * 提供 Algorithm Layer 共用的
 * 4 × 4 矩阵基础运算。
 *
 * 当前主要用于齐次变换矩阵。
 *
 * 本模块不属于 Kinematics 私有实现，
 * 后续 Trajectory / Pose Conversion
 * 同样可以复用。
 */

#ifndef MATRIX4_H
#define MATRIX4_H

#include "robot_types.h"
#include "error_code.h"


robot_status_t matrix4_identity(
    robot_transform_t *matrix
);


robot_status_t matrix4_multiply(
    const robot_transform_t *left,
    const robot_transform_t *right,
    robot_transform_t *result
);


#endif