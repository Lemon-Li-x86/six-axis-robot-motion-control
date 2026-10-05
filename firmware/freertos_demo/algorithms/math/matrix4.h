/*
 * 文件：matrix4.h
 *
 * 用途：
 * 提供 Algorithm Layer 共用的
 * 4 × 4 矩阵基础运算。
 *
 * 当前主要用于齐次变换矩阵。
 */

#ifndef MATRIX4_H
#define MATRIX4_H

#include "robot_types.h"
#include "error_code.h"


/**
 * @brief 构造 4 × 4 单位矩阵。
 *
 * @param[out] matrix
 * 输出单位矩阵。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * matrix 为空。
 */
robot_status_t matrix4_identity(
    robot_transform_t *matrix
);


/**
 * @brief 计算两个 4 × 4 矩阵的乘积。
 *
 * @param[in] left
 * 左矩阵。
 *
 * @param[in] right
 * 右矩阵。
 *
 * @param[out] result
 * 输出：
 *
 * result = left × right
 *
 * @return
 * ROBOT_STATUS_OK：
 * 计算成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 任一输入或输出指针为空。
 *
 * @note
 * result 允许与 left 或 right
 * 指向同一个对象。
 */
robot_status_t matrix4_multiply(
    const robot_transform_t *left,
    const robot_transform_t *right,
    robot_transform_t *result
);


#endif