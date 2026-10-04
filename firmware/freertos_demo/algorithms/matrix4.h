/*
 * 文件：matrix4.h
 *
 * 用途：
 * 提供机器人运动学所需的
 * 4 × 4 齐次变换矩阵基础运算。
 *
 * 当前提供：
 *
 * 1. 单位矩阵初始化；
 * 2. 4 × 4 矩阵乘法。
 *
 * 本模块不依赖：
 *
 * UART
 * Protocol
 * FreeRTOS
 * Motor Driver
 */

#ifndef MATRIX4_H
#define MATRIX4_H

#include "robot_types.h"
#include "error_code.h"


/**
 * @brief 将 4 × 4 变换矩阵设置为单位矩阵。
 *
 * @param[out] matrix
 * 输出单位矩阵。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 操作成功。
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
 * 计算：
 *
 * result = left × right
 *
 * @param[in] left
 * 左矩阵。
 *
 * @param[in] right
 * 右矩阵。
 *
 * @param[out] result
 * 输出乘积矩阵。
 *
 * @note
 * result 可以与 left 或 right
 * 指向同一个对象。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 计算成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 任意参数为空。
 */
robot_status_t matrix4_multiply(
    const robot_transform_t *left,
    const robot_transform_t *right,
    robot_transform_t *result
);


#endif