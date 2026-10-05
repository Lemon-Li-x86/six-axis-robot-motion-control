/*
 * 文件：matrix4.c
 *
 * 用途：
 * 实现 Algorithm Layer 共用的
 * 4 × 4 矩阵基础运算。
 */

#include <stddef.h>
#include <stdint.h>

#include "matrix4.h"


robot_status_t matrix4_identity(
    robot_transform_t *matrix
)
{
    uint32_t row;
    uint32_t column;

    if (matrix == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    for (row = 0U; row < 4U; row++)
    {
        for (column = 0U; column < 4U; column++)
        {
            matrix->matrix[row][column] =
                (row == column) ? 1.0F : 0.0F;
        }
    }

    return ROBOT_STATUS_OK;
}


robot_status_t matrix4_multiply(
    const robot_transform_t *left,
    const robot_transform_t *right,
    robot_transform_t *result
)
{
    robot_transform_t temporary;

    uint32_t row;
    uint32_t column;
    uint32_t k;

    if (
        left == NULL
        || right == NULL
        || result == NULL
    )
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    /*
     * 先将计算结果写入临时矩阵。
     *
     * 这样即使 result 与 left 或 right
     * 指向同一个对象，也不会在计算过程中
     * 覆盖尚未使用的输入元素。
     */
    for (row = 0U; row < 4U; row++)
    {
        for (column = 0U; column < 4U; column++)
        {
            robot_real_t sum = 0.0F;

            for (k = 0U; k < 4U; k++)
            {
                sum +=
                    left->matrix[row][k]
                    * right->matrix[k][column];
            }

            temporary.matrix[row][column] = sum;
        }
    }

    *result = temporary;

    return ROBOT_STATUS_OK;
}