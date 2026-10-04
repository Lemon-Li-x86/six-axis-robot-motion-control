/*
 * 文件：matrix4.c
 *
 * 用途：
 * 实现机器人运动学所需的
 * 4 × 4 齐次变换矩阵基础运算。
 */

#include <stddef.h>
#include <stdint.h>

#include "matrix4.h"


/* =========================================================
 * Identity Matrix
 * ========================================================= */

robot_status_t matrix4_identity(
    robot_transform_t *matrix
)
{
    uint32_t row;
    uint32_t column;


    if (
        matrix == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    for (
        row = 0U;
        row < 4U;
        row++
    )
    {
        for (
            column = 0U;
            column < 4U;
            column++
        )
        {
            if (
                row == column
            )
            {
                matrix->matrix[row][column] =
                    1.0F;
            }
            else
            {
                matrix->matrix[row][column] =
                    0.0F;
            }
        }
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Matrix Multiplication
 * ========================================================= */

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
        ||
        right == NULL
        ||
        result == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    /*
     * 使用 temporary，
     * 而不是直接写入 result。
     *
     * 这样允许：
     *
     * matrix4_multiply(
     *     &current,
     *     &next,
     *     &current
     * );
     *
     * 不会因为覆盖输入矩阵
     * 导致后续计算错误。
     */
    for (
        row = 0U;
        row < 4U;
        row++
    )
    {
        for (
            column = 0U;
            column < 4U;
            column++
        )
        {
            robot_real_t sum =
                0.0F;


            for (
                k = 0U;
                k < 4U;
                k++
            )
            {
                sum +=
                    left->matrix[row][k]
                    *
                    right->matrix[k][column];
            }


            temporary.matrix[row][column] =
                sum;
        }
    }


    /*
     * 计算全部完成后，
     * 再一次性复制到输出。
     */
    for (
        row = 0U;
        row < 4U;
        row++
    )
    {
        for (
            column = 0U;
            column < 4U;
            column++
        )
        {
            result->matrix[row][column] =
                temporary.matrix[row][column];
        }
    }


    return
        ROBOT_STATUS_OK;
}