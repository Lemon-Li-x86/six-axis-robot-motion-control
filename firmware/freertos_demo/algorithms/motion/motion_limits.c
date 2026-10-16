/*
 * 文件：motion_limits.c
 *
 * 用途：
 * 实现关节连续位置限位检查和规范化角度展开。
 *
 * 全部计算使用整数定点格式：
 * 1 unit = 0.01 degree。
 *
 * 使用 int64_t 计算候选圈数和中间结果，
 * 避免连续位置差值计算发生 int32_t 溢出。
 */

#include <stddef.h>
#include <stdint.h>

#include "motion_limits.h"
#include "robot_model.h"


/* =========================================================
 * Integer Division Helpers
 * ========================================================= */

/*
 * 对正除数实现数学意义上的向下取整。
 *
 * C 语言有符号整数除法默认向零截断，
 * 因而负数计算需要额外处理。
 */
static int64_t motion_limits_floor_div(
    int64_t numerator,
    int64_t denominator
)
{
    int64_t quotient;
    int64_t remainder;

    quotient = numerator / denominator;
    remainder = numerator % denominator;

    if (remainder < 0)
    {
        quotient--;
    }

    return quotient;
}


/*
 * 对正除数实现数学意义上的向上取整。
 */
static int64_t motion_limits_ceil_div(
    int64_t numerator,
    int64_t denominator
)
{
    int64_t quotient;
    int64_t remainder;

    quotient = numerator / denominator;
    remainder = numerator % denominator;

    if (remainder > 0)
    {
        quotient++;
    }

    return quotient;
}


/* =========================================================
 * Model Limit Validation
 * ========================================================= */

static robot_status_t motion_limits_validate_model(
    const robot_model_t *model
)
{
    uint32_t i;

    if (model == NULL)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        if (
            model->joint_min_positions.value[i]
            > model->joint_max_positions.value[i]
        )
        {
            return ROBOT_STATUS_ERROR_INTERNAL;
        }
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Continuous Position Limit Check
 * ========================================================= */

robot_status_t motion_limits_check_positions(
    const robot_joint_positions_t *positions
)
{
    const robot_model_t *model;
    uint32_t i;
    robot_status_t status;

    if (positions == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    model = robot_model_get_active();
    status = motion_limits_validate_model(model);

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        if (
            positions->value[i]
                < model->joint_min_positions.value[i]
            || positions->value[i]
                > model->joint_max_positions.value[i]
        )
        {
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
        }
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Canonical Angle Unwrap
 * ========================================================= */

robot_status_t motion_limits_unwrap_nearest(
    const robot_joint_angles_t *canonical_angles,
    const robot_joint_positions_t *current_positions,
    robot_joint_positions_t *target_positions
)
{
    const robot_model_t *model;
    robot_joint_positions_t candidate_positions;
    robot_status_t status;
    uint32_t i;

    if (
        canonical_angles == NULL
        || current_positions == NULL
        || target_positions == NULL
    )
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    model = robot_model_get_active();
    status = motion_limits_validate_model(model);

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    status = motion_limits_check_positions(current_positions);

    if (status != ROBOT_STATUS_OK)
    {
        return status;
    }

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        int64_t angle;
        int64_t current;
        int64_t lower;
        int64_t upper;
        int64_t turns_min;
        int64_t turns_max;
        int64_t turns_nearest;
        int64_t continuous;

        angle = (int64_t)canonical_angles->value[i];
        current = (int64_t)current_positions->value[i];
        lower = (int64_t)model->joint_min_positions.value[i];
        upper = (int64_t)model->joint_max_positions.value[i];

        /*
         * 此处仅接受规范化角度。
         * 超出范围的输入不能静默取模。
         */
        if (
            angle < ROBOT_JOINT_ANGLE_MIN_RAW
            || angle > ROBOT_JOINT_ANGLE_MAX_RAW
        )
        {
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        }

        /*
         * 找到所有合法等效角的圈数区间：
         *
         * lower <= angle + 36000*k <= upper。
         */
        turns_min = motion_limits_ceil_div(
            lower - angle,
            ROBOT_JOINT_FULL_TURN_RAW
        );

        turns_max = motion_limits_floor_div(
            upper - angle,
            ROBOT_JOINT_FULL_TURN_RAW
        );

        if (turns_min > turns_max)
        {
            return ROBOT_STATUS_ERROR_NO_SOLUTION;
        }

        /*
         * 最近等效角：
         *
         * round((current - angle) / 36000)。
         *
         * 恰好半圈的平局向较大连续角取整。
         */
        turns_nearest = motion_limits_floor_div(
            current - angle + ROBOT_JOINT_HALF_TURN_RAW,
            ROBOT_JOINT_FULL_TURN_RAW
        );

        /*
         * 如果最近等效角越限，则夹紧到
         * 最接近当前连续位置的合法圈数。
         */
        if (turns_nearest < turns_min)
        {
            turns_nearest = turns_min;
        }
        else if (turns_nearest > turns_max)
        {
            turns_nearest = turns_max;
        }

        continuous =
            angle
            + turns_nearest * ROBOT_JOINT_FULL_TURN_RAW;

        candidate_positions.value[i] =
            (robot_joint_position_t)continuous;
    }

    /*
     * 六轴全部成功后才写入最终结果。
     * 任一关节失败时，调用者的输出保持不变。
     */
    *target_positions = candidate_positions;

    return ROBOT_STATUS_OK;
}
