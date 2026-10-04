/*
 * 文件：motor_driver.c
 *
 * 用途：
 * 实现六轴机器人关节电机驱动抽象层。
 *
 * 当前实现属于 Simulation Backend：
 *
 * 1. 保存规范化目标关节位置；
 * 2. 保存规范化位置反馈；
 * 3. 重建连续关节位置；
 * 4. 正确处理 ±180° Angle Wrap；
 * 5. 根据连续位置差计算关节速度；
 * 6. 向上层提供统一 Motor API。
 *
 * 本模块不依赖 UART、Protocol 或 FreeRTOS。
 *
 * 因此将来切换到真实 Servo / Encoder Backend 时，
 * 上层 Algorithm / Control 模块仍可以使用相同接口。
 */

#include <stddef.h>
#include <stdint.h>

#include "motor_driver.h"


/* =========================================================
 * Driver 内部状态
 * ========================================================= */

static uint8_t
    motor_driver_initialized = 0U;


/*
 * 标记是否已经收到过至少一组有效 Feedback。
 */
static uint8_t
    motor_feedback_valid = 0U;


/*
 * 当前目标关节位置。
 *
 * Canonical Angle：
 *
 * [-180°, 180°)
 */
static robot_joint_angles_t
    motor_target_positions;


/*
 * 最近一次规范化关节位置反馈。
 *
 * Canonical Angle：
 *
 * [-180°, 180°)
 */
static robot_joint_angles_t
    motor_feedback_positions;


/*
 * 连续关节位置。
 *
 * 不在 ±180° 处回绕。
 */
static robot_joint_positions_t
    motor_feedback_unwrapped_positions;


/*
 * 根据连续位置变化计算得到的关节速度。
 *
 * 单位：
 * degree / second。
 */
static robot_joint_velocities_t
    motor_feedback_velocities;


/* =========================================================
 * Angle Normalize
 * ========================================================= */

/*
 * 将任意 0.01° 整数角规范化到：
 *
 * [-18000, 18000)
 *
 * 即：
 *
 * [-180°, 180°)
 */
static robot_joint_angle_t
motor_driver_normalize_angle_raw(
    int32_t raw_angle
)
{
    int32_t normalized;


    normalized =
        raw_angle
        %
        ROBOT_JOINT_FULL_TURN_RAW;


    if (
        normalized
        >= ROBOT_JOINT_HALF_TURN_RAW
    )
    {
        normalized -=
            ROBOT_JOINT_FULL_TURN_RAW;
    }
    else if (
        normalized
        <
        -ROBOT_JOINT_HALF_TURN_RAW
    )
    {
        normalized +=
            ROBOT_JOINT_FULL_TURN_RAW;
    }


    return
        (robot_joint_angle_t)normalized;
}


/* =========================================================
 * Shortest Angular Delta
 * ========================================================= */

/*
 * 计算两个 Canonical Angle 之间的
 * 最短角位移。
 *
 * 例如：
 *
 * previous = +179°
 * current  = -179°
 *
 * 普通减法：
 *
 * -358°   X
 *
 * 本函数：
 *
 * +2°     OK
 *
 * 返回单位：
 *
 * 0.01 degree
 */
static int32_t
motor_driver_shortest_delta_raw(
    robot_joint_angle_t current,
    robot_joint_angle_t previous
)
{
    int32_t delta;


    delta =
        (int32_t)current
        -
        (int32_t)previous;


    delta =
        (int32_t)
        motor_driver_normalize_angle_raw(
            delta
        );


    return
        delta;
}


/* =========================================================
 * Motor Driver 初始化
 * ========================================================= */

robot_status_t motor_driver_init(void)
{
    uint32_t i;


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        motor_target_positions.value[i] =
            0;


        motor_feedback_positions.value[i] =
            0;


        motor_feedback_unwrapped_positions.value[i] =
            0;


        motor_feedback_velocities.value[i] =
            0.0F;
    }


    motor_feedback_valid =
        0U;


    motor_driver_initialized =
        1U;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 设置 Target Position
 * ========================================================= */

robot_status_t motor_driver_set_target_positions(
    const robot_joint_angles_t *targets
)
{
    uint32_t i;


    if (
        targets == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !motor_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        motor_target_positions.value[i] =
            motor_driver_normalize_angle_raw(
                (int32_t)targets->value[i]
            );
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 获取 Target Position
 * ========================================================= */

robot_status_t motor_driver_get_target_positions(
    robot_joint_angles_t *targets
)
{
    uint32_t i;


    if (
        targets == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !motor_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        targets->value[i] =
            motor_target_positions.value[i];
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 更新 Position Feedback
 * ========================================================= */

robot_status_t motor_driver_update_feedback(
    const robot_joint_angles_t *feedback,
    robot_real_t delta_time_s
)
{
    uint32_t i;


    if (
        feedback == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !motor_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        robot_joint_angle_t
            current_angle;


        current_angle =
            motor_driver_normalize_angle_raw(
                (int32_t)feedback->value[i]
            );


        if (
            motor_feedback_valid
        )
        {
            int32_t
                delta_raw;


            delta_raw =
                motor_driver_shortest_delta_raw(
                    current_angle,
                    motor_feedback_positions.value[i]
                );


            /*
             * 使用最短角位移更新连续位置。
             */
            motor_feedback_unwrapped_positions.value[i]
                +=
                delta_raw;


            /*
             * 时间间隔有效时，
             * 根据经过 Wrap Correction 的
             * 连续角位移计算速度。
             */
            if (
                delta_time_s > 0.0F
            )
            {
                robot_real_t
                    delta_degree;


                delta_degree =
                    (robot_real_t)delta_raw
                    *
                    ROBOT_JOINT_ANGLE_UNIT_DEG;


                motor_feedback_velocities.value[i] =
                    delta_degree
                    /
                    delta_time_s;
            }
            else
            {
                motor_feedback_velocities.value[i] =
                    0.0F;
            }
        }
        else
        {
            /*
             * 第一组 Feedback
             * 没有历史状态可用于展开。
             *
             * 因此将 Canonical Angle
             * 作为连续位置初始值。
             */
            motor_feedback_unwrapped_positions.value[i] =
                (robot_joint_position_t)current_angle;


            motor_feedback_velocities.value[i] =
                0.0F;
        }


        /*
         * 保存最新 Canonical Feedback。
         */
        motor_feedback_positions.value[i] =
            current_angle;
    }


    motor_feedback_valid =
        1U;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 获取 Canonical Position Feedback
 * ========================================================= */

robot_status_t motor_driver_get_positions(
    robot_joint_angles_t *positions
)
{
    uint32_t i;


    if (
        positions == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !motor_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    if (
        !motor_feedback_valid
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        positions->value[i] =
            motor_feedback_positions.value[i];
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 获取 Continuous Position Feedback
 * ========================================================= */

robot_status_t motor_driver_get_unwrapped_positions(
    robot_joint_positions_t *positions
)
{
    uint32_t i;


    if (
        positions == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !motor_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    if (
        !motor_feedback_valid
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        positions->value[i] =
            motor_feedback_unwrapped_positions.value[i];
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 获取 Velocity Feedback
 * ========================================================= */

robot_status_t motor_driver_get_velocities(
    robot_joint_velocities_t *velocities
)
{
    uint32_t i;


    if (
        velocities == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !motor_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    if (
        !motor_feedback_valid
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        velocities->value[i] =
            motor_feedback_velocities.value[i];
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Feedback 状态查询
 * ========================================================= */

uint8_t motor_driver_has_feedback(void)
{
    if (
        !motor_driver_initialized
    )
    {
        return 0U;
    }


    return
        motor_feedback_valid;
}