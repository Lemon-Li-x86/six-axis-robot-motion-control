/*
 * 文件：motor_driver.c
 *
 * 用途：
 * 实现六轴机器人关节电机驱动抽象层。
 *
 * 当前实现属于 Simulation Backend：
 *
 * 1. 保存目标关节位置；
 * 2. 保存位置反馈；
 * 3. 根据连续两次位置反馈计算关节速度；
 * 4. 向上层提供统一 Motor API。
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
 */
static robot_joint_angles_t
    motor_target_positions;


/*
 * 最近一次关节位置反馈。
 */
static robot_joint_angles_t
    motor_feedback_positions;


/*
 * 根据连续位置反馈计算得到的关节速度。
 *
 * 单位：
 * degree / second。
 */
static robot_joint_velocities_t
    motor_feedback_velocities;


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
            targets->value[i];
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


    /*
     * 已经存在上一组有效 Feedback，
     * 且时间间隔有效时，
     * 根据位置差计算关节速度。
     */
    if (
        motor_feedback_valid
        &&
        delta_time_s > 0.0F
    )
    {
        for (
            i = 0U;
            i < ROBOT_JOINT_COUNT;
            i++
        )
        {
            int32_t
                delta_raw;


            robot_real_t
                delta_degree;


            delta_raw =
                (int32_t)feedback->value[i]
                -
                (int32_t)motor_feedback_positions.value[i];


            delta_degree =
                (robot_real_t)delta_raw
                *
                ROBOT_JOINT_ANGLE_UNIT_DEG;


            motor_feedback_velocities.value[i] =
                delta_degree
                /
                delta_time_s;
        }
    }
    else
    {
        /*
         * 第一组 Feedback，
         * 或时间间隔无效时，
         * 当前速度统一置零。
         */
        for (
            i = 0U;
            i < ROBOT_JOINT_COUNT;
            i++
        )
        {
            motor_feedback_velocities.value[i] =
                0.0F;
        }
    }


    /*
     * 保存最新 Position Feedback。
     *
     * 下一次 update_feedback() 调用时，
     * 该值同时作为上一采样点使用。
     */
    for (
        i = 0U;
        i < ROBOT_JOINT_COUNT;
        i++
    )
    {
        motor_feedback_positions.value[i] =
            feedback->value[i];
    }


    motor_feedback_valid =
        1U;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * 获取 Position Feedback
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