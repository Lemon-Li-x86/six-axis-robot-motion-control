/*
 * 文件：motor_driver.h
 *
 * 用途：
 * 定义六轴机器人关节电机驱动抽象接口。
 *
 * 当前仿真环境中，
 * Joint Feedback 来自 Python / PyBullet。
 *
 * 后续迁移到真实硬件时，
 * Feedback 可以由 Encoder / Servo Driver 提供，
 * 上层 Algorithm / Control 模块无需改变接口。
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


/**
 * @brief 初始化六轴 Motor Driver。
 *
 * 初始化目标位置、位置反馈、速度反馈和内部状态。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 初始化成功。
 */
robot_status_t motor_driver_init(void);


/**
 * @brief 设置六轴目标关节位置。
 *
 * @param[in] targets
 * 六轴目标关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 设置成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * targets 为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化。
 */
robot_status_t motor_driver_set_target_positions(
    const robot_joint_angles_t *targets
);


/**
 * @brief 获取当前六轴目标关节位置。
 *
 * @param[out] targets
 * 输出当前目标关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 获取成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * targets 为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化。
 */
robot_status_t motor_driver_get_target_positions(
    robot_joint_angles_t *targets
);


/**
 * @brief 更新六轴关节位置反馈。
 *
 * 当前仿真环境中，
 * Feedback 来自 PyBullet。
 *
 * @param[in] feedback
 * 当前六轴位置反馈。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param[in] delta_time_s
 * 当前 Feedback 与上一组 Feedback 的时间间隔。
 *
 * 单位：
 * second。
 *
 * 当 delta_time_s <= 0 时，
 * 仍然更新位置，但速度统一置为 0。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 更新成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * feedback 为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化。
 */
robot_status_t motor_driver_update_feedback(
    const robot_joint_angles_t *feedback,
    robot_real_t delta_time_s
);


/**
 * @brief 获取最近一次六轴位置反馈。
 *
 * @param[out] positions
 * 输出六轴关节位置。
 *
 * 单位：
 * 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 获取成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * positions 为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化，
 * 或尚未收到有效 Feedback。
 */
robot_status_t motor_driver_get_positions(
    robot_joint_angles_t *positions
);


/**
 * @brief 获取六轴关节速度反馈。
 *
 * @param[out] velocities
 * 输出根据连续位置反馈计算得到的关节速度。
 *
 * 单位：
 * degree / second。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 获取成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * velocities 为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化，
 * 或尚未收到有效 Feedback。
 */
robot_status_t motor_driver_get_velocities(
    robot_joint_velocities_t *velocities
);


/**
 * @brief 判断 Motor Driver 是否已经收到有效反馈。
 *
 * @return
 * 1：
 * 已经收到至少一组有效 Feedback。
 *
 * 0：
 * Driver 尚未初始化，
 * 或尚未收到有效 Feedback。
 */
uint8_t motor_driver_has_feedback(void);


#endif