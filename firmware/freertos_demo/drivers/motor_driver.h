/*
 * 文件：motor_driver.h
 *
 * 用途：
 * 定义六轴机器人关节电机驱动抽象接口。
 *
 * 本模块向上层提供统一的 Motor API，
 * 屏蔽底层状态来源。
 *
 * 当前仿真环境中：
 *
 * Joint Feedback
 * 来自 Python / PyBullet。
 *
 * 将来迁移到真实硬件时：
 *
 * Joint Feedback
 * 可以来自 Encoder / Servo Driver。
 *
 * 上层控制算法不需要因此修改接口。
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * Motor Driver 初始化
 * ========================================================= */

/**
 * @brief 初始化六轴 Motor Driver。
 *
 * 初始化：
 *
 * 1. Target Position；
 * 2. Feedback Position；
 * 3. Feedback Velocity；
 * 4. 内部状态标志。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 初始化成功。
 */
robot_status_t motor_driver_init(void);


/* =========================================================
 * Target Position API
 * ========================================================= */

/**
 * @brief 设置六轴目标位置。
 *
 * @param targets
 * 六轴目标关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化。
 */
robot_status_t motor_driver_set_target_positions(
    const robot_joint_angles_t *targets
);


/**
 * @brief 获取当前六轴目标位置。
 *
 * @param targets
 * 输出当前目标关节角。
 */
robot_status_t motor_driver_get_target_positions(
    robot_joint_angles_t *targets
);


/* =========================================================
 * Feedback API
 * ========================================================= */

/**
 * @brief 更新六轴关节位置反馈。
 *
 * 当前仿真环境中，
 * feedback 来自 PyBullet。
 *
 * 后续真实硬件中，
 * 可以由 Encoder / Servo Feedback 替换。
 *
 * @param feedback
 * 当前六轴位置反馈。
 *
 * @param delta_time_s
 * 当前反馈与上一反馈之间的时间，
 * 单位为 second。
 *
 * 如果 delta_time_s <= 0，
 * 仍然更新位置，
 * 但当前速度置为 0。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 成功。
 */
robot_status_t motor_driver_update_feedback(
    const robot_joint_angles_t *feedback,
    robot_real_t delta_time_s
);


/**
 * @brief 获取最近一次六轴位置反馈。
 */
robot_status_t motor_driver_get_positions(
    robot_joint_angles_t *positions
);


/**
 * @brief 获取根据位置反馈计算出的六轴速度。
 *
 * 单位：
 * degree / second。
 */
robot_status_t motor_driver_get_velocities(
    robot_joint_velocities_t *velocities
);


/**
 * @brief 判断是否已经收到过有效反馈。
 *
 * @return
 * 1：
 * 已经收到反馈。
 *
 * 0：
 * 尚未收到反馈。
 */
uint8_t motor_driver_has_feedback(void);


#endif