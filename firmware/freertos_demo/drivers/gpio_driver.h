/*
 * 文件：gpio_driver.h
 *
 * 用途：
 * 定义通用 GPIO 输出控制接口。
 *
 * 当前项目运行于：
 *
 * QEMU MPS2-AN386
 *
 * 当前 QEMU 环境没有提供可直接使用的
 * CMSDK AHB GPIO 实际行为，因此驱动支持：
 *
 * 1. CMSDK GPIO MMIO Backend；
 * 2. QEMU Shadow Backend。
 *
 * 上层模块统一使用本文件提供的接口，
 * 不直接访问 GPIO 寄存器。
 */

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdint.h>

#include "error_code.h"


/* =========================================================
 * GPIO 基本配置
 * ========================================================= */

/*
 * 当前 GPIO0 按 32 bit GPIO Port 处理。
 */
#define GPIO_DRIVER_PIN_COUNT 32U


/* =========================================================
 * GPIO Level
 * ========================================================= */

typedef enum
{
    GPIO_DRIVER_LEVEL_LOW = 0,

    GPIO_DRIVER_LEVEL_HIGH = 1

} gpio_driver_level_t;


/* =========================================================
 * GPIO Driver API
 * ========================================================= */

/**
 * @brief 初始化 GPIO Driver。
 *
 * QEMU Shadow Backend：
 *
 * 1. 清空 Output Enable 状态；
 * 2. 清空 Output Data 状态。
 *
 * CMSDK MMIO Backend：
 *
 * 软件 Shadow 同样被清零，
 * 后续由 gpio_driver_configure_output()
 * 配置具体输出引脚。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 初始化成功。
 */
robot_status_t gpio_driver_init(void);


/**
 * @brief 将指定 GPIO0 引脚配置为普通输出。
 *
 * @param[in] pin
 * GPIO Pin Number。
 *
 * 有效范围：
 *
 * 0 ~ 31
 *
 * @return
 * ROBOT_STATUS_OK：
 * 配置成功。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * Pin Number 超出范围。
 */
robot_status_t gpio_driver_configure_output(
    uint8_t pin
);


/**
 * @brief 设置指定 GPIO 输出引脚电平。
 *
 * @param[in] pin
 * GPIO Pin Number。
 *
 * @param[in] level
 * GPIO_DRIVER_LEVEL_LOW
 * 或
 * GPIO_DRIVER_LEVEL_HIGH。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 写入成功。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化，
 * 或该 Pin 尚未配置为输出。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * Pin Number 超出范围。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * Level 非法。
 */
robot_status_t gpio_driver_write(
    uint8_t pin,
    gpio_driver_level_t level
);


/**
 * @brief 翻转指定 GPIO 输出引脚电平。
 *
 * HIGH -> LOW
 * LOW  -> HIGH
 *
 * @param[in] pin
 * GPIO Pin Number。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 翻转成功。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化，
 * 或该 Pin 尚未配置为输出。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * Pin Number 超出范围。
 */
robot_status_t gpio_driver_toggle(
    uint8_t pin
);


/**
 * @brief 获取当前 GPIO 输出锁存状态。
 *
 * 当前接口读取的是 Driver 保存的
 * Output State，
 * 而不是物理 GPIO Input。
 *
 * 因此该接口既可以用于：
 *
 * 1. MMIO Backend；
 * 2. QEMU Shadow Backend；
 * 3. 后续 Unit Test。
 *
 * @param[in] pin
 * GPIO Pin Number。
 *
 * @param[out] level
 * 当前输出电平。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 读取成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * level 为空。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * Driver 尚未初始化，
 * 或该 Pin 尚未配置为输出。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * Pin Number 超出范围。
 */
robot_status_t gpio_driver_get_output_level(
    uint8_t pin,
    gpio_driver_level_t *level
);


/**
 * @brief 判断当前是否使用真实 GPIO MMIO Backend。
 *
 * @return
 *
 * 1：
 * 使用 CMSDK GPIO MMIO。
 *
 * 0：
 * 使用 QEMU Shadow Backend。
 */
uint8_t gpio_driver_is_mmio_backend(void);


#endif