/*
 * 文件：timer_driver.h
 *
 * 用途：
 * 定义 MPS2-AN386 CMSDK APB Timer0 的基础计时接口。
 *
 * 当前主要用于：
 *
 * 1. Protocol Parser 执行时间测量；
 * 2. UART ISR -> ProtocolRX Task 唤醒时间测量。
 *
 * Timer0 当前配置为自由运行的 32 bit 向下计数器。
 */

#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>


/**
 * @brief 初始化 CMSDK APB Timer0。
 *
 * Timer0 被配置为：
 *
 * 1. 使用系统 PCLK；
 * 2. 32 bit 自由运行向下计数；
 * 3. 不开启 Timer Interrupt。
 */
void timer_driver_init(void);


/**
 * @brief 读取当前 Timer0 原始计数值。
 *
 * @return
 * 当前 32 bit 向下计数值。
 */
uint32_t timer_driver_get_counter(void);


/**
 * @brief 计算两个 Timer0 采样点之间经过的 Tick 数。
 *
 * @param[in] start_counter
 * 开始时的向下计数值。
 *
 * @param[in] end_counter
 * 结束时的向下计数值。
 *
 * @return
 * 经过的 Timer Tick 数。
 *
 * @note
 * 使用无符号减法，因此可以正确处理一次
 * 32 bit Counter Wrap。
 */
uint32_t timer_driver_elapsed_ticks(
    uint32_t start_counter,
    uint32_t end_counter
);


/**
 * @brief 获取 Timer0 输入时钟频率。
 *
 * @return
 * Timer0 每秒 Tick 数。
 *
 * 当前 QEMU MPS2-AN386 为：
 *
 * 25,000,000 Hz。
 */
uint32_t timer_driver_get_frequency_hz(void);


#endif