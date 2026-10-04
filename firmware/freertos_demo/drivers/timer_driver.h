/*
 * 文件：timer_driver.h
 *
 * 用途：
 * 定义 MPS2-AN386 CMSDK APB Timer 驱动接口。
 *
 * 当前使用：
 *
 * Timer0：
 * 32 bit 自由运行高分辨率计时器，
 * 用于性能测量。
 *
 * Timer1：
 * 周期中断定时器，
 * 用于周期事件和后续实时控制任务。
 */

#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include <stdint.h>

#include "error_code.h"


/* =========================================================
 * Periodic Timer Callback
 * ========================================================= */

/*
 * Timer1 周期中断 Callback。
 *
 * Callback 在 ISR Context 中执行。
 *
 * 因此：
 *
 * 1. 必须尽可能短；
 * 2. 不允许阻塞；
 * 3. 如果调用 FreeRTOS API，
 *    必须使用 FromISR 版本。
 */
typedef void (*timer_driver_periodic_callback_t)(
    void
);


/* =========================================================
 * Driver Init
 * ========================================================= */

/**
 * @brief 初始化 Timer Driver。
 *
 * Timer0：
 *
 * 配置为 32 bit 自由运行向下计数器，
 * 不开启 Interrupt。
 *
 * Timer1：
 *
 * 初始化为停止状态，
 * 周期中断暂不开启。
 */
void timer_driver_init(void);


/* =========================================================
 * Timer0 - Performance Counter
 * ========================================================= */

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
 * 使用无符号减法，
 * 因此可以正确处理一次
 * 32 bit Counter Wrap。
 */
uint32_t timer_driver_elapsed_ticks(
    uint32_t start_counter,
    uint32_t end_counter
);


/**
 * @brief 获取 Timer 输入时钟频率。
 *
 * @return
 * Timer 每秒 Tick 数。
 *
 * 当前 QEMU MPS2-AN386：
 *
 * 25,000,000 Hz。
 */
uint32_t timer_driver_get_frequency_hz(void);


/* =========================================================
 * Timer1 - Periodic Interrupt
 * ========================================================= */

/**
 * @brief 启动 Timer1 周期中断。
 *
 * @param[in] frequency_hz
 * 周期中断频率。
 *
 * 单位：
 * Hz。
 *
 * 例如：
 *
 * 100 Hz -> 10 ms
 * 1000 Hz -> 1 ms
 *
 * @param[in] callback
 * 周期中断 Callback。
 *
 * 允许为 NULL / 0。
 *
 * 即使 Callback 为空，
 * Driver 仍会：
 *
 * 1. 处理 Timer IRQ；
 * 2. 清除 Interrupt；
 * 3. 更新 IRQ Counter。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 启动成功。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * frequency_hz 为 0。
 *
 * ROBOT_STATUS_ERROR_OUT_OF_RANGE：
 * 所要求频率无法由当前 Timer 时钟生成。
 */
robot_status_t timer_driver_start_periodic(
    uint32_t frequency_hz,
    timer_driver_periodic_callback_t callback
);


/**
 * @brief 停止 Timer1 周期中断。
 */
void timer_driver_stop_periodic(void);


/**
 * @brief 获取 Timer1 实际配置的周期中断频率。
 *
 * @return
 * 当前配置频率。
 *
 * Timer 尚未启动时返回 0。
 */
uint32_t timer_driver_get_periodic_frequency_hz(void);


/**
 * @brief 获取 Timer1 IRQ 累计次数。
 *
 * 每进入一次 TIMER1_IRQHandler，
 * Counter 增加 1。
 *
 * @return
 * 自最近一次 start_periodic()
 * 以来的 IRQ Count。
 */
uint32_t timer_driver_get_periodic_irq_count(void);


/**
 * @brief 判断 Timer1 周期定时器是否正在运行。
 *
 * @return
 * 1：正在运行。
 * 0：未运行。
 */
uint8_t timer_driver_is_periodic_running(void);


#endif