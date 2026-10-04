/*
 * 文件：timer_driver.c
 *
 * 用途：
 * 实现 QEMU MPS2-AN386 CMSDK APB Timer0
 * 的自由运行高分辨率计时接口。
 *
 * Timer0：
 *
 * Base Address = 0x40000000
 * PCLK         = 25 MHz
 *
 * 当前不使用 Timer Interrupt，
 * 只读取自由运行 Counter 进行性能测量。
 */

#include <stdint.h>

#include "board.h"
#include "timer_driver.h"


/* =========================================================
 * Timer0 Register
 * ========================================================= */

#define TIMER0_BASE \
    0x40000000UL


#define TIMER0_CTRL \
    (*(volatile uint32_t *)(TIMER0_BASE + 0x000UL))


#define TIMER0_VALUE \
    (*(volatile uint32_t *)(TIMER0_BASE + 0x004UL))


#define TIMER0_RELOAD \
    (*(volatile uint32_t *)(TIMER0_BASE + 0x008UL))


#define TIMER0_INTSTATUS \
    (*(volatile uint32_t *)(TIMER0_BASE + 0x00CUL))


/* =========================================================
 * Timer Control
 * ========================================================= */

#define TIMER_CTRL_ENABLE \
    (1U << 0)


#define TIMER_COUNTER_RELOAD_VALUE \
    0xFFFFFFFFUL


/* =========================================================
 * Timer Driver API
 * ========================================================= */

void timer_driver_init(void)
{
    /*
     * 配置期间停止 Timer。
     */
    TIMER0_CTRL =
        0U;


    /*
     * 清除可能残留的 Interrupt Status。
     *
     * 当前不启用 Timer Interrupt，
     * 这里只保证初始状态干净。
     */
    TIMER0_INTSTATUS =
        1U;


    /*
     * 设置最大 Reload Value。
     *
     * Timer 将从 0xFFFFFFFF
     * 按 PCLK 向下计数。
     */
    TIMER0_RELOAD =
        TIMER_COUNTER_RELOAD_VALUE;


    /*
     * 只开启 Counter，
     * 不开启 IRQ。
     */
    TIMER0_CTRL =
        TIMER_CTRL_ENABLE;
}


uint32_t timer_driver_get_counter(void)
{
    return
        TIMER0_VALUE;
}


uint32_t timer_driver_elapsed_ticks(
    uint32_t start_counter,
    uint32_t end_counter
)
{
    /*
     * Counter 向下计数。
     *
     * unsigned arithmetic
     * 可以自然处理一次 32 bit Wrap。
     */
    return
        start_counter
        -
        end_counter;
}


uint32_t timer_driver_get_frequency_hz(void)
{
    return
        (uint32_t)BOARD_SYSCLK_HZ;
}