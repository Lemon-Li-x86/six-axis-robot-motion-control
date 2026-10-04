/*
 * 文件：timer_driver.c
 *
 * 用途：
 * 实现 QEMU MPS2-AN386 CMSDK APB Timer Driver。
 *
 * 当前分工：
 *
 * Timer0
 * ---------------------------------------------------------
 * Base Address = 0x40000000
 *
 * 用作自由运行高分辨率计时器。
 *
 * 用于：
 *
 * 1. Protocol Parser 执行时间；
 * 2. UART ISR -> ProtocolRX Task 唤醒时间；
 * 3. 后续性能统计。
 *
 *
 * Timer1
 * ---------------------------------------------------------
 * Base Address = 0x40001000
 * External IRQ = 9
 *
 * 用作周期中断定时器。
 *
 * 用于：
 *
 * 1. 第2阶段 Timer 周期中断验证；
 * 2. 后续周期控制事件；
 * 3. 后续实时任务触发。
 */

#include <stdint.h>

#include "board.h"
#include "timer_driver.h"


/* =========================================================
 * CMSDK APB Timer Register Layout
 * ========================================================= */

#define TIMER_CTRL_OFFSET \
    0x000UL


#define TIMER_VALUE_OFFSET \
    0x004UL


#define TIMER_RELOAD_OFFSET \
    0x008UL


#define TIMER_INTSTATUS_OFFSET \
    0x00CUL


#define REG32(address) \
    (*(volatile uint32_t *)(address))


/* =========================================================
 * Timer0
 * ========================================================= */

#define TIMER0_BASE \
    0x40000000UL


#define TIMER0_CTRL \
    REG32( \
        TIMER0_BASE \
        + \
        TIMER_CTRL_OFFSET \
    )


#define TIMER0_VALUE \
    REG32( \
        TIMER0_BASE \
        + \
        TIMER_VALUE_OFFSET \
    )


#define TIMER0_RELOAD \
    REG32( \
        TIMER0_BASE \
        + \
        TIMER_RELOAD_OFFSET \
    )


#define TIMER0_INTSTATUS \
    REG32( \
        TIMER0_BASE \
        + \
        TIMER_INTSTATUS_OFFSET \
    )


/* =========================================================
 * Timer1
 * ========================================================= */

#define TIMER1_BASE \
    0x40001000UL


#define TIMER1_CTRL \
    REG32( \
        TIMER1_BASE \
        + \
        TIMER_CTRL_OFFSET \
    )


#define TIMER1_VALUE \
    REG32( \
        TIMER1_BASE \
        + \
        TIMER_VALUE_OFFSET \
    )


#define TIMER1_RELOAD \
    REG32( \
        TIMER1_BASE \
        + \
        TIMER_RELOAD_OFFSET \
    )


#define TIMER1_INTSTATUS \
    REG32( \
        TIMER1_BASE \
        + \
        TIMER_INTSTATUS_OFFSET \
    )


/* =========================================================
 * Timer CTRL Bits
 * ========================================================= */

/*
 * CTRL Bit 0：
 * Timer Enable。
 */
#define TIMER_CTRL_ENABLE \
    (1UL << 0)


/*
 * CTRL Bit 3：
 * Timer Interrupt Enable。
 */
#define TIMER_CTRL_IRQ_ENABLE \
    (1UL << 3)


/* =========================================================
 * Timer Interrupt
 * ========================================================= */

/*
 * INTSTATUS Bit 0：
 *
 * Write 1 to Clear。
 */
#define TIMER_INTSTATUS_IRQ \
    (1UL << 0)


/* =========================================================
 * Timer0 Configuration
 * ========================================================= */

#define TIMER0_COUNTER_RELOAD_VALUE \
    0xFFFFFFFFUL


/* =========================================================
 * Cortex-M NVIC
 * ========================================================= */

#define NVIC_ISER0 \
    (*(volatile uint32_t *)0xE000E100UL)


#define NVIC_ICER0 \
    (*(volatile uint32_t *)0xE000E180UL)


#define NVIC_ICPR0 \
    (*(volatile uint32_t *)0xE000E280UL)


#define NVIC_IPR_BASE \
    0xE000E400UL


/*
 * QEMU MPS2-AN386：
 *
 * Timer0 -> IRQ 8
 * Timer1 -> IRQ 9
 *
 * 当前只为 Timer1 开启 Interrupt。
 */
#define TIMER1_IRQ_NUMBER \
    9U


#define TIMER1_IRQ_MASK \
    (1UL << TIMER1_IRQ_NUMBER)


#define TIMER1_IRQ_PRIORITY \
    (*(volatile uint8_t *)( \
        NVIC_IPR_BASE \
        + \
        TIMER1_IRQ_NUMBER \
    ))


/*
 * Timer ISR 后续可能通过 Callback
 * 使用 FreeRTOS FromISR API。
 *
 * 因此不使用默认最高优先级 0。
 *
 * 与 UART RX ISR 保持一致。
 */
#define TIMER1_IRQ_PRIORITY_VALUE \
    0x80U


/* =========================================================
 * Timer1 State
 * ========================================================= */

static volatile uint32_t
    timer1_irq_count = 0U;


static uint32_t
    timer1_frequency_hz = 0U;


static uint8_t
    timer1_running = 0U;


static timer_driver_periodic_callback_t
    timer1_callback = 0;


/* =========================================================
 * Driver Init
 * ========================================================= */

void
timer_driver_init(void)
{
    /* =====================================================
     * Timer0
     *
     * 自由运行性能计时器。
     * ===================================================== */

    /*
     * 配置期间停止 Timer0。
     */
    TIMER0_CTRL =
        0U;


    /*
     * 清除可能残留的 Interrupt Status。
     */
    TIMER0_INTSTATUS =
        TIMER_INTSTATUS_IRQ;


    /*
     * 设置最大 Reload Value。
     *
     * Timer0 从 0xFFFFFFFF
     * 持续向下计数。
     */
    TIMER0_RELOAD =
        TIMER0_COUNTER_RELOAD_VALUE;


    /*
     * Timer0 只开启 Counter，
     * 不开启 IRQ。
     */
    TIMER0_CTRL =
        TIMER_CTRL_ENABLE;


    /* =====================================================
     * Timer1
     *
     * 周期中断 Timer。
     * ===================================================== */

    /*
     * 先停止 Timer1。
     */
    TIMER1_CTRL =
        0U;


    /*
     * 清除可能残留的 Timer1 IRQ。
     */
    TIMER1_INTSTATUS =
        TIMER_INTSTATUS_IRQ;


    /*
     * Reload 初始设为 0。
     *
     * Timer1 尚未启动。
     */
    TIMER1_RELOAD =
        0U;


    /*
     * 关闭 NVIC Timer1 IRQ。
     */
    NVIC_ICER0 =
        TIMER1_IRQ_MASK;


    /*
     * 清除 NVIC Pending。
     */
    NVIC_ICPR0 =
        TIMER1_IRQ_MASK;


    timer1_irq_count =
        0U;


    timer1_frequency_hz =
        0U;


    timer1_running =
        0U;


    timer1_callback =
        0;
}


/* =========================================================
 * Timer0 Counter
 * ========================================================= */

uint32_t
timer_driver_get_counter(void)
{
    return
        TIMER0_VALUE;
}


/* =========================================================
 * Timer0 Elapsed Ticks
 * ========================================================= */

uint32_t
timer_driver_elapsed_ticks(
    uint32_t start_counter,
    uint32_t end_counter
)
{
    /*
     * Timer0 向下计数。
     *
     * unsigned arithmetic
     * 可以自然处理一次
     * 32 bit Counter Wrap。
     */
    return
        start_counter
        -
        end_counter;
}


/* =========================================================
 * Timer Clock Frequency
 * ========================================================= */

uint32_t
timer_driver_get_frequency_hz(void)
{
    return
        (uint32_t)
        BOARD_SYSCLK_HZ;
}


/* =========================================================
 * Start Timer1 Periodic Interrupt
 * ========================================================= */

robot_status_t
timer_driver_start_periodic(
    uint32_t frequency_hz,
    timer_driver_periodic_callback_t callback
)
{
    uint32_t
        ticks_per_period;


    uint32_t
        reload_value;


    /*
     * 0 Hz 没有物理意义。
     */
    if (
        frequency_hz
        ==
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }


    /*
     * 要求频率不能高于 Timer Clock。
     */
    if (
        frequency_hz
        >
        BOARD_SYSCLK_HZ
    )
    {
        return
            ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }


    /*
     * 计算一个周期对应多少个 PCLK Tick。
     */
    ticks_per_period =
        (uint32_t)(
            BOARD_SYSCLK_HZ
            /
            frequency_hz
        );


    /*
     * CMSDK Timer 在 Counter 到达 0 后
     * 会经历 Reload。
     *
     * 为得到 N Tick 周期，
     * Reload 使用：
     *
     * N - 1
     *
     * 至少要求 2 Tick。
     */
    if (
        ticks_per_period
        <
        2U
    )
    {
        return
            ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }


    reload_value =
        ticks_per_period
        -
        1U;


    /*
     * 配置期间停止 Timer1。
     */
    TIMER1_CTRL =
        0U;


    /*
     * 暂时关闭 NVIC IRQ。
     */
    NVIC_ICER0 =
        TIMER1_IRQ_MASK;


    /*
     * 清除 Timer Interrupt Status。
     */
    TIMER1_INTSTATUS =
        TIMER_INTSTATUS_IRQ;


    /*
     * 清除 NVIC Pending。
     */
    NVIC_ICPR0 =
        TIMER1_IRQ_MASK;


    /*
     * 保存 Callback。
     */
    timer1_callback =
        callback;


    /*
     * IRQ Counter 从 0 重新开始。
     */
    timer1_irq_count =
        0U;


    /*
     * 保存配置频率。
     */
    timer1_frequency_hz =
        frequency_hz;


    /*
     * 设置 Timer Reload。
     *
     * CMSDK Timer 在写 RELOAD 时
     * 同时更新 Current Counter。
     */
    TIMER1_RELOAD =
        reload_value;


    /*
     * 设置 Timer1 Interrupt Priority。
     */
    TIMER1_IRQ_PRIORITY =
        TIMER1_IRQ_PRIORITY_VALUE;


    /*
     * 清除最后一次 Pending。
     */
    NVIC_ICPR0 =
        TIMER1_IRQ_MASK;


    /*
     * NVIC 开启 Timer1 IRQ。
     */
    NVIC_ISER0 =
        TIMER1_IRQ_MASK;


    /*
     * 开启：
     *
     * Timer Enable
     * +
     * Timer IRQ Enable
     */
    TIMER1_CTRL =
        TIMER_CTRL_ENABLE
        |
        TIMER_CTRL_IRQ_ENABLE;


    timer1_running =
        1U;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Stop Timer1
 * ========================================================= */

void
timer_driver_stop_periodic(void)
{
    /*
     * 停止硬件 Timer。
     */
    TIMER1_CTRL =
        0U;


    /*
     * 关闭 NVIC IRQ。
     */
    NVIC_ICER0 =
        TIMER1_IRQ_MASK;


    /*
     * 清除 Timer IRQ。
     */
    TIMER1_INTSTATUS =
        TIMER_INTSTATUS_IRQ;


    /*
     * 清除 NVIC Pending。
     */
    NVIC_ICPR0 =
        TIMER1_IRQ_MASK;


    timer1_running =
        0U;


    timer1_frequency_hz =
        0U;


    timer1_callback =
        0;
}


/* =========================================================
 * Timer1 Frequency
 * ========================================================= */

uint32_t
timer_driver_get_periodic_frequency_hz(void)
{
    return
        timer1_frequency_hz;
}


/* =========================================================
 * Timer1 IRQ Counter
 * ========================================================= */

uint32_t
timer_driver_get_periodic_irq_count(void)
{
    return
        timer1_irq_count;
}


/* =========================================================
 * Timer1 Running State
 * ========================================================= */

uint8_t
timer_driver_is_periodic_running(void)
{
    return
        timer1_running;
}


/* =========================================================
 * Timer1 Interrupt Handler
 * ========================================================= */

void
TIMER1_IRQHandler(void)
{
    /*
     * 确认当前确实存在 Timer IRQ。
     */
    if (
        TIMER1_INTSTATUS
        &
        TIMER_INTSTATUS_IRQ
    )
    {
        /*
         * CMSDK Timer：
         *
         * INTSTATUS 为 W1C。
         *
         * Write 1 清除 IRQ。
         */
        TIMER1_INTSTATUS =
            TIMER_INTSTATUS_IRQ;


        /*
         * 记录 IRQ 次数。
         */
        timer1_irq_count++;


        /*
         * Callback 运行于 ISR Context。
         */
        if (
            timer1_callback
            !=
            0
        )
        {
            timer1_callback();
        }
    }
}