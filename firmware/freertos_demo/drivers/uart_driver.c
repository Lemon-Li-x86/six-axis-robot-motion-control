/*
 * 文件：uart_driver.c
 *
 * 用途：
 * 实现 QEMU MPS2-AN386 平台 UART0 驱动。
 *
 * 本模块负责：
 *
 * 1. UART0 寄存器访问；
 * 2. UART 初始化；
 * 3. UART 数据发送；
 * 4. UART RX 中断；
 * 5. UART RX Ring Buffer；
 * 6. RX Event Callback；
 * 7. RX 丢字节统计。
 *
 * 数据接收路径：
 *
 * UART Hardware
 *      ↓
 * UART0_RX_IRQHandler
 *      ↓
 * Ring Buffer
 *      ↓
 * RX Event Callback
 */

#include "uart_driver.h"
#include "ring_buffer.h"


/* =========================================================
 * UART0 寄存器
 * ========================================================= */

#define UART0_BASE \
    0x40004000UL


#define UART0_DATA \
    (*(volatile uint32_t *)(UART0_BASE + 0x000UL))


#define UART0_STATE \
    (*(volatile uint32_t *)(UART0_BASE + 0x004UL))


#define UART0_CTRL \
    (*(volatile uint32_t *)(UART0_BASE + 0x008UL))


#define UART0_INTSTATUS \
    (*(volatile uint32_t *)(UART0_BASE + 0x00CUL))


#define UART0_BAUDDIV \
    (*(volatile uint32_t *)(UART0_BASE + 0x010UL))


/* =========================================================
 * UART STATE
 * ========================================================= */

#define UART_STATE_TX_FULL \
    (1U << 0)


/* =========================================================
 * UART CTRL
 * ========================================================= */

/* TX Enable */
#define UART_CTRL_TX_ENABLE \
    (1U << 0)

/* RX Enable */
#define UART_CTRL_RX_ENABLE \
    (1U << 1)

/* RX Interrupt Enable */
#define UART_CTRL_RX_INTERRUPT_ENABLE \
    (1U << 3)


/* =========================================================
 * UART Interrupt Status
 * ========================================================= */

#define UART_INTERRUPT_RX \
    (1U << 1)


/* =========================================================
 * Cortex-M NVIC
 * ========================================================= */

/*
 * Interrupt Set Enable Register。
 */
#define NVIC_ISER0 \
    (*(volatile uint32_t *)0xE000E100UL)


/*
 * Interrupt Clear Enable Register。
 */
#define NVIC_ICER0 \
    (*(volatile uint32_t *)0xE000E180UL)


/*
 * Interrupt Clear Pending Register。
 */
#define NVIC_ICPR0 \
    (*(volatile uint32_t *)0xE000E280UL)


/*
 * External IRQ 0 Priority Register。
 *
 * 每个 IRQ Priority 占一个 Byte。
 */
#define NVIC_IRQ0_PRIORITY \
    (*(volatile uint8_t *)0xE000E400UL)


/*
 * MPS2-AN386：
 *
 * UART0 RX = External IRQ 0。
 */
#define UART0_RX_IRQ_NUMBER \
    0U


#define UART0_RX_IRQ_MASK \
    (1U << UART0_RX_IRQ_NUMBER)


/*
 * UART ISR 会调用 FreeRTOS FromISR API。
 *
 * 因此不能继续使用 NVIC 默认最高优先级 0。
 *
 * 这里使用较低的中断优先级，
 * 使 ISR 可以安全调用 RTOS API。
 */
#define UART0_RX_IRQ_PRIORITY_VALUE \
    0x80U


/* =========================================================
 * UART RX Ring Buffer
 * ========================================================= */

static ring_buffer_t uart_rx_buffer;


/*
 * Ring Buffer 已满时，
 * 记录丢失的 RX 字节数量。
 */
static volatile uint32_t
    uart_rx_drop_count = 0U;


/* =========================================================
 * UART RX Event Callback
 * ========================================================= */

static uart_driver_rx_event_callback_t
    uart_rx_event_callback = 0;


/* =========================================================
 * UART 单字节发送
 * ========================================================= */

static void uart_driver_write_byte(
    uint8_t data
)
{
    /*
     * 等待 TX Buffer 可用。
     */
    while (
        UART0_STATE
        & UART_STATE_TX_FULL
    )
    {
    }


    UART0_DATA =
        (uint32_t)data;
}


/* =========================================================
 * UART 初始化
 * ========================================================= */

void uart_driver_init(void)
{
    /*
     * 初始化软件接收缓冲区。
     */
    ring_buffer_init(
        &uart_rx_buffer
    );


    uart_rx_drop_count = 0U;

    uart_rx_event_callback = 0;


    /*
     * 配置期间关闭 UART。
     */
    UART0_CTRL = 0U;


    /*
     * 波特率配置保持当前项目不变。
     */
    UART0_BAUDDIV = 16U;


    /*
     * 清除 UART RX Interrupt 状态。
     */
    UART0_INTSTATUS =
        UART_INTERRUPT_RX;


    /*
     * 暂时关闭 UART0 RX NVIC Interrupt。
     *
     * 后续由 ProtocolRX Task
     * 主动调用 enable 接口开启。
     */
    NVIC_ICER0 =
        UART0_RX_IRQ_MASK;


    /*
     * 清除残留 Pending 状态。
     */
    NVIC_ICPR0 =
        UART0_RX_IRQ_MASK;


    /*
     * 当前只开启 UART TX / RX 功能。
     *
     * 暂时不开 RX Interrupt。
     */
    UART0_CTRL =
        UART_CTRL_TX_ENABLE
        |
        UART_CTRL_RX_ENABLE;
}


/* =========================================================
 * 注册 RX Event Callback
 * ========================================================= */

void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
)
{
    uart_rx_event_callback =
        callback;
}


/* =========================================================
 * 开启 UART RX Interrupt
 * ========================================================= */

void uart_driver_enable_rx_interrupt(void)
{
    /*
     * 首先清除残留的 UART RX Interrupt。
     */
    UART0_INTSTATUS =
        UART_INTERRUPT_RX;


    /*
     * 清除 NVIC Pending 状态。
     */
    NVIC_ICPR0 =
        UART0_RX_IRQ_MASK;


    /*
     * 设置 UART0 RX IRQ Priority。
     *
     * ISR 后续会调用 FreeRTOS
     * FromISR API，因此不能保持
     * Cortex-M 默认最高优先级 0。
     */
    NVIC_IRQ0_PRIORITY =
        UART0_RX_IRQ_PRIORITY_VALUE;


    /*
     * 打开 UART RX Interrupt。
     */
    UART0_CTRL |=
        UART_CTRL_RX_INTERRUPT_ENABLE;


    /*
     * 在 NVIC 中开启 External IRQ 0。
     */
    NVIC_ISER0 =
        UART0_RX_IRQ_MASK;
}


/* =========================================================
 * UART 数据发送
 * ========================================================= */

void uart_driver_write(
    const uint8_t *data,
    uint32_t length
)
{
    uint32_t i;


    for (
        i = 0U;
        i < length;
        i++
    )
    {
        uart_driver_write_byte(
            data[i]
        );
    }
}


/* =========================================================
 * 从 RX Ring Buffer 读取数据
 * ========================================================= */

uint8_t uart_driver_read_byte(
    uint8_t *data
)
{
    return ring_buffer_read(
        &uart_rx_buffer,
        data
    );
}


/* =========================================================
 * 获取 RX 丢字节数量
 * ========================================================= */

uint32_t uart_driver_get_rx_drop_count(void)
{
    return uart_rx_drop_count;
}


/* =========================================================
 * UART0 RX Interrupt Handler
 * ========================================================= */

void UART0_RX_IRQHandler(void)
{
    uint8_t byte;


    /*
     * 确认 RX Interrupt。
     */
    if (
        UART0_INTSTATUS
        & UART_INTERRUPT_RX
    )
    {
        /*
         * 清除 RX Interrupt 状态。
         */
        UART0_INTSTATUS =
            UART_INTERRUPT_RX;


        /*
         * 从 UART DATA Register
         * 读取接收到的字节。
         */
        byte =
            (uint8_t)(
                UART0_DATA
                & 0xFFU
            );


        /*
         * 将数据放入软件 Ring Buffer。
         */
        if (
            !ring_buffer_write(
                &uart_rx_buffer,
                byte
            )
        )
        {
            /*
             * Ring Buffer 已满。
             */
            uart_rx_drop_count++;
        }


        /*
         * 通知上层：
         * UART 收到了新数据。
         *
         * Callback 在 ISR 上下文运行。
         */
        if (
            uart_rx_event_callback
            != 0
        )
        {
            uart_rx_event_callback();
        }
    }
}