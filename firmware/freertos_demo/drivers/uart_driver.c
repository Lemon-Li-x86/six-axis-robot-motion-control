#include <stddef.h>
#include <stdint.h>

#include "uart_driver.h"
#include "ring_buffer.h"


/* =========================================================
 * UART0 Registers
 * ========================================================= */

#define UART0_BASE 0x40004000UL

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
 * UART CTRL
 * ========================================================= */

#define UART_CTRL_TX_ENABLE             (1UL << 0)
#define UART_CTRL_RX_ENABLE             (1UL << 1)
#define UART_CTRL_TX_INTERRUPT_ENABLE   (1UL << 2)
#define UART_CTRL_RX_INTERRUPT_ENABLE   (1UL << 3)


/* =========================================================
 * UART Interrupt Status
 * ========================================================= */

#define UART_INTERRUPT_TX (1UL << 0)
#define UART_INTERRUPT_RX (1UL << 1)


/* =========================================================
 * NVIC
 * ========================================================= */

#define NVIC_ISER0 \
    (*(volatile uint32_t *)0xE000E100UL)

#define NVIC_ICER0 \
    (*(volatile uint32_t *)0xE000E180UL)

#define NVIC_ICPR0 \
    (*(volatile uint32_t *)0xE000E280UL)

#define NVIC_IPR_BASE 0xE000E400UL


/*
 * MPS2-AN386：
 *
 * IRQ0 = UART0 RX
 * IRQ1 = UART0 TX
 */
#define UART0_RX_IRQ_NUMBER 0U
#define UART0_TX_IRQ_NUMBER 1U

#define UART0_RX_IRQ_MASK \
    (1UL << UART0_RX_IRQ_NUMBER)

#define UART0_TX_IRQ_MASK \
    (1UL << UART0_TX_IRQ_NUMBER)

#define UART0_RX_IRQ_PRIORITY \
    (*(volatile uint8_t *)(NVIC_IPR_BASE + UART0_RX_IRQ_NUMBER))

#define UART0_TX_IRQ_PRIORITY \
    (*(volatile uint8_t *)(NVIC_IPR_BASE + UART0_TX_IRQ_NUMBER))

#define UART_IRQ_PRIORITY_VALUE 0x80U


/* =========================================================
 * Driver State
 * ========================================================= */

static ring_buffer_t uart_rx_buffer;
static ring_buffer_t uart_tx_buffer;

static volatile uint32_t uart_rx_drop_count = 0U;

static volatile uint8_t uart_tx_active = 0U;
static uint8_t uart_driver_initialized = 0U;

static uart_driver_rx_event_callback_t
    uart_rx_event_callback = NULL;


/* =========================================================
 * Internal Helpers
 * ========================================================= */

static uint32_t uart_driver_ring_buffer_used(
    const ring_buffer_t *buffer
)
{
    uint32_t head = buffer->head;
    uint32_t tail = buffer->tail;

    if (head >= tail)
    {
        return head - tail;
    }

    return (
        RING_BUFFER_CAPACITY
        - tail
        + head
    );
}


static uint32_t uart_driver_tx_free_space(void)
{
    uint32_t used =
        uart_driver_ring_buffer_used(
            &uart_tx_buffer
        );

    /*
     * Ring Buffer 保留一个位置区分 full / empty。
     */
    return (
        RING_BUFFER_CAPACITY
        - 1U
        - used
    );
}


/* =========================================================
 * Initialization
 * ========================================================= */

robot_status_t uart_driver_init(void)
{
    ring_buffer_init(&uart_rx_buffer);
    ring_buffer_init(&uart_tx_buffer);

    uart_rx_drop_count = 0U;
    uart_tx_active = 0U;

    uart_rx_event_callback = NULL;

    UART0_CTRL = 0U;

    /*
     * CMSDK APB UART 要求 BAUDDIV >= 16。
     *
     * 保持项目已有配置。
     */
    UART0_BAUDDIV = 16U;

    /*
     * 清除 RX / TX 中断状态。
     */
    UART0_INTSTATUS =
        UART_INTERRUPT_RX
        |
        UART_INTERRUPT_TX;

    /*
     * 初始化阶段关闭两个 UART NVIC IRQ。
     */
    NVIC_ICER0 =
        UART0_RX_IRQ_MASK
        |
        UART0_TX_IRQ_MASK;

    NVIC_ICPR0 =
        UART0_RX_IRQ_MASK
        |
        UART0_TX_IRQ_MASK;

    /*
     * 初始只开启 UART RX / TX 功能，
     * 暂不打开 RX/TX Interrupt。
     */
    UART0_CTRL =
        UART_CTRL_TX_ENABLE
        |
        UART_CTRL_RX_ENABLE;

    uart_driver_initialized = 1U;

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * RX Callback
 * ========================================================= */

void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
)
{
    uart_rx_event_callback = callback;
}


/* =========================================================
 * Enable RX Interrupt
 * ========================================================= */

robot_status_t uart_driver_enable_rx_interrupt(void)
{
    if (!uart_driver_initialized)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }

    UART0_INTSTATUS =
        UART_INTERRUPT_RX;

    NVIC_ICPR0 =
        UART0_RX_IRQ_MASK;

    UART0_RX_IRQ_PRIORITY =
        UART_IRQ_PRIORITY_VALUE;

    UART0_CTRL |=
        UART_CTRL_RX_INTERRUPT_ENABLE;

    NVIC_ISER0 =
        UART0_RX_IRQ_MASK;

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Interrupt Driven TX
 * ========================================================= */

robot_status_t uart_driver_write(
    const uint8_t *data,
    uint32_t length
)
{
    uint32_t i;
    uint8_t first_byte;

    if (!uart_driver_initialized)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }

    if (data == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    if (length == 0U)
    {
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }

    /*
     * 暂时屏蔽 TX IRQ。
     *
     * 这样 Task 在检查空间并向 TX Ring Buffer
     * 写数据时，TX ISR 不会同时修改 tail。
     */
    NVIC_ICER0 =
        UART0_TX_IRQ_MASK;

    /*
     * 要求本次写入必须整体进入 Ring Buffer。
     *
     * 不允许只写半个协议帧。
     */
    if (length > uart_driver_tx_free_space())
    {
        if (uart_tx_active)
        {
            NVIC_ISER0 =
                UART0_TX_IRQ_MASK;
        }

        return ROBOT_STATUS_ERROR_BUFFER_FULL;
    }

    for (i = 0U; i < length; i++)
    {
        if (!ring_buffer_write(
                &uart_tx_buffer,
                data[i]))
        {
            /*
             * 前面已经检查过可用空间，
             * 正常情况下不应进入这里。
             */
            if (uart_tx_active)
            {
                NVIC_ISER0 =
                    UART0_TX_IRQ_MASK;
            }

            return ROBOT_STATUS_ERROR_INTERNAL;
        }
    }

    /*
     * 已经有 TX 正在进行：
     *
     * 新数据只需排在 TX Ring Buffer 后面，
     * ISR 会继续发送。
     */
    if (uart_tx_active)
    {
        NVIC_ISER0 =
            UART0_TX_IRQ_MASK;

        return ROBOT_STATUS_OK;
    }

    /*
     * 当前 UART TX 空闲。
     *
     * 取出第一个字节作为 kick-start，
     * 后面的字节由 TX IRQ 连续发送。
     */
    if (!ring_buffer_read(
            &uart_tx_buffer,
            &first_byte))
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    UART0_INTSTATUS =
        UART_INTERRUPT_TX;

    NVIC_ICPR0 =
        UART0_TX_IRQ_MASK;

    UART0_TX_IRQ_PRIORITY =
        UART_IRQ_PRIORITY_VALUE;

    UART0_CTRL |=
        UART_CTRL_TX_INTERRUPT_ENABLE;

    uart_tx_active = 1U;

    /*
     * 写入第一个字节。
     *
     * 当硬件完成发送、TXFULL 从 1 变为 0 时，
     * CMSDK UART 将产生 TX Interrupt。
     */
    UART0_DATA =
        (uint32_t)first_byte;

    /*
     * 最后打开 NVIC IRQ。
     *
     * 如果第一个字节已经在 QEMU 中立即完成，
     * TX IRQ 此时已经 pending，
     * 开启后会立即进入 ISR。
     */
    NVIC_ISER0 =
        UART0_TX_IRQ_MASK;

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * RX Ring Buffer
 * ========================================================= */

uint8_t uart_driver_read_byte(
    uint8_t *data
)
{
    if (!uart_driver_initialized ||
        data == NULL)
    {
        return 0U;
    }

    return ring_buffer_read(
        &uart_rx_buffer,
        data
    );
}


uint32_t uart_driver_get_rx_drop_count(void)
{
    return uart_rx_drop_count;
}


/* =========================================================
 * TX State
 * ========================================================= */

uint8_t uart_driver_is_tx_busy(void)
{
    return uart_tx_active;
}


/* =========================================================
 * UART0 RX IRQ
 * ========================================================= */

void UART0_RX_IRQHandler(void)
{
    uint8_t byte;

    if (!(UART0_INTSTATUS & UART_INTERRUPT_RX))
    {
        return;
    }

    /*
     * W1C：Write 1 to Clear。
     */
    UART0_INTSTATUS =
        UART_INTERRUPT_RX;

    byte =
        (uint8_t)(UART0_DATA & 0xFFU);

    if (!ring_buffer_write(
            &uart_rx_buffer,
            byte))
    {
        uart_rx_drop_count++;
    }

    if (uart_rx_event_callback != NULL)
    {
        uart_rx_event_callback();
    }
}


/* =========================================================
 * UART0 TX IRQ
 * ========================================================= */

void UART0_TX_IRQHandler(void)
{
    uint8_t byte;

    if (!(UART0_INTSTATUS & UART_INTERRUPT_TX))
    {
        return;
    }

    /*
     * 当前字节发送完成。
     */
    UART0_INTSTATUS =
        UART_INTERRUPT_TX;

    /*
     * 软件 TX Buffer 还有数据：
     * 继续发送下一个字节。
     */
    if (ring_buffer_read(
            &uart_tx_buffer,
            &byte))
    {
        UART0_DATA =
            (uint32_t)byte;

        return;
    }

    /*
     * Ring Buffer 已空。
     *
     * 当前完整 TX 流已经发送完毕，
     * 关闭 TX IRQ，等待下一次 uart_driver_write()
     * 重新 kick-start。
     */
    UART0_CTRL &=
        ~UART_CTRL_TX_INTERRUPT_ENABLE;

    uart_tx_active = 0U;

    NVIC_ICER0 =
        UART0_TX_IRQ_MASK;

    NVIC_ICPR0 =
        UART0_TX_IRQ_MASK;
}