/*真正操作UART寄存器*/

#include "uart_driver.h"


/* =========================================================
 * UART0 寄存器定义
 * ========================================================= */

#define UART0_BASE 0x40004000UL

#define UART0_DATA \
    (*(volatile uint32_t *)(UART0_BASE + 0x000UL))

#define UART0_STATE \
    (*(volatile uint32_t *)(UART0_BASE + 0x004UL))

#define UART0_CTRL \
    (*(volatile uint32_t *)(UART0_BASE + 0x008UL))

#define UART0_BAUDDIV \
    (*(volatile uint32_t *)(UART0_BASE + 0x010UL))


/*
 * STATE bit0：
 * TX FIFO 已满。
 */
#define UART_TX_FULL (1U << 0)

/*
 * STATE bit1：
 * RX FIFO 中存在可读数据。
 */
#define UART_RX_FULL (1U << 1)


/* =========================================================
 * UART 单字节发送
 *
 * 该函数只供 uart_driver.c 内部使用，
 * 因此声明为 static。
 * ========================================================= */

static void uart_driver_write_byte(
    uint8_t data
)
{
    /*
     * 如果发送 FIFO 已满，
     * 则等待直到出现可用空间。
     */
    while (UART0_STATE & UART_TX_FULL)
    {
    }

    /*
     * 将一个字节写入 UART 数据寄存器。
     */
    UART0_DATA = (uint32_t)data;
}


/* =========================================================
 * UART 初始化
 * ========================================================= */

void uart_driver_init(void)
{
    /*
     * 设置 UART 波特率分频值。
     *
     * 当前值保持与原有 QEMU UART
     * 配置一致，本次重构不改变行为。
     */
    UART0_BAUDDIV = 16U;

    /*
     * CTRL bit0：使能发送。
     * CTRL bit1：使能接收。
     */
    UART0_CTRL = 3U;
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

    /*
     * 按顺序逐字节发送整个缓冲区。
     */
    for (i = 0U; i < length; i++)
    {
        uart_driver_write_byte(data[i]);
    }
}


/* =========================================================
 * UART 非阻塞单字节接收
 * ========================================================= */

uint8_t uart_driver_read_byte(
    uint8_t *data
)
{
    /*
     * 当前 RX FIFO 中没有数据时，
     * 立即返回，不阻塞当前任务。
     */
    if ((UART0_STATE & UART_RX_FULL) == 0U)
    {
        return 0U;
    }

    /*
     * 从 UART 数据寄存器中
     * 读取最低 8 位数据。
     */
    *data =
        (uint8_t)(UART0_DATA & 0xFFU);

    return 1U;
}