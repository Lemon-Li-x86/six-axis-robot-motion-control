#include <stdint.h>

/* UART0 外设的基地址。
 * 对这个 QEMU mps2-an386 平台来说，UART0 的寄存器从这个地址开始。
 */
#define UART0_BASE 0x40004000UL

/* UART0 数据寄存器：
 * 往这里写数据，就相当于让 UART 发送字符。
 */
#define UART0_DATA    (*(volatile uint32_t *)(UART0_BASE + 0x000UL))

/* UART0 状态寄存器：
 * 可以用来查看发送缓冲区是否已满等状态。
 */
#define UART0_STATE   (*(volatile uint32_t *)(UART0_BASE + 0x004UL))

/* UART0 控制寄存器：
 * 用来开启或关闭 UART 的发送、接收功能。
 */
#define UART0_CTRL    (*(volatile uint32_t *)(UART0_BASE + 0x008UL))

/* UART0 波特率分频寄存器：
 * 用于设置串口通信速率相关参数。
 */
#define UART0_BAUDDIV (*(volatile uint32_t *)(UART0_BASE + 0x010UL))

/* UART_STATE 的 bit0：
 * 为 1 时表示发送缓冲区已满。
 */
#define UART_TX_FULL (1U << 0)

/* 初始化 UART0 */
static void uart_init(void)
{
    /* 设置波特率分频值。
     * 这里先采用最简单的测试值。
     */
    UART0_BAUDDIV = 16U;

    /* CTRL bit0 = 1，开启 UART 发送功能。 */
    UART0_CTRL = 1U;
}

/* 发送一个字符 */
static void uart_putc(char c)
{
    /* 如果发送缓冲区满了，就一直等待。 */
    while (UART0_STATE & UART_TX_FULL)
    {
    }

    /* 把字符写入 UART 数据寄存器。 */
    UART0_DATA = (uint32_t)c;
}

/* 发送一整个字符串 */
static void uart_puts(const char *s)
{
    /* 逐个读取字符串中的字符，
     * 直到遇到字符串结尾 '\0'。
     */
    while (*s)
    {
        uart_putc(*s++);
    }
}

/* C 程序入口 */
int main(void)
{
    /* 先初始化 UART。 */
    uart_init();

    /* 通过 UART 输出测试字符串。 */
    uart_puts("Hello from Cortex-M4!\r\n");

    /* 嵌入式程序一般不会“运行完退出”，
     * 所以这里保持无限循环。
     */
    while (1)
    {
    }
}