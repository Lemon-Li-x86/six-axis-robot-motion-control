#include <stdint.h>

#define UART0_BASE 0x40004000UL
#define UART0_DATA (*(volatile uint32_t *)(UART0_BASE + 0x000UL))
#define UART0_STATE (*(volatile uint32_t *)(UART0_BASE + 0x004UL))

static void uart_putc(char c)
{
    while (UART0_STATE & 1U)
    {
    }

    UART0_DATA = (uint32_t)c;
}

static void uart_puts(const char *s)
{
    while (*s)
    {
        uart_putc(*s++);
    }
}

int main(void)
{
    uart_puts("Hello from Cortex-M4!\n");

    while (1)
    {
    }
}
