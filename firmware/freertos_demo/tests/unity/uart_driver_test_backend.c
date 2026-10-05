#include <stdint.h>

#include "uart_driver_test_backend.h"


volatile uint32_t
    uart_driver_test_uart0_data = 0U;

volatile uint32_t
    uart_driver_test_uart0_state = 0U;

volatile uint32_t
    uart_driver_test_uart0_ctrl = 0U;

volatile uint32_t
    uart_driver_test_uart0_intstatus = 0U;

volatile uint32_t
    uart_driver_test_uart0_bauddiv = 0U;


volatile uint32_t
    uart_driver_test_nvic_iser0 = 0U;

volatile uint32_t
    uart_driver_test_nvic_icer0 = 0U;

volatile uint32_t
    uart_driver_test_nvic_icpr0 = 0U;

volatile uint8_t
    uart_driver_test_nvic_ipr[32];


void uart_driver_test_backend_reset(void)
{
    uint32_t i;

    uart_driver_test_uart0_data = 0U;
    uart_driver_test_uart0_state = 0U;
    uart_driver_test_uart0_ctrl = 0U;
    uart_driver_test_uart0_intstatus = 0U;
    uart_driver_test_uart0_bauddiv = 0U;

    uart_driver_test_nvic_iser0 = 0U;
    uart_driver_test_nvic_icer0 = 0U;
    uart_driver_test_nvic_icpr0 = 0U;

    for (i = 0U; i < 32U; i++)
    {
        uart_driver_test_nvic_ipr[i] = 0U;
    }
}