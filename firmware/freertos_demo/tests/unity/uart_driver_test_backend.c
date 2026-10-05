/*
 * 文件：uart_driver_test_backend.c
 *
 * 用途：
 * 实现 UART Driver Host Unit Test
 * 使用的模拟 UART / NVIC Register。
 */

#include <stdint.h>

#include "uart_driver_test_backend.h"


/* =========================================================
 * UART0 Test Registers
 * ========================================================= */

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


/* =========================================================
 * NVIC Test Registers
 * ========================================================= */

volatile uint32_t
    uart_driver_test_nvic_iser0 = 0U;

volatile uint32_t
    uart_driver_test_nvic_icer0 = 0U;

volatile uint32_t
    uart_driver_test_nvic_icpr0 = 0U;

volatile uint8_t
    uart_driver_test_nvic_ipr[32];


/* =========================================================
 * Test Backend API
 * ========================================================= */

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


    /*
     * NVIC Priority Register 在真实 Cortex-M4
     * 中按 IRQ Number 索引。
     *
     * Host Backend 使用 32 Byte Array
     * 模拟当前测试所需的 NVIC Priority Space。
     */
    for (i = 0U; i < 32U; i++)
    {
        uart_driver_test_nvic_ipr[i] = 0U;
    }
}