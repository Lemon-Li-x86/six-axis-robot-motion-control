/**
 * @file uart_driver_test_backend.h
 * @brief 测试共享声明/夹具：uart driver test backend。
 */

#ifndef STAGE2_UART_DRIVER_TEST_BACKEND_H
#define STAGE2_UART_DRIVER_TEST_BACKEND_H

#include <stdint.h>

/* UART0 Test Registers */

extern volatile uint32_t
    uart_driver_test_uart0_data;

extern volatile uint32_t
    uart_driver_test_uart0_state;

extern volatile uint32_t
    uart_driver_test_uart0_ctrl;

extern volatile uint32_t
    uart_driver_test_uart0_intstatus;

extern volatile uint32_t
    uart_driver_test_uart0_bauddiv;

/* NVIC Test Registers */

extern volatile uint32_t
    uart_driver_test_nvic_iser0;

extern volatile uint32_t
    uart_driver_test_nvic_icer0;

extern volatile uint32_t
    uart_driver_test_nvic_icpr0;

extern volatile uint8_t
    uart_driver_test_nvic_ipr[32];

/* Test Backend API */

/**
 * @brief 重置 UART / NVIC Host Test Backend。
 *
 * 将所有模拟寄存器恢复为初始状态，
 * 用于保证不同 Unit Test 之间状态隔离。
 */
void uart_driver_test_backend_reset(void);

#endif
