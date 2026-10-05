#ifndef UART_DRIVER_TEST_BACKEND_H
#define UART_DRIVER_TEST_BACKEND_H

#include <stdint.h>


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


extern volatile uint32_t
    uart_driver_test_nvic_iser0;

extern volatile uint32_t
    uart_driver_test_nvic_icer0;

extern volatile uint32_t
    uart_driver_test_nvic_icpr0;

extern volatile uint8_t
    uart_driver_test_nvic_ipr[32];


void uart_driver_test_backend_reset(void);


#endif