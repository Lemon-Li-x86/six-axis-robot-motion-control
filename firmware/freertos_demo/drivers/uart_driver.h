#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>

#include "error_code.h"


typedef void (*uart_driver_rx_event_callback_t)(void);


/* 初始化 UART0、RX/TX Ring Buffer 和中断状态。 */
robot_status_t uart_driver_init(void);


/* 在上层 RX Task 和 Callback 准备完成后开启 RX 中断。 */
robot_status_t uart_driver_enable_rx_interrupt(void);


/* 注册 RX ISR 事件回调，允许 callback == NULL。 */
void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
);


/*
 * 将一段数据加入 TX Ring Buffer。
 *
 * 实际发送由 UART0 TX IRQ 完成。
 */
robot_status_t uart_driver_write(
    const uint8_t *data,
    uint32_t length
);


/*
 * 从 RX Ring Buffer 读取一个字节。
 *
 * 返回：
 * 1 = 成功
 * 0 = 无数据或 data == NULL
 */
uint8_t uart_driver_read_byte(
    uint8_t *data
);


/* RX Ring Buffer 溢出丢失字节数。 */
uint32_t uart_driver_get_rx_drop_count(void);


/* UART TX 是否仍有数据等待发送。 */
uint8_t uart_driver_is_tx_busy(void);


/* Interrupt Handlers */
void UART0_RX_IRQHandler(void);
void UART0_TX_IRQHandler(void);


#endif