/*对外声明UART驱动能提供什么功能*/

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>


/**
 * @brief 初始化 UART0。
 */
void uart_driver_init(void);


/**
 * @brief 通过 UART0 发送一段数据。
 *
 * @param data 待发送数据缓冲区。
 * @param length 待发送数据长度，单位为字节。
 */
void uart_driver_write(
    const uint8_t *data,
    uint32_t length
);


/**
 * @brief 非阻塞读取一个 UART 字节。
 *
 * @param data 用于保存接收到的数据。
 *
 * @return 1 表示成功读取一个字节；
 *         0 表示当前没有可读数据。
 */
uint8_t uart_driver_read_byte(
    uint8_t *data
);


#endif