/*
 * 文件：uart_driver.h
 *
 * 用途：
 * 定义 UART0 驱动对外提供的接口。
 *
 * UART RX 数据由中断写入驱动内部 Ring Buffer。
 *
 * 上层可以注册 RX Event Callback，
 * 当 ISR 收到 UART 数据时获得通知。
 *
 * UART Driver 本身不依赖 FreeRTOS。
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>


/* =========================================================
 * UART RX Event Callback
 * ========================================================= */

/*
 * RX Event Callback 在 UART ISR 上下文中执行。
 *
 * 因此回调函数必须：
 *
 * 1. 尽量短；
 * 2. 不能阻塞；
 * 3. 如果调用 RTOS API，
 *    必须使用 FromISR 版本。
 */
typedef void (*uart_driver_rx_event_callback_t)(
    void
);


/* =========================================================
 * UART Driver API
 * ========================================================= */

/**
 * @brief 初始化 UART0。
 *
 * 初始化 UART 和内部 Ring Buffer，
 * 但暂时不打开 RX Interrupt。
 */
void uart_driver_init(void);


/**
 * @brief 开启 UART0 RX Interrupt。
 *
 * 应在系统已经准备好处理中断事件以后调用。
 */
void uart_driver_enable_rx_interrupt(void);


/**
 * @brief 注册 UART RX Event Callback。
 *
 * @param callback
 * 在 UART RX ISR 中调用的回调函数。
 */
void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
);


/**
 * @brief 通过 UART0 发送一段数据。
 *
 * @param data
 * 待发送数据缓冲区。
 *
 * @param length
 * 数据长度，单位为字节。
 */
void uart_driver_write(
    const uint8_t *data,
    uint32_t length
);


/**
 * @brief 从 UART 软件接收缓冲区读取一个字节。
 *
 * 本函数不直接轮询 UART 硬件。
 *
 * UART RX ISR 已经负责将收到的数据
 * 放入内部 Ring Buffer。
 *
 * @param data
 * 用于保存读取到的数据。
 *
 * @return
 * 1：成功读取一个字节。
 * 0：当前 Ring Buffer 为空。
 */
uint8_t uart_driver_read_byte(
    uint8_t *data
);


/**
 * @brief 获取 Ring Buffer 满导致的 RX 丢字节数量。
 */
uint32_t uart_driver_get_rx_drop_count(void);


#endif