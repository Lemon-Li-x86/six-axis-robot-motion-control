/*
 * 文件：uart_driver.h
 *
 * 用途：
 * 定义 UART0 驱动对外接口。
 *
 * UART RX 数据由中断写入驱动内部 Ring Buffer。
 *
 * 上层可以注册 RX Event Callback，
 * 当 ISR 收到 UART 数据时获得事件通知。
 *
 * UART Driver 本身不依赖 FreeRTOS。
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>


/*
 * RX Event Callback 在 UART ISR 上下文执行。
 *
 * Callback 必须：
 *
 * 1. 尽量短；
 * 2. 不得阻塞；
 * 3. 调用 RTOS API 时必须使用 FromISR 版本。
 */
typedef void (*uart_driver_rx_event_callback_t)(
    void
);


/**
 * @brief 初始化 UART0 和内部 RX Ring Buffer。
 *
 * 初始化完成后 UART TX / RX 功能可用，
 * 但 RX Interrupt 暂时保持关闭。
 */
void uart_driver_init(void);


/**
 * @brief 开启 UART0 RX Interrupt。
 *
 * 应在上层 Task 和 RX Callback
 * 已经准备完成之后调用。
 */
void uart_driver_enable_rx_interrupt(void);


/**
 * @brief 注册 UART RX Event Callback。
 *
 * @param[in] callback
 * UART RX ISR 中调用的事件回调。
 *
 * 允许传入空回调，
 * 表示当前不向上层发送 RX Event。
 */
void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
);


/**
 * @brief 通过 UART0 发送一段数据。
 *
 * 当前发送实现为阻塞式发送。
 *
 * @param[in] data
 * 待发送数据缓冲区。
 *
 * @param[in] length
 * 待发送数据长度。
 *
 * 单位：
 * byte。
 */
void uart_driver_write(
    const uint8_t *data,
    uint32_t length
);


/**
 * @brief 从 UART 软件 RX Ring Buffer 读取一个字节。
 *
 * 本函数不直接轮询 UART Hardware。
 *
 * UART RX ISR 已经负责将接收到的数据
 * 写入内部 Ring Buffer。
 *
 * @param[out] data
 * 输出读取到的一个字节。
 *
 * @return
 * 1：
 * 成功读取一个字节。
 *
 * 0：
 * 当前 Ring Buffer 为空。
 */
uint8_t uart_driver_read_byte(
    uint8_t *data
);


/**
 * @brief 获取因 RX Ring Buffer 满而丢失的字节数量。
 *
 * @return
 * 从最近一次 uart_driver_init() 开始累计的
 * RX Drop Byte Count。
 */
uint32_t uart_driver_get_rx_drop_count(void);


#endif