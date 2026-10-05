/*
 * 文件：uart_driver.h
 *
 * 用途：
 * 定义 MPS2-AN386 CMSDK APB UART0
 * 驱动公共接口。
 *
 * 当前提供：
 *
 * 1. UART RX / TX Interrupt；
 * 2. RX / TX Ring Buffer；
 * 3. RX Event Callback；
 * 4. RX Drop Counter；
 * 5. TX Busy State。
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>

#include "error_code.h"


/* =========================================================
 * RX Event Callback
 * ========================================================= */

/**
 * @brief UART RX 事件回调类型。
 *
 * @note
 * Callback 在 UART RX ISR Context 中执行。
 * Callback 应保持短小，并且不能执行阻塞操作。
 */
typedef void (*uart_driver_rx_event_callback_t)(void);


/* =========================================================
 * Driver Initialization
 * ========================================================= */

/**
 * @brief 初始化 UART0 Driver。
 *
 * 初始化 UART0、RX / TX Ring Buffer、
 * Interrupt 状态及内部 Driver State。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 初始化成功。
 */
robot_status_t uart_driver_init(void);


/**
 * @brief 开启 UART0 RX Interrupt。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 开启成功。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * UART Driver 尚未初始化。
 *
 * @note
 * 应在上层 RX Task 和 RX Event Callback
 * 准备完成后调用。
 */
robot_status_t uart_driver_enable_rx_interrupt(void);


/**
 * @brief 注册 UART RX ISR 事件回调。
 *
 * @param[in] callback
 * RX Event Callback。
 *
 * 允许为 NULL。
 */
void uart_driver_set_rx_event_callback(
    uart_driver_rx_event_callback_t callback
);


/* =========================================================
 * TX API
 * ========================================================= */

/**
 * @brief 将一段数据加入 UART TX Ring Buffer。
 *
 * @param[in] data
 * 待发送数据。
 *
 * @param[in] length
 * 数据长度。
 *
 * 单位：
 * Byte。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 数据已加入发送队列。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * UART Driver 尚未初始化。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * data 为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * length 为 0。
 *
 * ROBOT_STATUS_ERROR_BUFFER_FULL：
 * TX Ring Buffer 空间不足。
 *
 * ROBOT_STATUS_ERROR_INTERNAL：
 * Driver 内部状态异常。
 *
 * @note
 * 一次调用的数据必须整体进入 TX Ring Buffer，
 * 不允许发生部分写入。
 *
 * 实际 UART 发送由 TX Interrupt 完成。
 */
robot_status_t uart_driver_write(
    const uint8_t *data,
    uint32_t length
);


/**
 * @brief 查询 UART TX 是否仍在发送。
 *
 * @return
 * 1：
 * TX 正在进行。
 *
 * 0：
 * TX 当前空闲。
 */
uint8_t uart_driver_is_tx_busy(void);


/* =========================================================
 * RX API
 * ========================================================= */

/**
 * @brief 从 UART RX Ring Buffer 读取一个字节。
 *
 * @param[out] data
 * 输出读取到的 Byte。
 *
 * @return
 * 1：
 * 读取成功。
 *
 * 0：
 * 当前无数据、Driver 未初始化或 data 为空。
 */
uint8_t uart_driver_read_byte(
    uint8_t *data
);


/**
 * @brief 获取 RX Ring Buffer 溢出丢失字节数。
 *
 * @return
 * 自最近一次 uart_driver_init()
 * 以来累计的 RX Drop Count。
 */
uint32_t uart_driver_get_rx_drop_count(void);


/* =========================================================
 * Interrupt Handlers
 * ========================================================= */

/**
 * @brief UART0 RX Interrupt Handler。
 */
void UART0_RX_IRQHandler(void);


/**
 * @brief UART0 TX Interrupt Handler。
 */
void UART0_TX_IRQHandler(void);


#endif