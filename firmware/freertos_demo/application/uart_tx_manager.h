/*
 * 文件：uart_tx_manager.h
 *
 * 用途：
 * 定义 UART 完整协议帧的任务级串行发送接口。
 *
 * UART Driver 本身保持与 FreeRTOS 解耦。
 *
 * 多个 FreeRTOS Task 如果需要发送完整协议帧，
 * 应通过本模块发送，
 * 避免多个 Task 同时调用 UART Driver
 * 导致协议帧字节交叉。
 */

#ifndef UART_TX_MANAGER_H
#define UART_TX_MANAGER_H

#include <stdint.h>

#include "error_code.h"


/**
 * @brief 初始化 UART TX Manager。
 *
 * 当前实现会创建一个 FreeRTOS Mutex，
 * 用于保护完整 UART Frame 的发送过程。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 初始化成功。
 *
 * ROBOT_STATUS_ERROR_INTERNAL：
 * Mutex 创建失败。
 */
robot_status_t uart_tx_manager_init(void);


/**
 * @brief 原子地发送一整个 UART 协议帧。
 *
 * 对其他 FreeRTOS Task 而言，
 * 一次本函数调用对应一个不可被其他发送者
 * 插入字节的完整发送区间。
 *
 * @param[in] data
 * 待发送完整协议帧。
 *
 * @param[in] length
 * 帧长度。
 *
 * 单位：
 * byte。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 发送完成。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * data 为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * length 为 0。
 *
 * ROBOT_STATUS_ERROR_NOT_READY：
 * TX Manager 尚未初始化。
 *
 * ROBOT_STATUS_ERROR_INTERNAL：
 * Mutex 操作失败。
 *
 * @note
 * 本接口只能在 Task Context 中调用，
 * 不能从 ISR 调用。
 */
robot_status_t uart_tx_manager_send_frame(
    const uint8_t *data,
    uint32_t length
);


#endif