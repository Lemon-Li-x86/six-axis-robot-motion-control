/**
 * @file protocol_tasks.h
 * @brief 协议任务启动接口。
 */

#ifndef PROTOCOL_TASKS_H
#define PROTOCOL_TASKS_H

#include "error_code.h"

/**
 * @brief 创建 ProtocolTX / ProtocolRX Task，
 *        并注册 UART RX ISR Callback。
 *
 * @return
 * ROBOT_STATUS_OK
 * ROBOT_STATUS_ERROR_INTERNAL
 */
robot_status_t protocol_tasks_start(void);

#endif
