/*
 * 文件：protocol_tasks.h
 *
 * 用途：
 * 定义 Protocol Application Tasks 启动接口。
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