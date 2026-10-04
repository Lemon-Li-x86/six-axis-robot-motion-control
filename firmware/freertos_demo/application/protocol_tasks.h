/*
 * 文件：protocol_tasks.h
 *
 * 用途：
 * 定义机器人固件 Protocol Application Tasks
 * 的启动接口。
 *
 * 本模块负责：
 *
 * 1. ProtocolTX Task；
 * 2. ProtocolRX Task；
 * 3. UART RX ISR -> Task Notification；
 * 4. 已解析协议帧的 Application Dispatch；
 * 5. Joint State Feedback 处理；
 * 6. Diagnostics 请求处理。
 *
 * main.c 只负责系统级初始化和启动，
 * 不直接实现 Protocol Task 业务逻辑。
 */

#ifndef PROTOCOL_TASKS_H
#define PROTOCOL_TASKS_H

#include "error_code.h"


/**
 * @brief 创建并配置 Protocol Application Tasks。
 *
 * 当前创建：
 *
 * 1. ProtocolTX Task；
 * 2. ProtocolRX Task。
 *
 * 同时注册 UART RX Event Callback。
 *
 * UART RX Interrupt 不在本函数中立即开启，
 * 而是在 ProtocolRX Task 真正运行后开启。
 *
 * 这样可以确保：
 *
 * 1. Scheduler 已运行；
 * 2. ProtocolRX Task 已存在；
 * 3. UART RX Callback 已注册；
 *
 * 之后才允许 RX Interrupt 进入系统。
 *
 * @return
 * ROBOT_STATUS_OK：
 * Protocol Tasks 创建成功。
 *
 * ROBOT_STATUS_ERROR_INTERNAL：
 * 至少一个 FreeRTOS Task 创建失败。
 */
robot_status_t protocol_tasks_start(void);


#endif