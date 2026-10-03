/*
 * 文件：FreeRTOSConfig.h
 *
 * 用途：
 * 配置当前 Cortex-M4 / FreeRTOS 固件所使用的
 * 调度、内存、Tick、中断和任务通知参数。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "board.h"


/* =========================================================
 * CPU 与系统 Tick
 * ========================================================= */

#define configCPU_CLOCK_HZ \
    ((unsigned long)BOARD_SYSCLK_HZ)

/*
 * 系统每秒产生 1000 个 RTOS Tick，
 * 即 1 ms 一个 Tick。
 */
#define configTICK_RATE_HZ \
    ((TickType_t)1000)


/* =========================================================
 * 调度配置
 * ========================================================= */

/* 使用抢占式调度。 */
#define configUSE_PREEMPTION 1

/* 最大任务优先级数量。 */
#define configMAX_PRIORITIES 5

/* Idle Task 最小栈大小。 */
#define configMINIMAL_STACK_SIZE 128

/* Idle Task 可以主动让出 CPU。 */
#define configIDLE_SHOULD_YIELD 1

/* 当前使用 32 位 Tick。 */
#define configUSE_16_BIT_TICKS 0


/* =========================================================
 * 内存配置
 * ========================================================= */

#define configTOTAL_HEAP_SIZE \
    (32 * 1024)

#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 0


/* =========================================================
 * Task 配置
 * ========================================================= */

#define configMAX_TASK_NAME_LEN 16

/*
 * 开启 Direct-to-Task Notification。
 *
 * UART RX ISR 将通过 Task Notification
 * 唤醒 ProtocolRX Task。
 */
#define configUSE_TASK_NOTIFICATIONS 1


/* =========================================================
 * Timer / Hook
 * ========================================================= */

#define configUSE_TIMERS 0

#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0


/* =========================================================
 * FreeRTOS API
 * ========================================================= */

#define INCLUDE_vTaskDelay 1


/* =========================================================
 * Cortex-M 中断优先级
 * ========================================================= */

#define configKERNEL_INTERRUPT_PRIORITY \
    255

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    4


/* =========================================================
 * Cortex-M 优化
 * ========================================================= */

#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1


#endif