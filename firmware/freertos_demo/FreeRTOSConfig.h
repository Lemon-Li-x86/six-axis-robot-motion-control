#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Cortex-M4 / QEMU MPS2-AN386 当前按 25 MHz 配置。
 * FreeRTOS 官方 MPS2 QEMU demo 也使用该频率。
 */
#define configCPU_CLOCK_HZ                 ( ( unsigned long ) 25000000 )

/* 系统每秒产生 1000 个 RTOS Tick，也就是 1 ms 一个 Tick。 */
#define configTICK_RATE_HZ                 ( ( TickType_t ) 1000 )

/* 使用抢占式调度。 */
#define configUSE_PREEMPTION               1

/* 任务最大优先级数量。 */
#define configMAX_PRIORITIES               5

/* Idle Task 的最小栈大小。 */
#define configMINIMAL_STACK_SIZE           128

/* FreeRTOS 动态内存池大小。 */
#define configTOTAL_HEAP_SIZE              ( 32 * 1024 )

/* 任务名称最大长度。 */
#define configMAX_TASK_NAME_LEN            16

/* 当前使用 32 位 Tick。 */
#define configUSE_16_BIT_TICKS             0

/* Idle Task 可以让出 CPU。 */
#define configIDLE_SHOULD_YIELD            1

/* 当前 demo 暂时不用软件 Timer。 */
#define configUSE_TIMERS                   0

/* 当前 demo 不需要 idle hook / tick hook。 */
#define configUSE_IDLE_HOOK                0
#define configUSE_TICK_HOOK                0

/* 支持动态创建任务。 */
#define configSUPPORT_DYNAMIC_ALLOCATION   1
#define configSUPPORT_STATIC_ALLOCATION    0

/* 我们后面任务中要用 vTaskDelay()。 */
#define INCLUDE_vTaskDelay                 1

/* Cortex-M 中断优先级配置。 */
#define configKERNEL_INTERRUPT_PRIORITY        255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  4

/* 使用 Cortex-M 优化的任务选择实现。 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#endif