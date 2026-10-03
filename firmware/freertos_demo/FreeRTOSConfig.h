/*
 * 文件：FreeRTOSConfig.h
 *
 * 用途：
 * 配置当前 Cortex-M4 / FreeRTOS 固件使用的
 * 调度、内存、Tick、中断优先级和任务通知参数。
 *
 * 当前目标平台：
 *
 * QEMU MPS2-AN386
 * ARM Cortex-M4
 *
 * 注意：
 * 当前 QEMU ARMv7-M NVIC 使用 8 个 Priority Bit。
 * 如果未来迁移到真实 MCU，
 * 必须根据目标芯片实际实现的 NVIC Priority Bit
 * 修改 configPRIO_BITS 及相关优先级配置。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "board.h"


/* =========================================================
 * CPU 与系统 Tick
 * ========================================================= */

/*
 * 当前 QEMU MPS2-AN386
 * 系统时钟为 25 MHz。
 */
#define configCPU_CLOCK_HZ \
    ((unsigned long)BOARD_SYSCLK_HZ)


/*
 * FreeRTOS Tick：
 *
 * 1000 Hz
 *
 * 即：
 *
 * 1 Tick = 1 ms
 */
#define configTICK_RATE_HZ \
    ((TickType_t)1000)


/* =========================================================
 * Scheduler
 * ========================================================= */

/*
 * 使用抢占式调度。
 */
#define configUSE_PREEMPTION 1


/*
 * 可使用的任务优先级数量：
 *
 * 0 ~ 4
 */
#define configMAX_PRIORITIES 5


/*
 * Idle Task 最小栈大小。
 */
#define configMINIMAL_STACK_SIZE 128


/*
 * Idle Task 可以主动让出 CPU。
 */
#define configIDLE_SHOULD_YIELD 1


/*
 * 使用 32 bit Tick。
 */
#define configUSE_16_BIT_TICKS 0


/* =========================================================
 * Memory
 * ========================================================= */

/*
 * FreeRTOS Heap：
 *
 * 32 KiB
 */
#define configTOTAL_HEAP_SIZE \
    (32U * 1024U)


#define configSUPPORT_DYNAMIC_ALLOCATION 1

#define configSUPPORT_STATIC_ALLOCATION 0


/* =========================================================
 * Task
 * ========================================================= */

#define configMAX_TASK_NAME_LEN 16


/*
 * 开启 Direct-to-Task Notification。
 *
 * 当前使用场景：
 *
 * UART RX ISR
 *      ↓
 * vTaskNotifyGiveFromISR()
 *      ↓
 * ProtocolRX Task
 */
#define configUSE_TASK_NOTIFICATIONS 1


/* =========================================================
 * Software Timer / Hook
 * ========================================================= */

#define configUSE_TIMERS 0

#define configUSE_IDLE_HOOK 0

#define configUSE_TICK_HOOK 0


/* =========================================================
 * FreeRTOS API
 * ========================================================= */

#define INCLUDE_vTaskDelay 1


/* =========================================================
 * Cortex-M NVIC Priority
 * ========================================================= */

/*
 * 当前 QEMU MPS2-AN386 的 ARMv7-M NVIC
 * 使用 8 个 Priority Bit。
 *
 * Cortex-M Priority 规则：
 *
 * 数值越小：
 * 逻辑优先级越高。
 *
 * 数值越大：
 * 逻辑优先级越低。
 *
 * 例如：
 *
 * 0x00
 * 高优先级
 *
 * 0x80
 * 中间优先级
 *
 * 0xFF
 * 最低优先级
 */
#define configPRIO_BITS 8U


/*
 * FreeRTOS Kernel 自身使用最低中断优先级。
 *
 * 8 Priority Bit 时：
 *
 * 最低优先级 = 255 = 0xFF
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY \
    255U


/*
 * 可以调用 FreeRTOS FromISR API 的
 * 最高逻辑中断优先级边界。
 *
 * 当前设为：
 *
 * 128 = 0x80
 *
 * 因此：
 *
 * 0x00 ~ 0x7F
 *
 * 属于比 FreeRTOS System Call Boundary
 * 更高的逻辑优先级，
 * 这些 ISR 不允许调用 FreeRTOS API。
 *
 * 0x80 ~ 0xFF
 *
 * 可以调用 FreeRTOS FromISR API。
 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY \
    128U


/*
 * FreeRTOS Cortex-M Port
 * 要求下面两个值使用 NVIC Hardware Format：
 *
 * 即 Priority 值必须已经移动到
 * 8 bit Priority Register 的高位。
 *
 * 当前：
 *
 * configPRIO_BITS = 8
 *
 * 所以：
 *
 * 8 - configPRIO_BITS = 0
 *
 * 不需要额外左移。
 */
#define configKERNEL_INTERRUPT_PRIORITY \
    ( \
        configLIBRARY_LOWEST_INTERRUPT_PRIORITY \
        << \
        (8U - configPRIO_BITS) \
    )


#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( \
        configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY \
        << \
        (8U - configPRIO_BITS) \
    )


/*
 * configMAX_SYSCALL_INTERRUPT_PRIORITY
 * 不能为 0。
 *
 * 0 是 Cortex-M 的最高逻辑优先级，
 * 不能作为 FreeRTOS System Call Boundary。
 */
#if (configMAX_SYSCALL_INTERRUPT_PRIORITY == 0U)

#error "configMAX_SYSCALL_INTERRUPT_PRIORITY must not be zero"

#endif


/*
 * System Call Boundary
 * 必须高于 Kernel Interrupt Priority。
 *
 * Cortex-M 数值越小，逻辑优先级越高。
 */
#if \
    ( \
        configMAX_SYSCALL_INTERRUPT_PRIORITY \
        >= \
        configKERNEL_INTERRUPT_PRIORITY \
    )

#error "Invalid FreeRTOS interrupt priority configuration"

#endif


/* =========================================================
 * Cortex-M Port Optimization
 * ========================================================= */

#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1


#endif