/*
 * 文件：uart_tx_manager.c
 *
 * 用途：
 * 实现多个 FreeRTOS Task 之间的 UART TX 串行化。
 *
 * 当前设计：
 *
 * Application Task
 * ->
 * UART TX Manager
 * ->
 * Mutex
 * ->
 * UART Driver
 *
 * UART Driver 仍然不依赖 FreeRTOS。
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "uart_driver.h"
#include "uart_tx_manager.h"


/* =========================================================
 * UART TX Mutex
 * ========================================================= */

/*
 * 保护一次完整 UART Frame 的发送过程。
 *
 * FreeRTOS Mutex 支持 Priority Inheritance，
 * 可以降低不同优先级任务之间的优先级反转风险。
 */
static SemaphoreHandle_t
    uart_tx_mutex = NULL;


/* =========================================================
 * Initialization
 * ========================================================= */

robot_status_t uart_tx_manager_init(void)
{
    /*
     * 允许重复调用初始化接口。
     */
    if (
        uart_tx_mutex
        != NULL
    )
    {
        return
            ROBOT_STATUS_OK;
    }


    uart_tx_mutex =
        xSemaphoreCreateMutex();


    if (
        uart_tx_mutex
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Frame Transmission
 * ========================================================= */

robot_status_t uart_tx_manager_send_frame(
    const uint8_t *data,
    uint32_t length
)
{
    BaseType_t
        semaphore_result;


    if (
        data == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        length == 0U
    )
    {
        return
            ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }


    if (
        uart_tx_mutex
        == NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    /*
     * 获取 UART TX Mutex。
     *
     * 如果另一个 Task 正在发送完整协议帧，
     * 当前 Task 在这里等待，
     * 不会从帧中间插入自己的数据。
     */
    semaphore_result =
        xSemaphoreTake(
            uart_tx_mutex,
            portMAX_DELAY
        );


    if (
        semaphore_result
        != pdTRUE
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    /*
     * Mutex 保护期间，
     * 当前 Task 独占 UART TX 数据流。
     */
    uart_driver_write(
        data,
        length
    );


    semaphore_result =
        xSemaphoreGive(
            uart_tx_mutex
        );


    if (
        semaphore_result
        != pdTRUE
    )
    {
        return
            ROBOT_STATUS_ERROR_INTERNAL;
    }


    return
        ROBOT_STATUS_OK;
}