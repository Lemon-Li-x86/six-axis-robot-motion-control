#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "uart_driver.h"
#include "uart_tx_manager.h"


static SemaphoreHandle_t uart_tx_mutex = NULL;


robot_status_t uart_tx_manager_init(void)
{
    if (uart_tx_mutex != NULL)
    {
        return ROBOT_STATUS_OK;
    }

    uart_tx_mutex =
        xSemaphoreCreateMutex();

    if (uart_tx_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return ROBOT_STATUS_OK;
}


robot_status_t uart_tx_manager_send_frame(
    const uint8_t *data,
    uint32_t length
)
{
    BaseType_t semaphore_result;
    robot_status_t tx_status;

    if (data == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    if (length == 0U)
    {
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }

    if (uart_tx_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }

    /*
     * 保证多个 Task 加入 TX Queue 时，
     * 每一个协议帧的全部字节连续进入 Ring Buffer。
     */
    semaphore_result = xSemaphoreTake(
        uart_tx_mutex,
        portMAX_DELAY
    );

    if (semaphore_result != pdTRUE)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    tx_status = uart_driver_write(
        data,
        length
    );

    semaphore_result =
        xSemaphoreGive(
            uart_tx_mutex
        );

    if (semaphore_result != pdTRUE)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    return tx_status;
}