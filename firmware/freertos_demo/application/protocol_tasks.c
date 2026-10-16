
/*
 * File: protocol_tasks.c
 *
 * FreeRTOS protocol task management.
 *
 * Responsibilities:
 * 1. Create ProtocolRX and ProtocolTX tasks.
 * 2. Connect UART RX interrupts to task notifications.
 * 3. Receive and parse UART byte streams.
 * 4. Dispatch complete frames to application services.
 * 5. Periodically transmit joint targets.
 *
 * Application command handling is implemented in:
 * application/services/protocol_commands.c
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "protocol_tasks.h"
#include "task_config.h"
#include "runtime_config.h"
#include "uart_tx_manager.h"

#include "protocol_commands.h"

#include "uart_driver.h"
#include "motor_driver.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"

#include "performance_monitor.h"


/* =========================================================
 * Task State
 * ========================================================= */

static TaskHandle_t protocol_rx_task_handle = NULL;


/* =========================================================
 * UART RX ISR Callback
 * ========================================================= */

static void uart_rx_event_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    /*
     * Preserve RX interrupt timing instrumentation.
     */
    performance_monitor_mark_rx_isr();

    if (protocol_rx_task_handle == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(
        protocol_rx_task_handle,
        &higher_priority_task_woken
    );

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


/* =========================================================
 * Protocol TX Task
 * ========================================================= */

static void task_protocol_tx(void *parameters)
{
    robot_joint_angles_t joint_targets;

    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN];

    (void)parameters;

    for (;;)
    {
        /*
         * Read the current canonical joint targets.
         */
        if (
            motor_driver_get_target_positions(
                &joint_targets
            ) == ROBOT_STATUS_OK
        )
        {
            if (
                protocol_build_joint_target_frame(
                    &joint_targets,
                    frame
                ) == ROBOT_STATUS_OK
            )
            {
                (void)uart_tx_manager_send_frame(
                    frame,
                    PROTOCOL_JOINT_FRAME_LEN
                );
            }
        }

        /*
         * TX period can be changed at runtime
         * through the SET_PARAMETER command.
         */
        vTaskDelay(
            pdMS_TO_TICKS(
                runtime_config_get_protocol_tx_period_ms()
            )
        );
    }
}


/* =========================================================
 * Protocol RX Task
 * ========================================================= */

static void task_protocol_rx(void *parameters)
{
    protocol_parser_t parser;
    protocol_frame_t received_frame;

    /*
     * Persistent feedback timing state.
     *
     * The task owns its storage.
     * The application command handler updates it
     * after successfully accepting joint feedback.
     */
    protocol_command_context_t command_context;

    (void)parameters;

    protocol_parser_init(&parser);

    protocol_commands_init(&command_context);

    /*
     * Enable RX interrupts only after the
     * receiving task starts running.
     */
    uart_driver_enable_rx_interrupt();

    for (;;)
    {
        uint8_t byte;

        performance_monitor_arm_task_wakeup();

        /*
         * Sleep until the UART RX interrupt
         * notifies this task.
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        performance_monitor_record_task_wakeup();

        /*
         * Consume all bytes currently stored
         * in the UART receive ring buffer.
         */
        while (uart_driver_read_byte(&byte))
        {
            if (
                protocol_parser_process_byte(
                    &parser,
                    byte,
                    &received_frame
                )
            )
            {
                /*
                 * Send a complete frame to the
                 * application command dispatcher.
                 *
                 * The dispatcher receives:
                 * - parsed frame
                 * - persistent feedback state
                 * - current system tick
                 * - ticks per second
                 *
                 * It does not call FreeRTOS APIs.
                 */
                protocol_commands_dispatch_frame(
                    &received_frame,
                    &command_context,
                    (uint32_t)xTaskGetTickCount(),
                    (uint32_t)configTICK_RATE_HZ
                );
            }
        }
    }
}


/* =========================================================
 * Public API
 * ========================================================= */

robot_status_t protocol_tasks_start(void)
{
    BaseType_t result;

    /*
     * Create periodic target-transmission task.
     */
    result = xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        TASK_STACK_DEPTH_PROTOCOL_TX,
        NULL,
        TASK_PRIORITY_PROTOCOL_TX,
        NULL
    );

    if (result != pdPASS)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * Create event-driven receiving task.
     */
    result = xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        TASK_STACK_DEPTH_PROTOCOL_RX,
        NULL,
        TASK_PRIORITY_PROTOCOL_RX,
        &protocol_rx_task_handle
    );

    if (result != pdPASS)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    /*
     * Register the UART RX ISR callback.
     *
     * Actual RX interrupts are enabled only
     * after ProtocolRX starts executing.
     */
    uart_driver_set_rx_event_callback(
        uart_rx_event_from_isr
    );

    return ROBOT_STATUS_OK;
}
