
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
#include "motor_state.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"

#include "performance_monitor.h"
#if ROBOT_ENABLE_STAGE4_TASKS && ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT
#include "motion_pipeline.h"
#endif


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
        robot_status_t status;
#if ROBOT_ENABLE_STAGE4_TASKS && ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT
        motion_stage4_diagnostics_t motion;
        status = motion_pipeline_get_diagnostics(&motion);
        if (status == ROBOT_STATUS_OK && motion.controller_steps == 0U)
        {
            /* Before the first validated Control output, hold the measured
             * pose. JSON startup targets belong to the legacy demo, not an
             * accepted motion request. No feedback means no target frame. */
            motor_state_observation_t feedback;
            status = motor_state_get_feedback(&feedback);
            if (status == ROBOT_STATUS_OK)
                joint_targets = feedback.feedback.canonical_positions;
        }
        else if (status == ROBOT_STATUS_OK)
        {
            /* Read AFTER the publication count: otherwise Control could
             * publish between a stale target read and this diagnostic read. */
            status = motor_state_get_targets(&joint_targets);
        }
#else
        status = motor_state_get_targets(&joint_targets);
#endif
        if (status == ROBOT_STATUS_OK &&
            protocol_build_joint_target_frame(&joint_targets, frame)
                == ROBOT_STATUS_OK)
        {
            (void)uart_tx_manager_send_frame(frame, PROTOCOL_JOINT_FRAME_LEN);
        }

        /* Runtime configuration owns the TX period; never busy-loop if
         * millisecond conversion rounds below one scheduler tick. */
        {
            TickType_t delay_ticks = pdMS_TO_TICKS(
                runtime_config_get_protocol_tx_period_ms());
            vTaskDelay(delay_ticks == 0U ? 1U : delay_ticks);
        }
    }
}


/* =========================================================
 * Protocol RX Task
 * ========================================================= */

static void task_protocol_rx(void *parameters)
{
    protocol_parser_t parser;
    protocol_frame_t received_frame;
    TickType_t last_byte_tick = 0U;
    TickType_t frame_timeout_ticks = pdMS_TO_TICKS(
        PROTOCOL_FRAME_TIMEOUT_MS);

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
    if (frame_timeout_ticks == 0U)
    {
        frame_timeout_ticks = 1U;
    }

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
        uint32_t notifications = ulTaskNotifyTake(
            pdTRUE,
            parser.state == PROTOCOL_STATE_WAIT_HEADER_0
                ? portMAX_DELAY
                : frame_timeout_ticks
        );

        performance_monitor_record_task_wakeup();

        /* A partial frame also expires when no subsequent byte arrives. */
        if (notifications == 0U)
        {
            TickType_t now = xTaskGetTickCount();
            (void)protocol_parser_expire_partial_frame(
                &parser,
                (uint32_t)(TickType_t)(now - last_byte_tick),
                (uint32_t)frame_timeout_ticks);
        }

        /*
         * Consume all bytes currently stored
         * in the UART receive ring buffer.
         */
        while (uart_driver_read_byte(&byte))
        {
            TickType_t byte_tick = xTaskGetTickCount();

            /*
             * A truncated frame must not consume a later command. UART
             * notifications wake this task as bytes arrive; comparing the
             * last consumed byte tick here also handles tick counter wrap.
             */
            (void)protocol_parser_expire_partial_frame(
                &parser,
                (uint32_t)(TickType_t)(byte_tick - last_byte_tick),
                (uint32_t)frame_timeout_ticks);

            last_byte_tick = byte_tick;
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
    TaskHandle_t protocol_tx_task_handle = NULL;

    /*
     * Create periodic target-transmission task.
     */
    result = xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        TASK_STACK_DEPTH_PROTOCOL_TX,
        NULL,
        TASK_PRIORITY_PROTOCOL_TX,
        &protocol_tx_task_handle
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
        /*
         * Both tasks are created before vTaskStartScheduler().
         * Roll back the successfully created TX task if RX creation fails.
         */
        vTaskDelete(protocol_tx_task_handle);
        protocol_rx_task_handle = NULL;
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
