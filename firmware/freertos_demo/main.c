/*
 * 文件：main.c
 *
 * 用途：
 * 六轴机器人 Cortex-M4 / FreeRTOS 固件入口。
 *
 * 当前负责：
 *
 * 1. Board 初始化；
 * 2. UART Driver 初始化；
 * 3. 创建 ProtocolTX Task；
 * 4. 创建 ProtocolRX Task；
 * 5. 使用 Task Notification 实现 UART RX 事件驱动；
 * 6. 处理应用层协议命令；
 * 7. 启动 FreeRTOS Scheduler。
 *
 * 核心机器人数据类型由 robot_types.h 统一定义。
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "uart_driver.h"
#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * ProtocolRX Task Handle
 * ========================================================= */

static TaskHandle_t
    protocol_rx_task_handle = NULL;


/* =========================================================
 * UART RX ISR Callback
 * ========================================================= */

static void uart_rx_event_from_isr(void)
{
    BaseType_t
        higher_priority_task_woken =
            pdFALSE;


    if (
        protocol_rx_task_handle
        != NULL
    )
    {
        vTaskNotifyGiveFromISR(
            protocol_rx_task_handle,
            &higher_priority_task_woken
        );


        portYIELD_FROM_ISR(
            higher_priority_task_woken
        );
    }
}


/* =========================================================
 * ProtocolTX Task
 * ========================================================= */

static void task_protocol_tx(
    void *parameters
)
{
    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    /*
     * 当前固定测试目标：
     *
     * Joint 1:
     *
     * 6000 × 0.01°
     * =
     * 60.00°
     */
    const robot_joint_angles_t
        joint_targets =
        {
            .value =
            {
                6000,
                0,
                0,
                0,
                0,
                0
            }
        };


    (void)parameters;


    for (;;)
    {
        if (
            protocol_build_joint_target_frame(
                &joint_targets,
                frame
            )
            == ROBOT_STATUS_OK
        )
        {
            uart_driver_write(
                frame,
                PROTOCOL_JOINT_FRAME_LEN
            );
        }


        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


/* =========================================================
 * ProtocolRX Task
 * ========================================================= */

static void task_protocol_rx(
    void *parameters
)
{
    protocol_parser_t parser;

    protocol_frame_t received_frame;


    robot_joint_angles_t
        joint_states;


    uint8_t ack_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    uint8_t diagnostics_frame[
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    ];


    (void)parameters;


    protocol_parser_init(
        &parser
    );


    /*
     * Scheduler 已运行，
     * Task 已创建，
     * Callback 已注册。
     *
     * 现在开启 UART RX IRQ。
     */
    uart_driver_enable_rx_interrupt();


    for (;;)
    {
        uint8_t byte;


        /*
         * 没有 UART RX Event 时，
         * 当前 Task 完全阻塞。
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


        /*
         * 一旦被 ISR 唤醒，
         * 将 Driver Ring Buffer
         * 当前已有数据全部消费。
         */
        while (
            uart_driver_read_byte(
                &byte
            )
        )
        {
            if (
                protocol_parser_process_byte(
                    &parser,
                    byte,
                    &received_frame
                )
            )
            {
                /* =========================================
                 * JOINT_STATE
                 * ========================================= */

                if (
                    protocol_parse_joint_state(
                        &received_frame,
                        &joint_states
                    )
                    == ROBOT_STATUS_OK
                )
                {
                    if (
                        protocol_build_joint_state_ack_frame(
                            &joint_states,
                            ack_frame
                        )
                        == ROBOT_STATUS_OK
                    )
                    {
                        uart_driver_write(
                            ack_frame,
                            PROTOCOL_JOINT_FRAME_LEN
                        );
                    }
                }

                /* =========================================
                 * GET_DIAGNOSTICS
                 * ========================================= */

                else if (
                    protocol_is_diagnostics_request(
                        &received_frame
                    )
                )
                {
                    uint32_t
                        rx_drop_count;


                    rx_drop_count =
                        uart_driver_get_rx_drop_count();


                    if (
                        protocol_build_diagnostics_response_frame(
                            rx_drop_count,
                            diagnostics_frame
                        )
                        == ROBOT_STATUS_OK
                    )
                    {
                        uart_driver_write(
                            diagnostics_frame,
                            PROTOCOL_DIAGNOSTICS_FRAME_LEN
                        );
                    }
                }
            }
        }
    }
}


/* =========================================================
 * Firmware Entry
 * ========================================================= */

int main(void)
{
    board_clock_init();


    board_gpio_init();


    uart_driver_init();


    xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        &protocol_rx_task_handle
    );


    uart_driver_set_rx_event_callback(
        uart_rx_event_from_isr
    );


    vTaskStartScheduler();


    while (1)
    {
    }
}