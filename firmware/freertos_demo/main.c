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
 * 3. Motor Driver 初始化；
 * 4. 创建 ProtocolTX Task；
 * 5. 创建 ProtocolRX Task；
 * 6. 使用 Task Notification 实现 UART RX 事件驱动；
 * 7. 将通信数据交给 Motor Driver；
 * 8. 启动 FreeRTOS Scheduler。
 *
 * 核心机器人数据类型由 robot_types.h 统一定义。
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "uart_driver.h"
#include "motor_driver.h"
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
 *
 * Motor Driver Target
 * ->
 * Protocol
 * ->
 * Python
 * ========================================================= */

static void task_protocol_tx(
    void *parameters
)
{
    robot_joint_angles_t
        joint_targets;


    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    (void)parameters;


    for (;;)
    {
        /*
         * 上层不再自己保存 Joint Target。
         *
         * 所有 Target Position
         * 从 Motor Driver 获取。
         */
        if (
            motor_driver_get_target_positions(
                &joint_targets
            )
            == ROBOT_STATUS_OK
        )
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
        }


        /*
         * 当前测试阶段：
         *
         * 每 1000 ms
         * 向 PyBullet 发送一次目标位置。
         */
        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


/* =========================================================
 * ProtocolRX Task
 *
 * Python Feedback
 * ->
 * Protocol
 * ->
 * Motor Driver
 * ========================================================= */

static void task_protocol_rx(
    void *parameters
)
{
    protocol_parser_t parser;

    protocol_frame_t received_frame;


    robot_joint_angles_t
        received_joint_states;


    robot_joint_angles_t
        stored_joint_states;


    uint8_t ack_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    uint8_t diagnostics_frame[
        PROTOCOL_DIAGNOSTICS_FRAME_LEN
    ];


    /*
     * 用于计算连续 Feedback
     * 之间的采样间隔。
     */
    TickType_t
        previous_feedback_tick = 0U;


    uint8_t
        feedback_time_valid = 0U;


    (void)parameters;


    protocol_parser_init(
        &parser
    );


    /*
     * Scheduler 已运行，
     * Task 已创建，
     * Callback 已注册。
     *
     * 此时开启 UART RX IRQ。
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
         * 被 ISR 唤醒以后，
         * 将 Driver Ring Buffer
         * 中当前已有数据全部消费。
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
                        &received_joint_states
                    )
                    == ROBOT_STATUS_OK
                )
                {
                    TickType_t
                        current_feedback_tick;


                    robot_real_t
                        delta_time_s;


                    current_feedback_tick =
                        xTaskGetTickCount();


                    /*
                     * 第一帧没有上一时刻，
                     * 因此不能计算速度。
                     */
                    if (
                        feedback_time_valid
                    )
                    {
                        TickType_t
                            elapsed_ticks;


                        elapsed_ticks =
                            current_feedback_tick
                            -
                            previous_feedback_tick;


                        delta_time_s =
                            (robot_real_t)elapsed_ticks
                            /
                            (robot_real_t)configTICK_RATE_HZ;
                    }
                    else
                    {
                        delta_time_s =
                            0.0F;
                    }


                    /*
                     * 将收到的 Joint Feedback
                     * 交给 Motor Driver。
                     *
                     * Motor Driver 内部负责：
                     *
                     * 1. 保存位置；
                     * 2. 计算速度。
                     */
                    if (
                        motor_driver_update_feedback(
                            &received_joint_states,
                            delta_time_s
                        )
                        == ROBOT_STATUS_OK
                    )
                    {
                        previous_feedback_tick =
                            current_feedback_tick;


                        feedback_time_valid =
                            1U;


                        /*
                         * 从 Motor Driver
                         * 再读取当前保存的位置。
                         *
                         * ACK 因此使用的是
                         * Motor Driver 当前状态，
                         * 而不是直接绕过 Driver
                         * 使用 Protocol 临时变量。
                         */
                        if (
                            motor_driver_get_positions(
                                &stored_joint_states
                            )
                            == ROBOT_STATUS_OK
                        )
                        {
                            if (
                                protocol_build_joint_state_ack_frame(
                                    &stored_joint_states,
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
    /*
     * 初始测试目标：
     *
     * Joint 1 = 60.00°
     */
    const robot_joint_angles_t
        initial_joint_targets =
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


    board_clock_init();


    board_gpio_init();


    uart_driver_init();


    /*
     * 初始化统一 Motor Driver。
     */
    if (
        motor_driver_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /*
     * 设置系统初始 Joint Target。
     *
     * ProtocolTX Task 后续只从
     * Motor Driver 获取目标位置。
     */
    if (
        motor_driver_set_target_positions(
            &initial_joint_targets
        )
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


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