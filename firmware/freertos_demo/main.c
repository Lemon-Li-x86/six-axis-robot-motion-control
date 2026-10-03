/*
 * 文件：main.c
 *
 * 用途：
 * 六轴机器人 Cortex-M4 / FreeRTOS 固件入口。
 *
 * 当前主要负责：
 *
 * 1. 初始化板级资源；
 * 2. 初始化 UART Driver；
 * 3. 创建 ProtocolTX Task；
 * 4. 创建 ProtocolRX Task；
 * 5. 使用 Task Notification
 *    实现 UART RX 事件驱动；
 * 6. 启动 FreeRTOS Scheduler。
 *
 * UART 硬件访问由 uart_driver 模块负责。
 * 协议解析由 protocol 模块负责。
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "uart_driver.h"
#include "protocol.h"


/* =========================================================
 * ProtocolRX Task Handle
 * ========================================================= */

/*
 * 保存 ProtocolRX Task 的句柄。
 *
 * UART RX ISR 产生事件后，
 * main 层的 ISR Callback
 * 使用该句柄通知 ProtocolRX Task。
 */
static TaskHandle_t
    protocol_rx_task_handle = NULL;


/* =========================================================
 * UART RX ISR Callback
 * ========================================================= */

/*
 * 本函数由 UART Driver
 * 在 UART RX ISR 中调用。
 *
 * 注意：
 *
 * 当前执行环境仍然属于 ISR，
 * 因此必须使用 FreeRTOS FromISR API。
 */
static void uart_rx_event_from_isr(void)
{
    BaseType_t
        higher_priority_task_woken =
            pdFALSE;


    /*
     * 确认 ProtocolRX Task
     * 已经成功创建。
     */
    if (
        protocol_rx_task_handle
        != NULL
    )
    {
        /*
         * 增加 ProtocolRX Task
         * 的 Notification Count。
         */
        vTaskNotifyGiveFromISR(
            protocol_rx_task_handle,
            &higher_priority_task_woken
        );


        /*
         * 如果被唤醒的任务需要立即运行，
         * 请求 ISR 退出后进行任务切换。
         */
        portYIELD_FROM_ISR(
            higher_priority_task_woken
        );
    }
}


/* =========================================================
 * Task 1：
 * Cortex-M4 -> Python
 *
 * 周期发送六轴目标角。
 * ========================================================= */

static void task_protocol_tx(
    void *parameters
)
{
    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    /*
     * 第一轴目标角：
     *
     * 6000 × 0.01°
     * =
     * 60.00°
     */
    const int16_t joint_targets[
        PROTOCOL_JOINT_COUNT
    ] =
    {
        6000,
        0,
        0,
        0,
        0,
        0
    };


    (void)parameters;


    for (;;)
    {
        /*
         * 构造目标关节角协议帧。
         */
        protocol_build_joint_target_frame(
            joint_targets,
            frame
        );


        /*
         * 发送完整协议帧。
         */
        uart_driver_write(
            frame,
            PROTOCOL_JOINT_FRAME_LEN
        );


        /*
         * 每 1000 ms 发送一次。
         */
        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


/* =========================================================
 * Task 2：
 * Python -> Cortex-M4
 *
 * 使用 Task Notification
 * 等待 UART RX 事件。
 * ========================================================= */

static void task_protocol_rx(
    void *parameters
)
{
    protocol_parser_t parser;

    protocol_frame_t received_frame;


    int16_t joint_states[
        PROTOCOL_JOINT_COUNT
    ];


    uint8_t ack_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    (void)parameters;


    /*
     * 初始化协议解析状态机。
     */
    protocol_parser_init(
        &parser
    );


    /*
     * 此时：
     *
     * 1. FreeRTOS Scheduler 已运行；
     * 2. 当前 ProtocolRX Task 已经存在；
     * 3. ISR Callback 已经注册。
     *
     * 因此现在可以安全开启
     * UART RX Interrupt。
     */
    uart_driver_enable_rx_interrupt();


    for (;;)
    {
        uint8_t byte;


        /*
         * 没有 UART RX 事件时，
         * ProtocolRX Task 在这里阻塞。
         *
         * portMAX_DELAY：
         * 可以无限等待。
         *
         * pdTRUE：
         * Task 被唤醒以后，
         * 将 Notification Count 清零。
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


        /*
         * 收到通知以后，
         * 将 Ring Buffer 中目前已有的
         * 数据全部取出。
         */
        while (
            uart_driver_read_byte(
                &byte
            )
        )
        {
            /*
             * 将 byte 交给 Protocol FSM。
             */
            if (
                protocol_parser_process_byte(
                    &parser,
                    byte,
                    &received_frame
                )
            )
            {
                /*
                 * 当前应用层只处理
                 * JOINT_STATE。
                 */
                if (
                    protocol_parse_joint_state(
                        &received_frame,
                        joint_states
                    )
                )
                {
                    /*
                     * 构造 ACK。
                     */
                    protocol_build_joint_state_ack_frame(
                        joint_states,
                        ack_frame
                    );


                    /*
                     * 发送 ACK。
                     */
                    uart_driver_write(
                        ack_frame,
                        PROTOCOL_JOINT_FRAME_LEN
                    );
                }
            }
        }
    }
}


/* =========================================================
 * 固件入口
 * ========================================================= */

int main(void)
{
    /*
     * 初始化系统时钟。
     */
    board_clock_init();


    /*
     * 初始化 GPIO。
     */
    board_gpio_init();


    /*
     * 初始化 UART。
     *
     * 此阶段只初始化 UART 硬件和
     * Ring Buffer，
     * 暂时不开 RX Interrupt。
     */
    uart_driver_init();


    /*
     * 创建周期发送任务。
     */
    xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    /*
     * 创建协议接收任务。
     *
     * 保存 Task Handle，
     * 后续 ISR 使用该句柄通知任务。
     */
    xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        &protocol_rx_task_handle
    );


    /*
     * 将 UART Driver 的 RX Event
     * 连接到 FreeRTOS Task Notification。
     */
    uart_driver_set_rx_event_callback(
        uart_rx_event_from_isr
    );


    /*
     * 启动 FreeRTOS Scheduler。
     */
    vTaskStartScheduler();


    /*
     * 正常情况下不会运行到这里。
     */
    while (1)
    {
    }
}