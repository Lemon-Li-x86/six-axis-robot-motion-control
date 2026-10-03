/*
 * 文件：main.c
 *
 * 用途：
 * 六轴机器人 Cortex-M4 / FreeRTOS 固件入口。
 *
 * 
 *
 * 1. 初始化板级资源和 UART 驱动；
 * 2. 创建协议发送任务；
 * 3. 创建协议接收任务；
 * 4. 启动 FreeRTOS 调度器。
 *
 * UART 寄存器操作由 uart_driver 模块负责。
 * 协议帧解析由 protocol 模块负责。
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "uart_driver.h"
#include "protocol.h"


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
     * 第一轴目标角 = 60°
     *
     * 当前通信协议单位：
     * 0.01°
     *
     * 因此：
     *
     * 6000 -> 60.00°
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
         * 根据目标关节角构造协议帧。
         */
        protocol_build_joint_target_frame(
            joint_targets,
            frame
        );


        /*
         * 通过 UART 驱动发送完整协议帧。
         */
        uart_driver_write(
            frame,
            PROTOCOL_JOINT_FRAME_LEN
        );


        /*
         * 每 1000 ms 发送一次目标角。
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
 * 接收 Python 发来的字节流，
 * 交给协议状态机处理。
 * ========================================================= */

static void task_protocol_rx(
    void *parameters
)
{
    /*
     * 协议解析器。
     *
     * 用于保存当前字节流
     * 已经解析到哪个状态。
     */
    protocol_parser_t parser;


    /*
     * 保存解析完成的一整帧数据。
     */
    protocol_frame_t received_frame;


    /*
     * 保存解析得到的六轴实际角度。
     */
    int16_t joint_states[
        PROTOCOL_JOINT_COUNT
    ];


    /*
     * MCU 返回给 Python 的 ACK 帧。
     */
    uint8_t ack_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];


    (void)parameters;


    /*
     * 初始化协议状态机。
     */
    protocol_parser_init(
        &parser
    );


    for (;;)
    {
        uint8_t byte;


        /*
         * 当前仍然采用 1 ms 轮询方式，
         * 尝试从 UART 获取一个字节。
         *
         * UART interrupt 和 Ring Buffer
         * 将在后续阶段加入。
         */
        if (
            uart_driver_read_byte(
                &byte
            )
        )
        {
            /*
             * 将当前字节交给协议状态机。
             *
             * 返回 1：
             * 已经得到一帧完整且
             * Checksum 正确的数据。
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
                 * JOINT_STATE 命令。
                 */
                if (
                    protocol_parse_joint_state(
                        &received_frame,
                        joint_states
                    )
                )
                {
                    /*
                     * 收到合法的六轴状态后，
                     * 构造 0x82 ACK。
                     */
                    protocol_build_joint_state_ack_frame(
                        joint_states,
                        ack_frame
                    );


                    /*
                     * 将 ACK 发回 Python。
                     */
                    uart_driver_write(
                        ack_frame,
                        PROTOCOL_JOINT_FRAME_LEN
                    );
                }
            }
        }


        /*
         * 当前暂时保持原有
         * 1 ms UART 轮询方式。
         *
         * 本次修改只重构协议解析，
         * 不同时修改 UART 接收机制。
         */
        vTaskDelay(
            pdMS_TO_TICKS(1)
        );
    }
}


/* =========================================================
 * 固件入口
 * ========================================================= */

int main(void)
{
    /*
     * 初始化板级时钟。
     */
    board_clock_init();


    /*
     * 初始化板级 GPIO。
     */
    board_gpio_init();


    /*
     * 初始化 UART 驱动。
     */
    uart_driver_init();


    /*
     * 创建协议发送任务。
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
     */
    xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    /*
     * 启动 FreeRTOS 调度器。
     */
    vTaskStartScheduler();


    /*
     * 正常情况下，
     * 调度器启动后不会返回这里。
     */
    while (1)
    {
    }
}