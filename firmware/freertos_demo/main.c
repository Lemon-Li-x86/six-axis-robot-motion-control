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
#include "ring_buffer.h"


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
 * UART 收到的数据先进入 Ring Buffer，
 * 再由协议状态机逐字节处理。
 * ========================================================= */

static void task_protocol_rx(
    void *parameters
)
{
    protocol_parser_t parser;

    protocol_frame_t received_frame;

    ring_buffer_t rx_buffer;


    int16_t joint_states[
        PROTOCOL_JOINT_COUNT
    ];


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


    /*
     * 初始化 UART 接收环形缓冲区。
     */
    ring_buffer_init(
        &rx_buffer
    );


    for (;;)
    {
        uint8_t byte;


        /* -------------------------------------------------
         * 第一阶段：
         * 尽可能读取 UART 当前已有的数据，
         * 并写入 Ring Buffer。
         * ------------------------------------------------- */

        while (
            uart_driver_read_byte(
                &byte
            )
        )
        {
            /*
             * 当前缓冲区满时，
             * 暂时直接停止继续写入。
             *
             * 后续会增加正式的错误处理机制。
             */
            if (
                !ring_buffer_write(
                    &rx_buffer,
                    byte
                )
            )
            {
                break;
            }
        }


        /* -------------------------------------------------
         * 第二阶段：
         * 从 Ring Buffer 读取所有已有数据，
         * 依次交给协议状态机。
         * ------------------------------------------------- */

        while (
            ring_buffer_read(
                &rx_buffer,
                &byte
            )
        )
        {
            /*
             * 将字节交给协议解析器。
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
                     * 构造并发送 ACK。
                     */
                    protocol_build_joint_state_ack_frame(
                        joint_states,
                        ack_frame
                    );


                    uart_driver_write(
                        ack_frame,
                        PROTOCOL_JOINT_FRAME_LEN
                    );
                }
            }
        }


        /*
         * 当前仍然保持 1 ms 周期轮询。
         *
         * 下一阶段将把 UART 接收
         * 改为中断方式。
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