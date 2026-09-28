#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "protocol.h"


/* =========================================================
 * UART0 寄存器
 * ========================================================= */

#define UART0_BASE 0x40004000UL

#define UART0_DATA \
    (*(volatile uint32_t *)(UART0_BASE + 0x000UL))

#define UART0_STATE \
    (*(volatile uint32_t *)(UART0_BASE + 0x004UL))

#define UART0_CTRL \
    (*(volatile uint32_t *)(UART0_BASE + 0x008UL))

#define UART0_BAUDDIV \
    (*(volatile uint32_t *)(UART0_BASE + 0x010UL))


/* STATE bit0：TX FIFO 满 */
#define UART_TX_FULL   (1U << 0)

/* STATE bit1：RX FIFO 有数据 */
#define UART_RX_FULL   (1U << 1)


/* =========================================================
 * UART 初始化
 * ========================================================= */

static void uart_init(void)
{
    UART0_BAUDDIV = 16U;

    /*
     * CTRL bit0 = TX enable
     * CTRL bit1 = RX enable
     */
    UART0_CTRL = 3U;
}


/* =========================================================
 * UART 发送
 * ========================================================= */

static void uart_write_byte(uint8_t data)
{
    while (UART0_STATE & UART_TX_FULL)
    {
    }

    UART0_DATA = (uint32_t)data;
}


static void uart_write(
    const uint8_t *data,
    uint32_t length
)
{
    uint32_t i;

    for (i = 0U; i < length; i++)
    {
        uart_write_byte(data[i]);
    }
}


/* =========================================================
 * UART 非阻塞接收
 *
 * 有数据：返回 1
 * 无数据：返回 0
 * ========================================================= */

static uint8_t uart_read_byte(
    uint8_t *data
)
{
    if ((UART0_STATE & UART_RX_FULL) == 0U)
    {
        return 0U;
    }

    *data =
        (uint8_t)(UART0_DATA & 0xFFU);

    return 1U;
}


/* =========================================================
 * Task 1：
 * Cortex-M4 -> Python
 *
 * 周期发送六轴目标角
 * ========================================================= */

static void task_protocol_tx(void *parameters)
{
    uint8_t frame[PROTOCOL_FRAME_LEN];

    /*
     * 第一轴目标 = 60°
     *
     * 协议单位 0.01°
     *
     * 6000 -> 60.00°
     */
    const int16_t joint_targets[PROTOCOL_JOINT_COUNT] =
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
        protocol_build_joint_target_frame(
            joint_targets,
            frame
        );

        uart_write(
            frame,
            PROTOCOL_FRAME_LEN
        );

        /* 每秒发一次 */
        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}


/* =========================================================
 * Task 2：
 * Python -> Cortex-M4
 *
 * 接收 JOINT_STATE
 * ========================================================= */

static void task_protocol_rx(void *parameters)
{
    uint8_t frame[PROTOCOL_FRAME_LEN];

    uint32_t frame_index = 0U;

    int16_t joint_states[
        PROTOCOL_JOINT_COUNT
    ];

    uint8_t ack_frame[
        PROTOCOL_FRAME_LEN
    ];


    (void)parameters;


    for (;;)
    {
        uint8_t byte;


        /* -------------------------------------------------
         * 尝试从 UART 读取一个字节
         * ------------------------------------------------- */

        if (uart_read_byte(&byte))
        {
            /*
             * 极简帧同步：
             *
             * index 0 必须为 AA
             * index 1 必须为 55
             */

            if (frame_index == 0U)
            {
                if (byte != PROTOCOL_HEADER_0)
                {
                    continue;
                }
            }


            if (frame_index == 1U)
            {
                if (byte != PROTOCOL_HEADER_1)
                {
                    frame_index = 0U;
                    continue;
                }
            }


            frame[frame_index] = byte;

            frame_index++;


            /* ---------------------------------------------
             * 收满 17 字节
             * --------------------------------------------- */

            if (frame_index >= PROTOCOL_FRAME_LEN)
            {
                frame_index = 0U;


                /* -----------------------------------------
                 * 验证并解析 JOINT_STATE
                 * ----------------------------------------- */

                if (
                    protocol_parse_joint_state_frame(
                        frame,
                        joint_states
                    )
                )
                {
                    /*
                     * 收到合法状态后，
                     * 立即把相同的六轴状态
                     * 用 0x82 ACK 发回 Python。
                     *
                     * 这样我们能验证 MCU
                     * 确实完成了解析。
                     */
                    protocol_build_joint_state_ack_frame(
                        joint_states,
                        ack_frame
                    );


                    uart_write(
                        ack_frame,
                        PROTOCOL_FRAME_LEN
                    );
                }
            }
        }


        /*
         * 不一直占用 CPU。
         *
         * 每 1 ms 检查一次 UART。
         */
        vTaskDelay(
            pdMS_TO_TICKS(1)
        );
    }
}


/* =========================================================
 * main
 * ========================================================= */

int main(void)
{
    uart_init();


    /* Cortex-M4 -> Python */
    xTaskCreate(
        task_protocol_tx,
        "ProtocolTX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    /* Python -> Cortex-M4 */
    xTaskCreate(
        task_protocol_rx,
        "ProtocolRX",
        configMINIMAL_STACK_SIZE,
        NULL,
        1,
        NULL
    );


    vTaskStartScheduler();


    while (1)
    {
    }
}