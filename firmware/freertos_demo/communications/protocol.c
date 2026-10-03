/*
 * 文件：protocol.c
 *
 * 用途：
 * 实现 UART 二进制协议。
 *
 * 主要负责：
 *
 * 1. 将 UART 收到的连续字节流解析成完整协议帧；
 * 2. 根据 Length 支持不同长度的 Payload；
 * 3. 验证 Checksum；
 * 4. 构造六轴目标角和 ACK 帧；
 * 5. 将 JOINT_STATE Payload 解析为六轴关节数据。
 *
 * 本文件不直接访问 UART 硬件。
 */

#include "protocol.h"


/* =========================================================
 * 重置协议解析器
 * ========================================================= */

static void protocol_parser_reset(
    protocol_parser_t *parser
)
{
    parser->state =
        PROTOCOL_STATE_WAIT_HEADER_0;

    parser->payload_index = 0U;

    parser->checksum = 0U;

    parser->frame.command = 0U;

    parser->frame.length = 0U;
}


/* =========================================================
 * 初始化协议解析器
 * ========================================================= */

void protocol_parser_init(
    protocol_parser_t *parser
)
{
    protocol_parser_reset(parser);
}


/* =========================================================
 * 字节流协议状态机
 * ========================================================= */

uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
)
{
    uint32_t i;


    switch (parser->state)
    {
        /* -------------------------------------------------
         * 等待第一个帧头字节 AA
         * ------------------------------------------------- */

        case PROTOCOL_STATE_WAIT_HEADER_0:

            if (byte == PROTOCOL_HEADER_0)
            {
                parser->state =
                    PROTOCOL_STATE_WAIT_HEADER_1;
            }

            break;


        /* -------------------------------------------------
         * 等待第二个帧头字节 55
         * ------------------------------------------------- */

        case PROTOCOL_STATE_WAIT_HEADER_1:

            if (byte == PROTOCOL_HEADER_1)
            {
                parser->state =
                    PROTOCOL_STATE_READ_COMMAND;
            }
            else if (byte == PROTOCOL_HEADER_0)
            {
                /*
                 * 如果连续收到 AA，
                 * 当前这个 AA 仍可能是下一帧的开始，
                 * 因此继续等待 55。
                 */
                parser->state =
                    PROTOCOL_STATE_WAIT_HEADER_1;
            }
            else
            {
                /*
                 * 帧头错误，
                 * 回到初始状态重新寻找 AA。
                 */
                protocol_parser_reset(parser);
            }

            break;


        /* -------------------------------------------------
         * 读取 Command
         * ------------------------------------------------- */

        case PROTOCOL_STATE_READ_COMMAND:

            parser->frame.command = byte;

            /*
             * Checksum 从 Command 开始累计。
             */
            parser->checksum = byte;

            parser->state =
                PROTOCOL_STATE_READ_LENGTH;

            break;


        /* -------------------------------------------------
         * 读取 Payload Length
         * ------------------------------------------------- */

        case PROTOCOL_STATE_READ_LENGTH:

            parser->frame.length = byte;

            parser->checksum =
                (uint8_t)(
                    parser->checksum + byte
                );

            parser->payload_index = 0U;


            /*
             * Payload 长度不能超过缓冲区容量。
             *
             * 如果长度非法，直接丢弃当前帧，
             * 防止数组越界。
             */
            if (
                parser->frame.length >
                PROTOCOL_MAX_PAYLOAD_LEN
            )
            {
                protocol_parser_reset(parser);

                break;
            }


            /*
             * 如果 Payload 长度为 0，
             * 可以直接读取 Checksum。
             */
            if (parser->frame.length == 0U)
            {
                parser->state =
                    PROTOCOL_STATE_READ_CHECKSUM;
            }
            else
            {
                parser->state =
                    PROTOCOL_STATE_READ_PAYLOAD;
            }

            break;


        /* -------------------------------------------------
         * 读取 Payload
         * ------------------------------------------------- */

        case PROTOCOL_STATE_READ_PAYLOAD:

            parser->frame.payload[
                parser->payload_index
            ] = byte;

            parser->payload_index++;

            parser->checksum =
                (uint8_t)(
                    parser->checksum + byte
                );


            /*
             * 已经收满 Length 指定的 Payload，
             * 下一字节应该是 Checksum。
             */
            if (
                parser->payload_index >=
                parser->frame.length
            )
            {
                parser->state =
                    PROTOCOL_STATE_READ_CHECKSUM;
            }

            break;


        /* -------------------------------------------------
         * 读取并验证 Checksum
         * ------------------------------------------------- */

        case PROTOCOL_STATE_READ_CHECKSUM:

            if (byte == parser->checksum)
            {
                /*
                 * Checksum 正确。
                 *
                 * 将完整协议帧复制给调用者。
                 */
                output_frame->command =
                    parser->frame.command;

                output_frame->length =
                    parser->frame.length;


                for (
                    i = 0U;
                    i < parser->frame.length;
                    i++
                )
                {
                    output_frame->payload[i] =
                        parser->frame.payload[i];
                }


                /*
                 * 当前帧完成，
                 * 重置状态机准备接收下一帧。
                 */
                protocol_parser_reset(parser);

                return 1U;
            }


            /*
             * Checksum 错误。
             *
             * 当前帧无效，直接丢弃，
             * 重新寻找下一帧帧头。
             */
            protocol_parser_reset(parser);

            break;


        /* -------------------------------------------------
         * 理论上不应该进入未知状态。
         * 如果状态异常，则重新初始化解析器。
         * ------------------------------------------------- */

        default:

            protocol_parser_reset(parser);

            break;
    }


    return 0U;
}


/* =========================================================
 * 通用六轴关节帧构造函数
 * ========================================================= */

static void protocol_build_joint_frame(
    uint8_t command,
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
)
{
    uint32_t i;

    uint8_t checksum = 0U;


    /* 固定帧头 */
    frame[0] = PROTOCOL_HEADER_0;

    frame[1] = PROTOCOL_HEADER_1;


    /* Command */
    frame[2] = command;


    /* Payload 长度 */
    frame[3] =
        PROTOCOL_JOINT_PAYLOAD_LEN;


    /*
     * 六个 int16_t 关节角，
     * 按 little-endian 写入 Payload。
     */
    for (
        i = 0U;
        i < PROTOCOL_JOINT_COUNT;
        i++
    )
    {
        uint16_t value =
            (uint16_t)joints[i];


        frame[4U + i * 2U] =
            (uint8_t)(
                value & 0xFFU
            );


        frame[5U + i * 2U] =
            (uint8_t)(
                (value >> 8U) & 0xFFU
            );
    }


    /*
     * Checksum =
     * Command + Length + Payload
     */
    for (
        i = 2U;
        i < (PROTOCOL_JOINT_FRAME_LEN - 1U);
        i++
    )
    {
        checksum =
            (uint8_t)(
                checksum + frame[i]
            );
    }


    /*
     * 最后一个字节保存 Checksum。
     */
    frame[
        PROTOCOL_JOINT_FRAME_LEN - 1U
    ] = checksum;
}


/* =========================================================
 * Cortex-M4 -> Python
 * 构造六轴目标角帧
 * ========================================================= */

void protocol_build_joint_target_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
)
{
    protocol_build_joint_frame(
        CMD_SET_JOINT_TARGETS,
        joints,
        frame
    );
}


/* =========================================================
 * Cortex-M4 -> Python
 * 构造状态 ACK 帧
 * ========================================================= */

void protocol_build_joint_state_ack_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
)
{
    protocol_build_joint_frame(
        CMD_JOINT_STATE_ACK,
        joints,
        frame
    );
}


/* =========================================================
 * Python -> Cortex-M4
 * 解析 JOINT_STATE
 * ========================================================= */

uint8_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    int16_t joints[PROTOCOL_JOINT_COUNT]
)
{
    uint32_t i;


    /*
     * 当前函数只处理
     * Python 发来的 JOINT_STATE。
     */
    if (
        frame->command !=
        CMD_JOINT_STATE
    )
    {
        return 0U;
    }


    /*
     * 六个 int16_t 应为 12 Byte。
     */
    if (
        frame->length !=
        PROTOCOL_JOINT_PAYLOAD_LEN
    )
    {
        return 0U;
    }


    /*
     * little-endian
     * ->
     * int16_t
     */
    for (
        i = 0U;
        i < PROTOCOL_JOINT_COUNT;
        i++
    )
    {
        uint16_t value;


        value =
            (uint16_t)
            frame->payload[i * 2U]
            |
            (
                (uint16_t)
                frame->payload[
                    i * 2U + 1U
                ]
                << 8U
            );


        joints[i] =
            (int16_t)value;
    }


    return 1U;
}