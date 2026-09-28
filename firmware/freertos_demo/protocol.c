#include "protocol.h"


/* =========================================================
 * 通用：构造六轴关节数据帧
 * ========================================================= */

static void protocol_build_joint_frame(
    uint8_t command,
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_FRAME_LEN]
)
{
    uint32_t i;
    uint8_t checksum = 0U;


    frame[0] = PROTOCOL_HEADER_0;
    frame[1] = PROTOCOL_HEADER_1;

    frame[2] = command;
    frame[3] = PROTOCOL_PAYLOAD_LEN;


    /* 六个 int16_t，按 little-endian 写入 */
    for (i = 0U; i < PROTOCOL_JOINT_COUNT; i++)
    {
        uint16_t value = (uint16_t)joints[i];

        frame[4U + i * 2U] =
            (uint8_t)(value & 0xFFU);

        frame[5U + i * 2U] =
            (uint8_t)((value >> 8U) & 0xFFU);
    }


    /* Command + Length + Payload */
    for (i = 2U; i < 16U; i++)
    {
        checksum =
            (uint8_t)(checksum + frame[i]);
    }

    frame[16] = checksum;
}


/* =========================================================
 * Cortex-M4 -> Python：目标角
 * ========================================================= */

void protocol_build_joint_target_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_FRAME_LEN]
)
{
    protocol_build_joint_frame(
        CMD_SET_JOINT_TARGETS,
        joints,
        frame
    );
}


/* =========================================================
 * Cortex-M4 -> Python：状态 ACK
 * ========================================================= */

void protocol_build_joint_state_ack_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_FRAME_LEN]
)
{
    protocol_build_joint_frame(
        CMD_JOINT_STATE_ACK,
        joints,
        frame
    );
}


/* =========================================================
 * Python -> Cortex-M4：解析 JOINT_STATE
 * ========================================================= */

uint8_t protocol_parse_joint_state_frame(
    const uint8_t frame[PROTOCOL_FRAME_LEN],
    int16_t joints[PROTOCOL_JOINT_COUNT]
)
{
    uint32_t i;
    uint8_t checksum = 0U;


    /* 帧头检查 */
    if (
        frame[0] != PROTOCOL_HEADER_0 ||
        frame[1] != PROTOCOL_HEADER_1
    )
    {
        return 0U;
    }


    /* Command 检查 */
    if (frame[2] != CMD_JOINT_STATE)
    {
        return 0U;
    }


    /* Payload 长度检查 */
    if (frame[3] != PROTOCOL_PAYLOAD_LEN)
    {
        return 0U;
    }


    /* Checksum */
    for (i = 2U; i < 16U; i++)
    {
        checksum =
            (uint8_t)(checksum + frame[i]);
    }


    if (checksum != frame[16])
    {
        return 0U;
    }


    /* little-endian -> int16_t */
    for (i = 0U; i < PROTOCOL_JOINT_COUNT; i++)
    {
        uint16_t value;

        value =
            (uint16_t)frame[4U + i * 2U]
            |
            ((uint16_t)frame[5U + i * 2U] << 8U);

        joints[i] = (int16_t)value;
    }


    return 1U;
}