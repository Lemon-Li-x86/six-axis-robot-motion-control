#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>


/* =========================================================
 * UART 二进制协议定义
 * ========================================================= */

#define PROTOCOL_HEADER_0           0xAAU
#define PROTOCOL_HEADER_1           0x55U

/* Cortex-M4 -> Python：六轴目标角 */
#define CMD_SET_JOINT_TARGETS       0x01U

/* Python -> Cortex-M4：六轴实际状态 */
#define CMD_JOINT_STATE             0x81U

/* Cortex-M4 -> Python：状态接收确认 */
#define CMD_JOINT_STATE_ACK         0x82U

#define PROTOCOL_JOINT_COUNT        6U
#define PROTOCOL_PAYLOAD_LEN        12U
#define PROTOCOL_FRAME_LEN          17U


/* 构造六轴目标角帧 */
void protocol_build_joint_target_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_FRAME_LEN]
);


/* 构造状态 ACK 帧 */
void protocol_build_joint_state_ack_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_FRAME_LEN]
);


/* 解析 Python 发来的 JOINT_STATE 帧
 *
 * 成功：返回 1
 * 失败：返回 0
 */
uint8_t protocol_parse_joint_state_frame(
    const uint8_t frame[PROTOCOL_FRAME_LEN],
    int16_t joints[PROTOCOL_JOINT_COUNT]
);


#endif