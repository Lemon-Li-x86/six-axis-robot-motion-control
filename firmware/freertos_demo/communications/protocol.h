/*
 * 文件：protocol.h
 *
 * 用途：
 * 定义 UART 通信协议的数据格式、协议帧结构、
 * 字节流解析状态机以及对外接口。
 *
 * Robot 核心数据类型由 robot_types.h 统一定义。
 * 通用返回状态由 error_code.h 统一定义。
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * 协议基本定义
 * ========================================================= */

#define PROTOCOL_HEADER_0 0xAAU
#define PROTOCOL_HEADER_1 0x55U


/* =========================================================
 * Command
 * ========================================================= */

/* Cortex-M4 -> Python：设置六轴目标角 */
#define CMD_SET_JOINT_TARGETS 0x01U

/* Python -> Cortex-M4：发送六轴实际状态 */
#define CMD_JOINT_STATE 0x81U

/* Cortex-M4 -> Python：确认状态接收完成 */
#define CMD_JOINT_STATE_ACK 0x82U

/* Python -> Cortex-M4：请求诊断信息 */
#define CMD_GET_DIAGNOSTICS 0x83U

/* Cortex-M4 -> Python：返回诊断信息 */
#define CMD_DIAGNOSTICS_RESPONSE 0x84U


/* =========================================================
 * Joint Protocol
 * ========================================================= */

/*
 * 每个 Joint：
 *
 * robot_joint_angle_t
 * =
 * int16_t
 * =
 * 2 Byte
 */
#define PROTOCOL_JOINT_PAYLOAD_LEN \
    (ROBOT_JOINT_COUNT * 2U)


/*
 * Header:
 * 2 Byte
 *
 * Command:
 * 1 Byte
 *
 * Length:
 * 1 Byte
 *
 * Payload:
 * 12 Byte
 *
 * Checksum:
 * 1 Byte
 */
#define PROTOCOL_JOINT_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_JOINT_PAYLOAD_LEN + 1U)


/*
 * 为旧代码提供兼容名称。
 *
 * 真正的机器人 Joint Count
 * 已统一定义在 robot_types.h。
 */
#define PROTOCOL_JOINT_COUNT \
    ROBOT_JOINT_COUNT


/* =========================================================
 * Diagnostics Protocol
 * ========================================================= */

/*
 * 当前 Diagnostics Payload：
 *
 * uint32_t uart_rx_drop_count
 */
#define PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN 4U


#define PROTOCOL_DIAGNOSTICS_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN + 1U)


/* =========================================================
 * 通用协议配置
 * ========================================================= */

#define PROTOCOL_MAX_PAYLOAD_LEN 64U


/* =========================================================
 * 通用协议帧
 * ========================================================= */

typedef struct
{
    uint8_t command;

    uint8_t length;

    uint8_t payload[
        PROTOCOL_MAX_PAYLOAD_LEN
    ];

} protocol_frame_t;


/* =========================================================
 * Parser State
 * ========================================================= */

typedef enum
{
    PROTOCOL_STATE_WAIT_HEADER_0 = 0,

    PROTOCOL_STATE_WAIT_HEADER_1,

    PROTOCOL_STATE_READ_COMMAND,

    PROTOCOL_STATE_READ_LENGTH,

    PROTOCOL_STATE_READ_PAYLOAD,

    PROTOCOL_STATE_READ_CHECKSUM

} protocol_parser_state_t;


/* =========================================================
 * Parser
 * ========================================================= */

typedef struct
{
    protocol_parser_state_t state;

    protocol_frame_t frame;

    uint8_t payload_index;

    uint8_t checksum;

} protocol_parser_t;


/* =========================================================
 * Parser API
 * ========================================================= */

/**
 * @brief 初始化协议解析器。
 *
 * @param parser
 * Parser 对象。
 */
void protocol_parser_init(
    protocol_parser_t *parser
);


/**
 * @brief 向协议状态机输入一个 Byte。
 *
 * @param parser
 * Parser 对象。
 *
 * @param byte
 * 当前输入字节。
 *
 * @param output_frame
 * 完整帧输出位置。
 *
 * @return
 * 1：
 * 得到完整且 Checksum 正确的协议帧。
 *
 * 0：
 * 当前尚未得到完整帧。
 */
uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
);


/* =========================================================
 * Joint Protocol API
 * ========================================================= */

/**
 * @brief 构造目标关节角协议帧。
 *
 * @param joints
 * 六轴目标关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param frame
 * 输出协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 成功。
 *
 * 其他：
 * 参数错误。
 */
robot_status_t protocol_build_joint_target_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 构造 JOINT_STATE ACK。
 */
robot_status_t protocol_build_joint_state_ack_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 从协议帧解析六轴 Joint State。
 *
 * @param frame
 * 完整协议帧。
 *
 * @param joints
 * 输出六轴关节角。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 成功。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * Command 错误。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 错误。
 */
robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints
);


/* =========================================================
 * Diagnostics API
 * ========================================================= */

/**
 * @brief 判断当前帧是否为 Diagnostics Request。
 *
 * 正确格式：
 *
 * Command = 0x83
 * Length = 0
 *
 * @return
 * 1：
 * 是。
 *
 * 0：
 * 不是。
 */
uint8_t protocol_is_diagnostics_request(
    const protocol_frame_t *frame
);


/**
 * @brief 构造 Diagnostics Response。
 */
robot_status_t protocol_build_diagnostics_response_frame(
    uint32_t uart_rx_drop_count,
    uint8_t frame[PROTOCOL_DIAGNOSTICS_FRAME_LEN]
);


#endif