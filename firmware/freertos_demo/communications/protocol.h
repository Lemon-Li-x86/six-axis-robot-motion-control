/*
 * 文件：protocol.h
 *
 * 用途：
 * 定义 UART 二进制通信协议格式、
 * 通用协议帧、字节流状态机以及对外接口。
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


#define PROTOCOL_HEADER_0 0xAAU

#define PROTOCOL_HEADER_1 0x55U


/* =========================================================
 * Command
 * ========================================================= */

#define CMD_SET_JOINT_TARGETS 0x01U

#define CMD_JOINT_STATE 0x81U

#define CMD_JOINT_STATE_ACK 0x82U

#define CMD_GET_DIAGNOSTICS 0x83U

#define CMD_DIAGNOSTICS_RESPONSE 0x84U


/* =========================================================
 * Joint Protocol
 * ========================================================= */

#define PROTOCOL_JOINT_PAYLOAD_LEN \
    (ROBOT_JOINT_COUNT * 2U)


#define PROTOCOL_JOINT_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_JOINT_PAYLOAD_LEN + 1U)


#define PROTOCOL_JOINT_COUNT \
    ROBOT_JOINT_COUNT


/* =========================================================
 * Diagnostics Protocol
 * ========================================================= */

/*
 * Diagnostics Response
 * 始终返回一个 uint32_t。
 */
#define PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN 4U


#define PROTOCOL_DIAGNOSTICS_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN + 1U)


/*
 * 空 Diagnostics Request
 * 保持旧行为：
 *
 * 返回 UART RX Drop Count。
 *
 * 如果 Request Payload 长度为 1，
 * 则该字节表示下面的 Metric Selector。
 */
#define DIAGNOSTICS_METRIC_RX_DROP_COUNT 0U

#define DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ 1U

#define DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT 2U

#define DIAGNOSTICS_METRIC_PARSER_MIN_TICKS 3U

#define DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS 4U

#define DIAGNOSTICS_METRIC_PARSER_MAX_TICKS 5U

#define DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT 6U

#define DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS 7U

#define DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS 8U

#define DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS 9U

#define DIAGNOSTICS_METRIC_MAX \
    DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS


/* =========================================================
 * Generic Protocol
 * ========================================================= */

#define PROTOCOL_MAX_PAYLOAD_LEN 64U


typedef struct
{
    uint8_t command;

    uint8_t length;

    uint8_t payload[
        PROTOCOL_MAX_PAYLOAD_LEN
    ];

} protocol_frame_t;


typedef enum
{
    PROTOCOL_STATE_WAIT_HEADER_0 = 0,

    PROTOCOL_STATE_WAIT_HEADER_1,

    PROTOCOL_STATE_READ_COMMAND,

    PROTOCOL_STATE_READ_LENGTH,

    PROTOCOL_STATE_READ_PAYLOAD,

    PROTOCOL_STATE_READ_CHECKSUM

} protocol_parser_state_t;


typedef struct
{
    protocol_parser_state_t state;

    protocol_frame_t frame;

    uint8_t payload_index;

    uint8_t checksum;

} protocol_parser_t;


/**
 * @brief 初始化协议 Parser。
 *
 * @param[in,out] parser
 * Parser 对象。
 */
void protocol_parser_init(
    protocol_parser_t *parser
);


/**
 * @brief 向协议 Parser 输入一个字节。
 *
 * @param[in,out] parser
 * Parser 状态对象。
 *
 * @param[in] byte
 * 当前输入字节。
 *
 * @param[out] output_frame
 * 完整合法协议帧输出位置。
 *
 * @return
 * 1：
 * 已解析得到完整合法帧。
 *
 * 0：
 * 尚未得到完整合法帧。
 */
uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
);


/**
 * @brief 构造目标关节角协议帧。
 */
robot_status_t protocol_build_joint_target_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 构造 Joint State ACK。
 */
robot_status_t protocol_build_joint_state_ack_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 解析 Joint State。
 */
robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints
);


/**
 * @brief 解析 Diagnostics Request。
 *
 * @param[in] frame
 * 完整协议帧。
 *
 * @param[out] selector
 * 输出需要查询的性能指标编号。
 *
 * Length = 0 时：
 * selector 自动设为
 * DIAGNOSTICS_METRIC_RX_DROP_COUNT。
 *
 * Length = 1 时：
 * Payload[0] 作为 selector。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 请求合法。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * Command 错误。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 不是 0 或 1。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * Selector 超出支持范围。
 */
robot_status_t protocol_parse_diagnostics_request(
    const protocol_frame_t *frame,
    uint8_t *selector
);


/**
 * @brief 判断当前帧是否为合法 Diagnostics Request。
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
 *
 * @param[in] value
 * 当前查询指标的 uint32_t 数值。
 *
 * @param[out] frame
 * 输出 Diagnostics Response。
 */
robot_status_t protocol_build_diagnostics_response_frame(
    uint32_t value,
    uint8_t frame[PROTOCOL_DIAGNOSTICS_FRAME_LEN]
);


#endif