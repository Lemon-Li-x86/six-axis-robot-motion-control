/*
 * 文件：protocol.h
 *
 * 用途：
 * 定义 UART 二进制通信协议格式、
 * 通用协议帧、字节流状态机以及对外接口。
 *
 * 核心 Robot 数据类型由 robot_types.h 定义。
 * 通用状态码由 error_code.h 定义。
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * Protocol Header
 * ========================================================= */

#define PROTOCOL_HEADER_0 0xAAU

#define PROTOCOL_HEADER_1 0x55U


/* =========================================================
 * Command
 * ========================================================= */

/* Cortex-M4 -> Python：设置六轴目标关节角。 */
#define CMD_SET_JOINT_TARGETS 0x01U

/* Python -> Cortex-M4：发送六轴实际关节状态。 */
#define CMD_JOINT_STATE 0x81U

/* Cortex-M4 -> Python：确认 Joint State 已接收。 */
#define CMD_JOINT_STATE_ACK 0x82U

/* Python -> Cortex-M4：请求 Diagnostics。 */
#define CMD_GET_DIAGNOSTICS 0x83U

/* Cortex-M4 -> Python：返回 Diagnostics。 */
#define CMD_DIAGNOSTICS_RESPONSE 0x84U


/* =========================================================
 * Joint Protocol
 * ========================================================= */

#define PROTOCOL_JOINT_PAYLOAD_LEN \
    (ROBOT_JOINT_COUNT * 2U)


#define PROTOCOL_JOINT_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_JOINT_PAYLOAD_LEN + 1U)


/*
 * 为旧代码保留兼容名称。
 *
 * 实际 Joint Count
 * 统一定义于 robot_types.h。
 */
#define PROTOCOL_JOINT_COUNT \
    ROBOT_JOINT_COUNT


/* =========================================================
 * Diagnostics Protocol
 * ========================================================= */

/*
 * Diagnostics Response Payload：
 *
 * uint32_t value
 *
 * 共 4 Byte，
 * Little Endian。
 */
#define PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN 4U


#define PROTOCOL_DIAGNOSTICS_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN + 1U)


/*
 * 空 Diagnostics Request
 * 保持旧版行为：
 *
 * Length = 0
 * ->
 * 查询 UART RX Drop Count。
 *
 * Length = 1 时：
 *
 * Payload[0]
 * ->
 * Metric Selector。
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
 * Parser Object
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
 * @brief 初始化协议字节流 Parser。
 *
 * @param[in,out] parser
 * 待初始化 Parser 对象。
 */
void protocol_parser_init(
    protocol_parser_t *parser
);


/**
 * @brief 向协议状态机输入一个字节。
 *
 * @param[in,out] parser
 * Parser 状态对象。
 *
 * @param[in] byte
 * 当前输入字节。
 *
 * @param[out] output_frame
 * 当完整合法帧解析成功时，
 * 输出解析后的完整协议帧。
 *
 * @return
 * 1：
 * 成功解析出完整且 Checksum 正确的协议帧。
 *
 * 0：
 * 当前尚未得到完整合法帧，
 * 或参数为空。
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
 * @brief 构造六轴目标关节角协议帧。
 *
 * @param[in] joints
 * 六轴目标关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param[out] frame
 * 输出完整 CMD_SET_JOINT_TARGETS 协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * joints 或 frame 为空。
 */
robot_status_t protocol_build_joint_target_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 构造 Joint State ACK 帧。
 *
 * @param[in] joints
 * 已接收并保存的六轴关节状态。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param[out] frame
 * 输出完整 CMD_JOINT_STATE_ACK 协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * joints 或 frame 为空。
 */
robot_status_t protocol_build_joint_state_ack_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 从协议帧解析六轴 Joint State。
 *
 * @param[in] frame
 * 已经通过 Parser Checksum 校验的完整协议帧。
 *
 * @param[out] joints
 * 输出六轴关节状态。
 *
 * 单位：
 * 0.01 degree。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 解析成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * frame 或 joints 为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * 当前帧不是 CMD_JOINT_STATE。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 不符合六轴 Joint State 格式。
 */
robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints
);


/* =========================================================
 * Diagnostics API
 * ========================================================= */

/**
 * @brief 解析 Diagnostics Request。
 *
 * @param[in] frame
 * 已通过 Parser 校验的完整协议帧。
 *
 * @param[out] selector
 * 输出需要查询的 Diagnostics Metric。
 *
 * Length = 0 时：
 *
 * selector 自动设为
 * DIAGNOSTICS_METRIC_RX_DROP_COUNT。
 *
 * Length = 1 时：
 *
 * Payload[0] 作为 selector。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 请求合法。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * frame 或 selector 为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * 当前 Command 不是 CMD_GET_DIAGNOSTICS。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 不是 0 或 1。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * Selector 超出当前支持范围。
 */
robot_status_t protocol_parse_diagnostics_request(
    const protocol_frame_t *frame,
    uint8_t *selector
);


/**
 * @brief 判断协议帧是否为合法 Diagnostics Request。
 *
 * @param[in] frame
 * 待判断协议帧。
 *
 * @return
 * 1：
 * 是合法 Diagnostics Request。
 *
 * 0：
 * 不是合法 Diagnostics Request。
 */
uint8_t protocol_is_diagnostics_request(
    const protocol_frame_t *frame
);


/**
 * @brief 构造 Diagnostics Response。
 *
 * @param[in] value
 * 当前 Diagnostics Metric 的 uint32_t 数值。
 *
 * @param[out] frame
 * 输出完整 CMD_DIAGNOSTICS_RESPONSE 协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * frame 为空。
 */
robot_status_t protocol_build_diagnostics_response_frame(
    uint32_t value,
    uint8_t frame[PROTOCOL_DIAGNOSTICS_FRAME_LEN]
);


#endif