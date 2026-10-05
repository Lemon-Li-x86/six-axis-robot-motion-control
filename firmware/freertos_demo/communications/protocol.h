/*
 * 文件：protocol.h
 *
 * 用途：
 * 定义 Cortex-M4 与 Python 仿真端之间的
 * 二进制通信协议公共接口。
 *
 * 当前协议支持：
 *
 * 1. 可变长度 Payload；
 * 2. Byte Stream Parser；
 * 3. Checksum；
 * 4. 六轴关节目标与状态；
 * 5. Runtime Parameter；
 * 6. Diagnostics。
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
 * Commands
 * ========================================================= */

#define CMD_SET_JOINT_TARGETS        0x01U
#define CMD_SET_PARAMETER            0x02U

#define CMD_JOINT_STATE              0x81U
#define CMD_JOINT_STATE_ACK          0x82U
#define CMD_GET_DIAGNOSTICS          0x83U
#define CMD_DIAGNOSTICS_RESPONSE     0x84U
#define CMD_PARAMETER_ACK            0x85U


/* =========================================================
 * Joint Protocol
 * ========================================================= */

#define PROTOCOL_JOINT_PAYLOAD_LEN \
    (ROBOT_JOINT_COUNT * 2U)

#define PROTOCOL_JOINT_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_JOINT_PAYLOAD_LEN + 1U)


/* =========================================================
 * Parameter Protocol
 * ========================================================= */

#define PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS 0x01U

#define PROTOCOL_PARAMETER_REQUEST_PAYLOAD_LEN 5U

#define PROTOCOL_PARAMETER_REQUEST_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_PARAMETER_REQUEST_PAYLOAD_LEN + 1U)

#define PROTOCOL_PARAMETER_ACK_PAYLOAD_LEN 6U

#define PROTOCOL_PARAMETER_ACK_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_PARAMETER_ACK_PAYLOAD_LEN + 1U)


/* =========================================================
 * Diagnostics
 * ========================================================= */

#define PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN 4U

#define PROTOCOL_DIAGNOSTICS_FRAME_LEN \
    (2U + 1U + 1U + PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN + 1U)

#define DIAGNOSTICS_METRIC_RX_DROP_COUNT                0U
#define DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ           1U
#define DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT          2U
#define DIAGNOSTICS_METRIC_PARSER_MIN_TICKS             3U
#define DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS         4U
#define DIAGNOSTICS_METRIC_PARSER_MAX_TICKS             5U
#define DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT     6U
#define DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS        7U
#define DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS    8U
#define DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS        9U

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


/* =========================================================
 * Parser API
 * ========================================================= */

/**
 * @brief 初始化 Protocol Byte Stream Parser。
 *
 * @param[in,out] parser
 * 待初始化的 Parser State。
 *
 * @note
 * parser 为 NULL 时函数直接返回。
 */
void protocol_parser_init(
    protocol_parser_t *parser
);


/**
 * @brief 向 Parser 输入一个 Byte。
 *
 * @param[in,out] parser
 * Parser State。
 *
 * @param[in] byte
 * 当前输入 Byte。
 *
 * @param[out] output_frame
 * 当合法完整 Frame 解析完成时，
 * 输出解析结果。
 *
 * @return
 * 1：
 * 已解析出一个完整合法 Frame。
 *
 * 0：
 * 当前尚未形成完整合法 Frame，
 * 或输入参数无效。
 */
uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
);


/* =========================================================
 * Joint API
 * ========================================================= */

/**
 * @brief 构造 SET_JOINT_TARGETS Frame。
 *
 * @param[in] joints
 * 六轴目标关节角。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param[out] frame
 * 输出完整协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出指针为空。
 */
robot_status_t protocol_build_joint_target_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 构造 JOINT_STATE_ACK Frame。
 *
 * @param[in] joints
 * 六轴关节状态。
 *
 * 单位：
 * 0.01 degree。
 *
 * @param[out] frame
 * 输出完整协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出指针为空。
 */
robot_status_t protocol_build_joint_state_ack_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 解析 JOINT_STATE Frame。
 *
 * @param[in] frame
 * 输入协议帧。
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
 * 输入或输出指针为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * Command 不是 CMD_JOINT_STATE。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 非法。
 */
robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints
);


/* =========================================================
 * Parameter API
 * ========================================================= */

/**
 * @brief 解析 SET_PARAMETER Frame。
 *
 * @param[in] frame
 * 输入协议帧。
 *
 * @param[out] parameter_id
 * 输出 Parameter ID。
 *
 * @param[out] value
 * 输出 Parameter Value。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 解析成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出指针为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * Command 非法。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 非法。
 */
robot_status_t protocol_parse_set_parameter(
    const protocol_frame_t *frame,
    uint8_t *parameter_id,
    uint32_t *value
);


/**
 * @brief 构造 PARAMETER_ACK Frame。
 *
 * @param[in] parameter_id
 * Parameter ID。
 *
 * @param[in] status
 * 参数配置结果。
 *
 * @param[in] effective_value
 * 当前实际生效值。
 *
 * @param[out] frame
 * 输出完整协议帧。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 构造成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * frame 为空。
 */
robot_status_t protocol_build_parameter_ack_frame(
    uint8_t parameter_id,
    robot_status_t status,
    uint32_t effective_value,
    uint8_t frame[PROTOCOL_PARAMETER_ACK_FRAME_LEN]
);


/* =========================================================
 * Diagnostics API
 * ========================================================= */

/**
 * @brief 解析 GET_DIAGNOSTICS Frame。
 *
 * @param[in] frame
 * 输入协议帧。
 *
 * @param[out] selector
 * 输出 Diagnostics Metric Selector。
 *
 * @return
 * ROBOT_STATUS_OK：
 * 解析成功。
 *
 * ROBOT_STATUS_ERROR_NULL_POINTER：
 * 输入或输出指针为空。
 *
 * ROBOT_STATUS_ERROR_INVALID_COMMAND：
 * Command 非法。
 *
 * ROBOT_STATUS_ERROR_INVALID_LENGTH：
 * Payload Length 非法。
 *
 * ROBOT_STATUS_ERROR_INVALID_ARGUMENT：
 * Selector 超出支持范围。
 *
 * @note
 * Length 为 0 时使用默认 Selector：
 * DIAGNOSTICS_METRIC_RX_DROP_COUNT。
 */
robot_status_t protocol_parse_diagnostics_request(
    const protocol_frame_t *frame,
    uint8_t *selector
);


/**
 * @brief 判断 Frame 是否为合法 Diagnostics Request。
 *
 * @param[in] frame
 * 输入协议帧。
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
 * @brief 构造 DIAGNOSTICS_RESPONSE Frame。
 *
 * @param[in] value
 * 需要返回的 32 bit Diagnostics Value。
 *
 * @param[out] frame
 * 输出完整协议帧。
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