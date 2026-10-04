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
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_LEN];

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

void protocol_parser_init(
    protocol_parser_t *parser
);

uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
);


/* =========================================================
 * Joint API
 * ========================================================= */

robot_status_t protocol_build_joint_target_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);

robot_status_t protocol_build_joint_state_ack_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);

robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints
);


/* =========================================================
 * Parameter API
 * ========================================================= */

robot_status_t protocol_parse_set_parameter(
    const protocol_frame_t *frame,
    uint8_t *parameter_id,
    uint32_t *value
);

robot_status_t protocol_build_parameter_ack_frame(
    uint8_t parameter_id,
    robot_status_t status,
    uint32_t effective_value,
    uint8_t frame[PROTOCOL_PARAMETER_ACK_FRAME_LEN]
);


/* =========================================================
 * Diagnostics API
 * ========================================================= */

robot_status_t protocol_parse_diagnostics_request(
    const protocol_frame_t *frame,
    uint8_t *selector
);

uint8_t protocol_is_diagnostics_request(
    const protocol_frame_t *frame
);

robot_status_t protocol_build_diagnostics_response_frame(
    uint32_t value,
    uint8_t frame[PROTOCOL_DIAGNOSTICS_FRAME_LEN]
);


#endif