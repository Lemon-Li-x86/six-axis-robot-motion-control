#include <stddef.h>
#include <stdint.h>

#include "protocol.h"


/* =========================================================
 * Internal Helpers
 * ========================================================= */

static void protocol_parser_reset(
    protocol_parser_t *parser
)
{
    parser->state = PROTOCOL_STATE_WAIT_HEADER_0;
    parser->payload_index = 0U;
    parser->checksum = 0U;

    parser->frame.command = 0U;
    parser->frame.length = 0U;
}


static robot_status_t protocol_build_frame(
    uint8_t command,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *frame
)
{
    uint8_t checksum;
    uint32_t i;

    if (frame == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    if (payload_length > PROTOCOL_MAX_PAYLOAD_LEN)
    {
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }

    if (payload_length > 0U && payload == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    frame[0] = PROTOCOL_HEADER_0;
    frame[1] = PROTOCOL_HEADER_1;
    frame[2] = command;
    frame[3] = payload_length;

    checksum = (uint8_t)(command + payload_length);

    for (i = 0U; i < payload_length; i++)
    {
        frame[4U + i] = payload[i];

        checksum = (uint8_t)(
            checksum + payload[i]
        );
    }

    frame[4U + payload_length] = checksum;

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Parser
 * ========================================================= */

void protocol_parser_init(
    protocol_parser_t *parser
)
{
    if (parser == NULL)
    {
        return;
    }

    protocol_parser_reset(parser);
}


uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
)
{
    uint32_t i;

    if (parser == NULL || output_frame == NULL)
    {
        return 0U;
    }

    switch (parser->state)
    {
        case PROTOCOL_STATE_WAIT_HEADER_0:

            if (byte == PROTOCOL_HEADER_0)
            {
                parser->state =
                    PROTOCOL_STATE_WAIT_HEADER_1;
            }

            break;

        case PROTOCOL_STATE_WAIT_HEADER_1:

            if (byte == PROTOCOL_HEADER_1)
            {
                parser->state =
                    PROTOCOL_STATE_READ_COMMAND;
            }
            else if (byte != PROTOCOL_HEADER_0)
            {
                protocol_parser_reset(parser);
            }

            break;

        case PROTOCOL_STATE_READ_COMMAND:

            parser->frame.command = byte;
            parser->checksum = byte;

            parser->state =
                PROTOCOL_STATE_READ_LENGTH;

            break;

        case PROTOCOL_STATE_READ_LENGTH:

            parser->frame.length = byte;

            parser->checksum = (uint8_t)(
                parser->checksum + byte
            );

            parser->payload_index = 0U;

            if (parser->frame.length >
                PROTOCOL_MAX_PAYLOAD_LEN)
            {
                protocol_parser_reset(parser);
                break;
            }

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

        case PROTOCOL_STATE_READ_PAYLOAD:

            parser->frame.payload[
                parser->payload_index
            ] = byte;

            parser->payload_index++;

            parser->checksum = (uint8_t)(
                parser->checksum + byte
            );

            if (parser->payload_index >=
                parser->frame.length)
            {
                parser->state =
                    PROTOCOL_STATE_READ_CHECKSUM;
            }

            break;

        case PROTOCOL_STATE_READ_CHECKSUM:

            if (byte == parser->checksum)
            {
                output_frame->command =
                    parser->frame.command;

                output_frame->length =
                    parser->frame.length;

                for (i = 0U;
                     i < parser->frame.length;
                     i++)
                {
                    output_frame->payload[i] =
                        parser->frame.payload[i];
                }

                protocol_parser_reset(parser);

                return 1U;
            }

            protocol_parser_reset(parser);

            break;

        default:

            protocol_parser_reset(parser);

            break;
    }

    return 0U;
}


/* =========================================================
 * Joint Protocol
 * ========================================================= */

static robot_status_t protocol_build_joint_frame(
    uint8_t command,
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
)
{
    uint8_t payload[PROTOCOL_JOINT_PAYLOAD_LEN];
    uint32_t i;

    if (joints == NULL || frame == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        uint16_t value =
            (uint16_t)joints->value[i];

        payload[i * 2U] =
            (uint8_t)(value & 0xFFU);

        payload[i * 2U + 1U] =
            (uint8_t)((value >> 8U) & 0xFFU);
    }

    return protocol_build_frame(
        command,
        payload,
        PROTOCOL_JOINT_PAYLOAD_LEN,
        frame
    );
}


robot_status_t protocol_build_joint_target_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
)
{
    return protocol_build_joint_frame(
        CMD_SET_JOINT_TARGETS,
        joints,
        frame
    );
}


robot_status_t protocol_build_joint_state_ack_frame(
    const robot_joint_angles_t *joints,
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
)
{
    return protocol_build_joint_frame(
        CMD_JOINT_STATE_ACK,
        joints,
        frame
    );
}


robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints
)
{
    uint32_t i;

    if (frame == NULL || joints == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    if (frame->command != CMD_JOINT_STATE)
    {
        return ROBOT_STATUS_ERROR_INVALID_COMMAND;
    }

    if (frame->length != PROTOCOL_JOINT_PAYLOAD_LEN)
    {
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        uint16_t value;

        value =
            (uint16_t)frame->payload[i * 2U]
            |
            (
                (uint16_t)frame->payload[
                    i * 2U + 1U
                ] << 8U
            );

        joints->value[i] =
            (robot_joint_angle_t)value;
    }

    return ROBOT_STATUS_OK;
}


/* =========================================================
 * Parameter Protocol
 * ========================================================= */

robot_status_t protocol_parse_set_parameter(
    const protocol_frame_t *frame,
    uint8_t *parameter_id,
    uint32_t *value
)
{
    if (frame == NULL ||
        parameter_id == NULL ||
        value == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    if (frame->command != CMD_SET_PARAMETER)
    {
        return ROBOT_STATUS_ERROR_INVALID_COMMAND;
    }

    if (frame->length !=
        PROTOCOL_PARAMETER_REQUEST_PAYLOAD_LEN)
    {
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }

    *parameter_id = frame->payload[0];

    *value =
        (uint32_t)frame->payload[1]
        |
        ((uint32_t)frame->payload[2] << 8U)
        |
        ((uint32_t)frame->payload[3] << 16U)
        |
        ((uint32_t)frame->payload[4] << 24U);

    return ROBOT_STATUS_OK;
}


robot_status_t protocol_build_parameter_ack_frame(
    uint8_t parameter_id,
    robot_status_t status,
    uint32_t effective_value,
    uint8_t frame[PROTOCOL_PARAMETER_ACK_FRAME_LEN]
)
{
    uint8_t payload[
        PROTOCOL_PARAMETER_ACK_PAYLOAD_LEN
    ];

    if (frame == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    payload[0] = parameter_id;
    payload[1] = (uint8_t)((int8_t)status);

    payload[2] =
        (uint8_t)(effective_value & 0xFFU);

    payload[3] =
        (uint8_t)((effective_value >> 8U) & 0xFFU);

    payload[4] =
        (uint8_t)((effective_value >> 16U) & 0xFFU);

    payload[5] =
        (uint8_t)((effective_value >> 24U) & 0xFFU);

    return protocol_build_frame(
        CMD_PARAMETER_ACK,
        payload,
        PROTOCOL_PARAMETER_ACK_PAYLOAD_LEN,
        frame
    );
}


/* =========================================================
 * Diagnostics
 * ========================================================= */

robot_status_t protocol_parse_diagnostics_request(
    const protocol_frame_t *frame,
    uint8_t *selector
)
{
    if (frame == NULL || selector == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    if (frame->command != CMD_GET_DIAGNOSTICS)
    {
        return ROBOT_STATUS_ERROR_INVALID_COMMAND;
    }

    if (frame->length == 0U)
    {
        *selector =
            DIAGNOSTICS_METRIC_RX_DROP_COUNT;

        return ROBOT_STATUS_OK;
    }

    if (frame->length != 1U)
    {
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    }

    if (frame->payload[0] > DIAGNOSTICS_METRIC_MAX)
    {
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }

    *selector = frame->payload[0];

    return ROBOT_STATUS_OK;
}


uint8_t protocol_is_diagnostics_request(
    const protocol_frame_t *frame
)
{
    uint8_t selector;

    return (
        protocol_parse_diagnostics_request(
            frame,
            &selector
        ) == ROBOT_STATUS_OK
    ) ? 1U : 0U;
}


robot_status_t protocol_build_diagnostics_response_frame(
    uint32_t value,
    uint8_t frame[PROTOCOL_DIAGNOSTICS_FRAME_LEN]
)
{
    uint8_t payload[
        PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN
    ];

    if (frame == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }

    payload[0] =
        (uint8_t)(value & 0xFFU);

    payload[1] =
        (uint8_t)((value >> 8U) & 0xFFU);

    payload[2] =
        (uint8_t)((value >> 16U) & 0xFFU);

    payload[3] =
        (uint8_t)((value >> 24U) & 0xFFU);

    return protocol_build_frame(
        CMD_DIAGNOSTICS_RESPONSE,
        payload,
        PROTOCOL_DIAGNOSTICS_PAYLOAD_LEN,
        frame
    );
}