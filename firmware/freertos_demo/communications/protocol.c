#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "protocol.h"


/* =========================================================
 * Internal Helpers
 * ========================================================= */

void protocol_parser_reset(
    protocol_parser_t *parser
)
{
    parser->state = PROTOCOL_STATE_WAIT_HEADER_0;
    parser->payload_index = 0U;
    parser->checksum = 0U;

    parser->frame.command = 0U;
    parser->frame.length = 0U;
}


uint8_t protocol_parser_expire_partial_frame(
    protocol_parser_t *parser,
    uint32_t idle_ticks,
    uint32_t timeout_ticks
)
{
    if (parser == NULL || timeout_ticks == 0U ||
        parser->state == PROTOCOL_STATE_WAIT_HEADER_0 ||
        idle_ticks < timeout_ticks)
    {
        return 0U;
    }

    protocol_parser_reset(parser);
    return 1U;
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

            /*
             * If a prior payload byte was lost, this byte may be the
             * next frame's 0xAA header rather than the old checksum.
             * Re-use it as a candidate header on CHECKSUM FAILURE.
             */
            protocol_parser_reset(parser);
            if (byte == PROTOCOL_HEADER_0)
            {
                parser->state = PROTOCOL_STATE_WAIT_HEADER_1;
            }

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


/* One raw six-axis payload decoder for UART feedback and Stage5 request.
 * Out-of-range canonical angles are rejected before modifying outputs.
 * Payload bytes and signed interpretation are identical for both commands. */
static robot_status_t decode_joint_payload(
    const uint8_t *bytes,
    robot_joint_angles_t *out)
{
    robot_joint_angles_t decoded={{0}};
    uint32_t i;
    if(bytes==NULL || out==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    for(i=0U;i<ROBOT_JOINT_COUNT;i++) {
        uint16_t raw=(uint16_t)bytes[2U*i] |
            ((uint16_t)bytes[2U*i+1U]<<8U);
        decoded.value[i]=(robot_joint_angle_t)raw;
        if(decoded.value[i] < -ROBOT_JOINT_HALF_TURN_RAW ||
           decoded.value[i] >= ROBOT_JOINT_HALF_TURN_RAW)
            return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }
    *out=decoded;
    return ROBOT_STATUS_OK;
}

robot_status_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    robot_joint_angles_t *joints)
{
    if(frame==NULL || joints==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(frame->command!=CMD_JOINT_STATE)
        return ROBOT_STATUS_ERROR_INVALID_COMMAND;
    if(frame->length!=PROTOCOL_JOINT_PAYLOAD_LEN)
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    return decode_joint_payload(frame->payload,joints);
}


/* Stage5 simulation move requests are never decoded as feedback. Only
 * canonical signed six-axis values use the existing 0.01-degree encoding.
 * Payload shares the authoritative constants, not a second angle scale. */
robot_status_t protocol_parse_joint_move_request(
    const protocol_frame_t *frame,
    uint8_t *sequence,
    robot_joint_angles_t *canonical_goal)
{
    robot_joint_angles_t decoded={{0}};
    robot_status_t status;
    if(frame==NULL || sequence==NULL || canonical_goal==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(frame->command!=CMD_REQUEST_JOINT_MOVE)
        return ROBOT_STATUS_ERROR_INVALID_COMMAND;
    if(frame->length!=PROTOCOL_MOVE_REQUEST_PAYLOAD_LEN)
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    status=decode_joint_payload(&frame->payload[1U],&decoded);
    if(status!=ROBOT_STATUS_OK) return status;
    *sequence=frame->payload[0];
    *canonical_goal=decoded;
    return ROBOT_STATUS_OK;
}

robot_status_t protocol_build_joint_move_ack_frame(
    uint8_t sequence,
    robot_status_t status,
    uint8_t frame[PROTOCOL_MOVE_ACK_FRAME_LEN])
{
    uint8_t payload[PROTOCOL_MOVE_ACK_PAYLOAD_LEN];
    if(frame==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(status<INT8_MIN || status>INT8_MAX)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    payload[0]=sequence;
    payload[1]=(uint8_t)((int8_t)status);
    return protocol_build_frame(CMD_JOINT_MOVE_ACK,payload,
        PROTOCOL_MOVE_ACK_PAYLOAD_LEN,frame);
}

/* QEMU Stage5 preview wire encoding. Use the existing frame builder and
 * enforce finite IEEE754 little-endian floats. This is NOT an actuator path.
 * No copy of C APF behavior, obstacles or tuning constants lives here. */
robot_status_t protocol_parse_apf_step_request(
    const protocol_frame_t *frame, protocol_apf_step_request_t *out)
{
    protocol_apf_step_request_t next={0};
    int16_t *fields[3]={next.goal_offset_mm,
        next.obstacle_min_offset_mm,next.obstacle_max_offset_mm};
    size_t i;
    _Static_assert(sizeof(float)==4U, "APF preview wire needs IEEE754 float32");
    if(frame==NULL || out==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(frame->command!=CMD_REQUEST_APF_STEP)
        return ROBOT_STATUS_ERROR_INVALID_COMMAND;
    if(frame->length!=PROTOCOL_APF_STEP_REQUEST_PAYLOAD_LEN)
        return ROBOT_STATUS_ERROR_INVALID_LENGTH;
    next.sequence=frame->payload[0];
    for(i=0U;i<9U;i++) {
        size_t pos=1U+2U*i;
        uint16_t raw=(uint16_t)frame->payload[pos] |
            ((uint16_t)frame->payload[pos+1U]<<8U);
        fields[i/3U][i%3U]=(int16_t)raw;
    }
    for(i=0U;i<6U;i++) {
        size_t pos=19U+4U*i;
        uint32_t raw=(uint32_t)frame->payload[pos] |
            ((uint32_t)frame->payload[pos+1U]<<8U) |
            ((uint32_t)frame->payload[pos+2U]<<16U) |
            ((uint32_t)frame->payload[pos+3U]<<24U);
        memcpy(&next.apf_parameters[i],&raw,sizeof(raw));
        if(!isfinite(next.apf_parameters[i]))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }
    *out=next;
    return ROBOT_STATUS_OK;
}

robot_status_t protocol_build_apf_step_response(
    uint8_t sequence, robot_status_t status,
    const float next_mm[3],
    uint8_t frame[PROTOCOL_APF_STEP_RESPONSE_FRAME_LEN])
{
    uint8_t payload[PROTOCOL_APF_STEP_RESPONSE_PAYLOAD_LEN]={0};
    size_t i;
    _Static_assert(sizeof(float)==4U, "APF preview wire needs IEEE754 float32");
    if(frame==NULL || (status==ROBOT_STATUS_OK && next_mm==NULL))
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(status<INT8_MIN || status>INT8_MAX)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    payload[0]=sequence;
    payload[1]=(uint8_t)((int8_t)status);
    if(status==ROBOT_STATUS_OK) {
        for(i=0U;i<3U;i++) {
            uint32_t raw;
            size_t pos=2U+4U*i;
            if(!isfinite(next_mm[i]))
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
            memcpy(&raw,&next_mm[i],sizeof(raw));
            payload[pos]=(uint8_t)raw;
            payload[pos+1U]=(uint8_t)(raw>>8U);
            payload[pos+2U]=(uint8_t)(raw>>16U);
            payload[pos+3U]=(uint8_t)(raw>>24U);
        }
    }
    return protocol_build_frame(CMD_APF_STEP_RESPONSE,payload,
        PROTOCOL_APF_STEP_RESPONSE_PAYLOAD_LEN,frame);
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
