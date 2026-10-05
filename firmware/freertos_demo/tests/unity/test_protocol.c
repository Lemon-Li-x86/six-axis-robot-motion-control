#include <stdint.h>

#include "unity.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"


static uint8_t calculate_checksum(
    const uint8_t *frame,
    uint32_t length
)
{
    uint8_t checksum = 0U;
    uint32_t i;

    for (i = 2U; i < length - 1U; i++)
    {
        checksum =
            (uint8_t)(
                checksum + frame[i]
            );
    }

    return checksum;
}


static uint8_t feed_frame(
    protocol_parser_t *parser,
    const uint8_t *data,
    uint32_t length,
    protocol_frame_t *output
)
{
    uint32_t i;
    uint8_t complete = 0U;

    for (i = 0U; i < length; i++)
    {
        complete =
            protocol_parser_process_byte(
                parser,
                data[i],
                output
            );
    }

    return complete;
}


void test_protocol_parser_accepts_valid_frame(void)
{
    protocol_parser_t parser;
    protocol_frame_t output;

    robot_joint_angles_t joints =
    {
        .value =
        {
            100,
            -200,
            300,
            -400,
            500,
            -600
        }
    };

    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];

    uint8_t complete;

    protocol_parser_init(
        &parser
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_build_joint_target_frame(
            &joints,
            frame
        )
    );

    complete =
        feed_frame(
            &parser,
            frame,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        complete
    );

    TEST_ASSERT_EQUAL_UINT8(
        CMD_SET_JOINT_TARGETS,
        output.command
    );

    TEST_ASSERT_EQUAL_UINT8(
        PROTOCOL_JOINT_PAYLOAD_LEN,
        output.length
    );
}


void test_protocol_parser_rejects_bad_checksum_and_recovers(void)
{
    protocol_parser_t parser;
    protocol_frame_t output;

    robot_joint_angles_t joints =
    {
        .value =
        {
            1000,
            2000,
            3000,
            4000,
            5000,
            6000
        }
    };

    uint8_t frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];

    uint8_t complete;

    protocol_parser_init(
        &parser
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_build_joint_target_frame(
            &joints,
            frame
        )
    );

    frame[
        PROTOCOL_JOINT_FRAME_LEN - 1U
    ] ^= 0x01U;

    complete =
        feed_frame(
            &parser,
            frame,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        complete
    );

    /*
     * Parser 必须能从错误帧恢复。
     */
    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_build_joint_target_frame(
            &joints,
            frame
        )
    );

    complete =
        feed_frame(
            &parser,
            frame,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        complete
    );
}


void test_protocol_parser_rejects_oversized_payload_and_recovers(void)
{
    protocol_parser_t parser;
    protocol_frame_t output;

    robot_joint_angles_t joints =
    {
        .value =
        {
            1,
            2,
            3,
            4,
            5,
            6
        }
    };

    uint8_t invalid_prefix[] =
    {
        PROTOCOL_HEADER_0,
        PROTOCOL_HEADER_1,
        CMD_SET_JOINT_TARGETS,
        PROTOCOL_MAX_PAYLOAD_LEN + 1U
    };

    uint8_t valid_frame[
        PROTOCOL_JOINT_FRAME_LEN
    ];

    uint8_t complete;

    protocol_parser_init(
        &parser
    );

    complete =
        feed_frame(
            &parser,
            invalid_prefix,
            sizeof(invalid_prefix),
            &output
        );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        complete
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_build_joint_target_frame(
            &joints,
            valid_frame
        )
    );

    complete =
        feed_frame(
            &parser,
            valid_frame,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        complete
    );
}


void test_protocol_parse_set_parameter(void)
{
    protocol_frame_t frame =
    {
        .command = CMD_SET_PARAMETER,
        .length =
            PROTOCOL_PARAMETER_REQUEST_PAYLOAD_LEN,
        .payload =
        {
            PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS,
            0xC8U,
            0x00U,
            0x00U,
            0x00U
        }
    };

    uint8_t parameter_id = 0U;
    uint32_t value = 0U;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_parse_set_parameter(
            &frame,
            &parameter_id,
            &value
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS,
        parameter_id
    );

    TEST_ASSERT_EQUAL_UINT32(
        200U,
        value
    );
}


void test_protocol_parse_set_parameter_rejects_invalid_length(void)
{
    protocol_frame_t frame =
    {
        .command = CMD_SET_PARAMETER,
        .length = 4U
    };

    uint8_t parameter_id = 0U;
    uint32_t value = 0U;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_INVALID_LENGTH,
        protocol_parse_set_parameter(
            &frame,
            &parameter_id,
            &value
        )
    );
}


void test_protocol_build_parameter_ack(void)
{
    uint8_t frame[
        PROTOCOL_PARAMETER_ACK_FRAME_LEN
    ];

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_build_parameter_ack_frame(
            PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS,
            ROBOT_STATUS_ERROR_OUT_OF_RANGE,
            1000U,
            frame
        )
    );

    TEST_ASSERT_EQUAL_HEX8(
        PROTOCOL_HEADER_0,
        frame[0]
    );

    TEST_ASSERT_EQUAL_HEX8(
        PROTOCOL_HEADER_1,
        frame[1]
    );

    TEST_ASSERT_EQUAL_HEX8(
        CMD_PARAMETER_ACK,
        frame[2]
    );

    TEST_ASSERT_EQUAL_UINT8(
        PROTOCOL_PARAMETER_ACK_PAYLOAD_LEN,
        frame[3]
    );

    TEST_ASSERT_EQUAL_UINT8(
        PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS,
        frame[4]
    );

    TEST_ASSERT_EQUAL_INT8(
        ROBOT_STATUS_ERROR_OUT_OF_RANGE,
        (int8_t)frame[5]
    );

    /*
     * 1000 = 0x000003E8
     * Little Endian。
     */
    TEST_ASSERT_EQUAL_HEX8(
        0xE8U,
        frame[6]
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x03U,
        frame[7]
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x00U,
        frame[8]
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x00U,
        frame[9]
    );

    TEST_ASSERT_EQUAL_HEX8(
        calculate_checksum(
            frame,
            PROTOCOL_PARAMETER_ACK_FRAME_LEN
        ),
        frame[
            PROTOCOL_PARAMETER_ACK_FRAME_LEN - 1U
        ]
    );
}


void test_protocol_parse_diagnostics_default_selector(void)
{
    protocol_frame_t frame =
    {
        .command = CMD_GET_DIAGNOSTICS,
        .length = 0U
    };

    uint8_t selector = 0xFFU;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_parse_diagnostics_request(
            &frame,
            &selector
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        DIAGNOSTICS_METRIC_RX_DROP_COUNT,
        selector
    );
}


void test_protocol_parse_joint_state_signed_values(void)
{
    protocol_frame_t frame =
    {
        .command = CMD_JOINT_STATE,
        .length = PROTOCOL_JOINT_PAYLOAD_LEN
    };

    robot_joint_angles_t joints;

    const int16_t expected[
        ROBOT_JOINT_COUNT
    ] =
    {
        -18000,
        -9000,
        -1,
        0,
        9000,
        17999
    };

    uint32_t i;

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        uint16_t raw =
            (uint16_t)expected[i];

        frame.payload[i * 2U] =
            (uint8_t)(raw & 0xFFU);

        frame.payload[i * 2U + 1U] =
            (uint8_t)(
                (raw >> 8U)
                & 0xFFU
            );
    }

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        protocol_parse_joint_state(
            &frame,
            &joints
        )
    );

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        TEST_ASSERT_EQUAL_INT16(
            expected[i],
            joints.value[i]
        );
    }
}