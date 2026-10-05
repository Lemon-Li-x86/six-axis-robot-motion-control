/*
 * 文件：test_protocol.c
 *
 * 用途：
 * 使用 Unity 验证 Protocol 模块的：
 *
 * 1. 正常 Frame 解析；
 * 2. Checksum 异常恢复；
 * 3. Payload Length 边界；
 * 4. Runtime Parameter 解析；
 * 5. Parameter ACK 构造；
 * 6. Diagnostics 请求；
 * 7. Signed Joint State 编解码。
 */

#include <stdint.h>

#include "unity.h"

#include "protocol.h"
#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * Test Helpers
 * ========================================================= */

/**
 * @brief 计算完整测试 Frame 的 Protocol Checksum。
 *
 * @param[in] frame
 * 完整协议帧。
 *
 * @param[in] length
 * Frame 总长度，单位 Byte。
 *
 * @return
 * Command + Length + Payload
 * 的低 8 bit 累加结果。
 *
 * @note
 * Header 和最后一个 Checksum Byte
 * 不参与 Checksum 计算。
 */
static uint8_t calculate_checksum(
    const uint8_t *frame,
    uint32_t length
)
{
    uint8_t checksum = 0U;
    uint32_t i;

    /*
     * Index 0~1 为 Header，
     * length - 1 为 Checksum。
     */
    for (i = 2U; i < length - 1U; i++)
    {
        checksum =
            (uint8_t)(
                checksum + frame[i]
            );
    }

    return checksum;
}


/**
 * @brief 将完整 Byte Stream 逐字节送入 Protocol Parser。
 *
 * @param[in,out] parser
 * Parser State。
 *
 * @param[in] data
 * 输入 Byte Stream。
 *
 * @param[in] length
 * 输入长度，单位 Byte。
 *
 * @param[out] output
 * Parser 输出 Frame。
 *
 * @return
 * 最后一个输入 Byte 处理完成后的
 * protocol_parser_process_byte() 返回值。
 *
 * @note
 * 当前测试输入均以一个完整协议帧结束，
 * 因此合法 Frame 应在最后一个 Byte
 * 返回 1。
 */
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


/* =========================================================
 * Parser Tests
 * ========================================================= */

/**
 * @brief 验证 Parser 可以接受合法完整 Frame。
 */
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


/**
 * @brief 验证错误 Checksum 被拒绝，
 *        且 Parser 可以继续解析后续合法 Frame。
 */
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

    /*
     * 翻转 Checksum 最低位，
     * 人为制造校验错误。
     */
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
     * Parser 必须能从错误 Frame 恢复，
     * 而不是永久停留在错误 State。
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


/**
 * @brief 验证超长 Payload 被拒绝，
 *        且 Parser 可以重新同步。
 */
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

    /*
     * Length 故意设置为：
     *
     * PROTOCOL_MAX_PAYLOAD_LEN + 1
     *
     * 验证 Parser 的 Payload Length 上界。
     */
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


/* =========================================================
 * Parameter Tests
 * ========================================================= */

/**
 * @brief 验证 SET_PARAMETER Payload 的 Little Endian 解析。
 */
void test_protocol_parse_set_parameter(void)
{
    /*
     * 0x000000C8 = 200。
     *
     * Wire Format 使用 Little Endian：
     *
     * C8 00 00 00
     */
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


/**
 * @brief 验证 SET_PARAMETER 非法 Payload Length 被拒绝。
 */
void test_protocol_parse_set_parameter_rejects_invalid_length(void)
{
    /*
     * SET_PARAMETER 正确 Payload Length 为 5 Byte。
     * 此处使用 4 Byte 验证边界检查。
     */
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


/**
 * @brief 验证 PARAMETER_ACK Frame 构造结果。
 */
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
     * effective_value = 1000
     *
     * 1000 = 0x000003E8
     *
     * Little Endian Wire Format：
     *
     * E8 03 00 00
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


/* =========================================================
 * Diagnostics Tests
 * ========================================================= */

/**
 * @brief 验证空 Diagnostics Payload
 *        使用默认 RX Drop Selector。
 */
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


/* =========================================================
 * Joint State Tests
 * ========================================================= */

/**
 * @brief 验证带符号 int16_t 关节角
 *        可以正确从 Little Endian Payload 恢复。
 */
void test_protocol_parse_joint_state_signed_values(void)
{
    protocol_frame_t frame =
    {
        .command = CMD_JOINT_STATE,
        .length = PROTOCOL_JOINT_PAYLOAD_LEN
    };

    robot_joint_angles_t joints;

    /*
     * 单位：
     * 0.01 degree。
     *
     * 覆盖：
     *
     * -180.00°
     * -90.00°
     * -0.01°
     * 0°
     * +90.00°
     * +179.99°
     */
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