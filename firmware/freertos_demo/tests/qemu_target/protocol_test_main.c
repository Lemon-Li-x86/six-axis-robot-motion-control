
/*
 * 文件：protocol_test_main.c
 *
 * 用途：
 * 在 QEMU Cortex-M4 上执行协议与 Ring Buffer 测试。
 *
 * 不依赖：
 * - Host Unity
 * - Windows/Linux C Runtime
 * - FreeRTOS Scheduler
 * - UART 输出
 *
 * 使用 GDB 读取最终测试结果。
 */

#include <stdint.h>

#include "protocol.h"
#include "ring_buffer.h"


#define TARGET_TEST_COUNT 8U
#define TARGET_TEST_MAGIC 0x51454D55UL

/*
 * 各项结果：
 * 1 = PASS
 * 0 = FAIL
 *
 * volatile 保证调试结果保留在内存中。
 */
typedef struct
{
    uint32_t magic;
    uint32_t total;
    uint32_t passed;
    uint32_t failed;
    uint32_t results[TARGET_TEST_COUNT];
} target_test_report_t;

volatile target_test_report_t
    g_qemu_target_test_report;


/* =========================================================
 * Helper: Feed Protocol Bytes
 * ========================================================= */

/*
 * 将完整字节流逐个送入真实协议解析器。
 *
 * 返回成功解析出的完整帧数量。
 */
static uint32_t feed_bytes(
    protocol_parser_t *parser,
    const uint8_t *bytes,
    uint32_t length,
    protocol_frame_t *output
)
{
    uint32_t i;
    uint32_t complete_count = 0U;

    for (i = 0U; i < length; i++)
    {
        if (
            protocol_parser_process_byte(
                parser,
                bytes[i],
                output
            ) == 1U
        )
        {
            complete_count++;
        }
    }

    return complete_count;
}


/* =========================================================
 * Test 1: Ring Buffer Initial State
 * ========================================================= */

static uint32_t test_ring_initial_state(void)
{
    ring_buffer_t buffer;
    uint8_t value = 0U;

    ring_buffer_init(&buffer);

    if (!ring_buffer_is_empty(&buffer))
    {
        return 0U;
    }

    if (ring_buffer_is_full(&buffer))
    {
        return 0U;
    }

    if (ring_buffer_read(&buffer, &value) != 0U)
    {
        return 0U;
    }

    return 1U;
}


/* =========================================================
 * Test 2: Ring Buffer Capacity Boundary
 * ========================================================= */

static uint32_t test_ring_capacity(void)
{
    ring_buffer_t buffer;
    uint32_t i;
    uint8_t value;

    ring_buffer_init(&buffer);

    /*
     * 物理容量 128 Byte。
     * 采用空一格算法，实际有效容量为 127。
     */
    for (
        i = 0U;
        i < RING_BUFFER_CAPACITY - 1U;
        i++
    )
    {
        if (
            ring_buffer_write(&buffer, (uint8_t)i)
            != 1U
        )
        {
            return 0U;
        }
    }

    if (!ring_buffer_is_full(&buffer))
    {
        return 0U;
    }

    /*
     * 缓冲区已满时，额外输入应被拒绝。
     */
    if (ring_buffer_write(&buffer, 0xAAU) != 0U)
    {
        return 0U;
    }

    /*
     * 检查满缓冲区中的 FIFO 数据没有被破坏。
     */
    for (
        i = 0U;
        i < RING_BUFFER_CAPACITY - 1U;
        i++
    )
    {
        if (ring_buffer_read(&buffer, &value) != 1U)
        {
            return 0U;
        }

        if (value != (uint8_t)i)
        {
            return 0U;
        }
    }

    return ring_buffer_is_empty(&buffer);
}


/* =========================================================
 * Test 3: Ring Buffer Wraparound
 * ========================================================= */

static uint32_t test_ring_wraparound(void)
{
    ring_buffer_t buffer;
    uint32_t i;
    uint8_t value;

    ring_buffer_init(&buffer);

    for (i = 0U; i < 100U; i++)
    {
        if (!ring_buffer_write(&buffer, (uint8_t)i))
        {
            return 0U;
        }
    }

    for (i = 0U; i < 80U; i++)
    {
        if (!ring_buffer_read(&buffer, &value))
        {
            return 0U;
        }

        if (value != (uint8_t)i)
        {
            return 0U;
        }
    }

    /*
     * 再写入 100 Byte，触发 head 回绕。
     */
    for (i = 100U; i < 200U; i++)
    {
        if (!ring_buffer_write(&buffer, (uint8_t)i))
        {
            return 0U;
        }
    }

    for (i = 80U; i < 200U; i++)
    {
        if (!ring_buffer_read(&buffer, &value))
        {
            return 0U;
        }

        if (value != (uint8_t)i)
        {
            return 0U;
        }
    }

    return ring_buffer_is_empty(&buffer);
}


/* =========================================================
 * Test 4: Protocol Joint Frame
 * ========================================================= */

static uint32_t test_protocol_joint_frame(void)
{
    const robot_joint_angles_t joints =
    {
        .value = {
            -18000, -9000, -1, 0, 9000, 17999
        }
    };

    uint8_t bytes[PROTOCOL_JOINT_FRAME_LEN];

    protocol_parser_t parser;
    protocol_frame_t output;

    uint32_t i;

    if (
        protocol_build_joint_target_frame(
            &joints,
            bytes
        ) != ROBOT_STATUS_OK
    )
    {
        return 0U;
    }

    protocol_parser_init(&parser);

    if (
        feed_bytes(
            &parser,
            bytes,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        ) != 1U
    )
    {
        return 0U;
    }

    if (
        output.command != CMD_SET_JOINT_TARGETS
        ||
        output.length != PROTOCOL_JOINT_PAYLOAD_LEN
    )
    {
        return 0U;
    }

    /*
     * 检查 int16_t 关节角使用 Little Endian。
     */
    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        uint16_t raw = (uint16_t)joints.value[i];

        if (
            output.payload[i * 2U]
            != (uint8_t)(raw & 0xFFU)
        )
        {
            return 0U;
        }

        if (
            output.payload[i * 2U + 1U]
            != (uint8_t)((raw >> 8U) & 0xFFU)
        )
        {
            return 0U;
        }
    }

    return 1U;
}


/* =========================================================
 * Test 5: Checksum Error Recovery
 * ========================================================= */

static uint32_t test_protocol_checksum_recovery(void)
{
    const robot_joint_angles_t joints =
    {
        .value = {100, 200, 300, 400, 500, 600}
    };

    uint8_t bytes[PROTOCOL_JOINT_FRAME_LEN];

    protocol_parser_t parser;
    protocol_frame_t output;

    protocol_parser_init(&parser);

    if (
        protocol_build_joint_target_frame(
            &joints,
            bytes
        ) != ROBOT_STATUS_OK
    )
    {
        return 0U;
    }

    /*
     * 故意破坏校验和。
     */
    bytes[PROTOCOL_JOINT_FRAME_LEN - 1U] ^= 0x01U;

    if (
        feed_bytes(
            &parser,
            bytes,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        ) != 0U
    )
    {
        return 0U;
    }

    /*
     * 恢复合法帧，检查 Parser 能否重新同步。
     */
    if (
        protocol_build_joint_target_frame(
            &joints,
            bytes
        ) != ROBOT_STATUS_OK
    )
    {
        return 0U;
    }

    return (
        feed_bytes(
            &parser,
            bytes,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        ) == 1U
    );
}


/* =========================================================
 * Test 6: Oversized Payload Recovery
 * ========================================================= */

static uint32_t test_protocol_length_boundary(void)
{
    protocol_parser_t parser;
    protocol_frame_t output;

    const uint8_t invalid_header[] =
    {
        PROTOCOL_HEADER_0,
        PROTOCOL_HEADER_1,
        CMD_SET_JOINT_TARGETS,
        (uint8_t)(PROTOCOL_MAX_PAYLOAD_LEN + 1U)
    };

    const robot_joint_angles_t joints =
    {
        .value = {1, 2, 3, 4, 5, 6}
    };

    uint8_t valid_frame[PROTOCOL_JOINT_FRAME_LEN];

    protocol_parser_init(&parser);

    /*
     * 超出 64 Byte 上限的 Payload
     * 应在解析 Length 时被直接拒绝。
     */
    if (
        feed_bytes(
            &parser,
            invalid_header,
            sizeof(invalid_header),
            &output
        ) != 0U
    )
    {
        return 0U;
    }

    if (
        protocol_build_joint_target_frame(
            &joints,
            valid_frame
        ) != ROBOT_STATUS_OK
    )
    {
        return 0U;
    }

    return (
        feed_bytes(
            &parser,
            valid_frame,
            PROTOCOL_JOINT_FRAME_LEN,
            &output
        ) == 1U
    );
}

/* Test 8: Partial frame timeout reset and valid-frame recovery. */
static uint32_t test_protocol_partial_frame_timeout(void)
{
    const robot_joint_angles_t joints =
    {
        .value = {11, 22, 33, 44, 55, 66}
    };
    uint8_t bytes[PROTOCOL_JOINT_FRAME_LEN];
    protocol_parser_t parser;
    protocol_frame_t output;

    protocol_parser_init(&parser);
    if (protocol_build_joint_target_frame(&joints, bytes)
        != ROBOT_STATUS_OK)
    {
        return 0U;
    }
    if (feed_bytes(&parser, bytes, 7U, &output) != 0U ||
        parser.state == PROTOCOL_STATE_WAIT_HEADER_0)
    {
        return 0U;
    }
    if (protocol_parser_expire_partial_frame(&parser, 4U, 5U) != 0U ||
        protocol_parser_expire_partial_frame(&parser, 5U, 5U) != 1U ||
        parser.state != PROTOCOL_STATE_WAIT_HEADER_0)
    {
        return 0U;
    }
    return feed_bytes(&parser, bytes, PROTOCOL_JOINT_FRAME_LEN, &output)
        == 1U;
}


/* =========================================================
 * Test 7: Parameter Little Endian
 * ========================================================= */

static uint32_t test_parameter_little_endian(void)
{
    const protocol_frame_t frame =
    {
        .command = CMD_SET_PARAMETER,
        .length = PROTOCOL_PARAMETER_REQUEST_PAYLOAD_LEN,
        .payload = {
            PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS,
            0x78U,
            0x56U,
            0x34U,
            0x12U
        }
    };

    uint8_t parameter_id = 0U;
    uint32_t value = 0U;

    if (
        protocol_parse_set_parameter(
            &frame,
            &parameter_id,
            &value
        ) != ROBOT_STATUS_OK
    )
    {
        return 0U;
    }

    if (
        parameter_id
        != PROTOCOL_PARAMETER_PROTOCOL_TX_PERIOD_MS
    )
    {
        return 0U;
    }

    return (value == 0x12345678UL);
}


/* =========================================================
 * Test Completion
 * ========================================================= */

/*
 * GDB 断点位置。
 *
 * 完成后停在此函数，查看全局测试报告。
 *
 * 不使用 UART 输出，
 * 因此不会干扰生产固件的通信协议。
 */
__attribute__((noinline))
void qemu_target_test_done(void)
{
    while (1)
    {
        __asm volatile ("nop");
    }
}


/* =========================================================
 * Test Entry
 * ========================================================= */

int main(void)
{
    uint32_t i;

    g_qemu_target_test_report.magic =
        TARGET_TEST_MAGIC;

    g_qemu_target_test_report.total =
        TARGET_TEST_COUNT;

    g_qemu_target_test_report.passed = 0U;
    g_qemu_target_test_report.failed = 0U;

    g_qemu_target_test_report.results[0] =
        test_ring_initial_state();

    g_qemu_target_test_report.results[1] =
        test_ring_capacity();

    g_qemu_target_test_report.results[2] =
        test_ring_wraparound();

    g_qemu_target_test_report.results[3] =
        test_protocol_joint_frame();

    g_qemu_target_test_report.results[4] =
        test_protocol_checksum_recovery();

    g_qemu_target_test_report.results[5] =
        test_protocol_length_boundary();

    g_qemu_target_test_report.results[6] =
        test_parameter_little_endian();

    g_qemu_target_test_report.results[7] =
        test_protocol_partial_frame_timeout();

    /*
     * 统计全部测试，不提前退出。
     *
     * 某一项失败时，其他测试仍可以执行，
     * 方便一次性获得完整结果。
     */
    for (i = 0U; i < TARGET_TEST_COUNT; i++)
    {
        if (g_qemu_target_test_report.results[i] == 1U)
        {
            g_qemu_target_test_report.passed++;
        }
        else
        {
            g_qemu_target_test_report.failed++;
        }
    }

    qemu_target_test_done();

    return 0;
}
