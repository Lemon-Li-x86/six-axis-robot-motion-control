#include <stdint.h>

#include "unity.h"

#include "ring_buffer.h"


void test_ring_buffer_initially_empty(void)
{
    ring_buffer_t buffer;
    uint8_t value = 0U;

    ring_buffer_init(
        &buffer
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_is_empty(
            &buffer
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        ring_buffer_is_full(
            &buffer
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        ring_buffer_read(
            &buffer,
            &value
        )
    );
}


void test_ring_buffer_preserves_fifo_order(void)
{
    ring_buffer_t buffer;
    uint8_t value;

    ring_buffer_init(
        &buffer
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_write(
            &buffer,
            0x11U
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_write(
            &buffer,
            0x22U
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_write(
            &buffer,
            0x33U
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_read(
            &buffer,
            &value
        )
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x11U,
        value
    );

    ring_buffer_read(
        &buffer,
        &value
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x22U,
        value
    );

    ring_buffer_read(
        &buffer,
        &value
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x33U,
        value
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_is_empty(
            &buffer
        )
    );
}


void test_ring_buffer_capacity_boundary(void)
{
    ring_buffer_t buffer;
    uint32_t i;

    ring_buffer_init(
        &buffer
    );

    /*
     * 物理容量 128，
     * 可用容量 127。
     */
    for (
        i = 0U;
        i < RING_BUFFER_CAPACITY - 1U;
        i++
    )
    {
        TEST_ASSERT_EQUAL_UINT8(
            1U,
            ring_buffer_write(
                &buffer,
                (uint8_t)i
            )
        );
    }

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_is_full(
            &buffer
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        ring_buffer_write(
            &buffer,
            0xAAU
        )
    );
}


void test_ring_buffer_wraparound(void)
{
    ring_buffer_t buffer;

    uint32_t i;
    uint8_t value;

    ring_buffer_init(
        &buffer
    );

    /*
     * 先写 100 Byte。
     */
    for (i = 0U; i < 100U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(
            1U,
            ring_buffer_write(
                &buffer,
                (uint8_t)i
            )
        );
    }

    /*
     * 读掉前 80 Byte。
     */
    for (i = 0U; i < 80U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(
            1U,
            ring_buffer_read(
                &buffer,
                &value
            )
        );

        TEST_ASSERT_EQUAL_UINT8(
            (uint8_t)i,
            value
        );
    }

    /*
     * 再写 100 Byte，
     * 此时 head 会跨越数组末尾。
     */
    for (i = 100U; i < 200U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(
            1U,
            ring_buffer_write(
                &buffer,
                (uint8_t)i
            )
        );
    }

    /*
     * 剩余内容应为 80..199，
     * 顺序不能因为 wrap 被破坏。
     */
    for (i = 80U; i < 200U; i++)
    {
        TEST_ASSERT_EQUAL_UINT8(
            1U,
            ring_buffer_read(
                &buffer,
                &value
            )
        );

        TEST_ASSERT_EQUAL_UINT8(
            (uint8_t)i,
            value
        );
    }

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        ring_buffer_is_empty(
            &buffer
        )
    );
}