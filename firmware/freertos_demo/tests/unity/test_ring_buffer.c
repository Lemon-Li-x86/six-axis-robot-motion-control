/*
 * 文件：test_ring_buffer.c
 *
 * 用途：
 * 使用 Unity 验证 Ring Buffer 的：
 *
 * 1. 初始化状态；
 * 2. FIFO 顺序；
 * 3. 容量边界；
 * 4. Index Wraparound。
 */

#include <stdint.h>

#include "unity.h"

#include "ring_buffer.h"


/* =========================================================
 * Initialization Tests
 * ========================================================= */

/**
 * @brief 验证 Ring Buffer 初始化后为空，
 *        且读取空 Buffer 返回失败。
 */
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


/* =========================================================
 * FIFO Tests
 * ========================================================= */

/**
 * @brief 验证 Ring Buffer 保持
 *        First-In First-Out 顺序。
 */
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


/* =========================================================
 * Capacity Tests
 * ========================================================= */

/**
 * @brief 验证 Ring Buffer 的实际可用容量
 *        和 Full 边界行为。
 */
void test_ring_buffer_capacity_boundary(void)
{
    ring_buffer_t buffer;
    uint32_t i;

    ring_buffer_init(
        &buffer
    );

    /*
     * RING_BUFFER_CAPACITY = 128 Byte。
     *
     * head == tail 用于表示 Empty，
     * 因此必须保留一个 Slot：
     *
     * Usable Capacity = 127 Byte。
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

    /*
     * Buffer 已满后，
     * 新 Byte 必须被拒绝。
     */
    TEST_ASSERT_EQUAL_UINT8(
        0U,
        ring_buffer_write(
            &buffer,
            0xAAU
        )
    );
}


/* =========================================================
 * Wraparound Tests
 * ========================================================= */

/**
 * @brief 验证 head / tail 跨越数组末尾后
 *        FIFO 顺序仍然正确。
 */
void test_ring_buffer_wraparound(void)
{
    ring_buffer_t buffer;

    uint32_t i;
    uint8_t value;

    ring_buffer_init(
        &buffer
    );

    /*
     * Step 1：
     * 写入 0..99，共 100 Byte。
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
     * Step 2：
     * 读取 0..79，共 80 Byte。
     *
     * Buffer 中剩余：
     * 80..99。
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
     * Step 3：
     * 再写入 100..199，共 100 Byte。
     *
     * 此过程中 head 必须跨越
     * 数组末尾并回绕到开头。
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
     * Step 4：
     * 当前逻辑内容应连续为：
     *
     * 80..199。
     *
     * 数组物理 Wraparound
     * 不得破坏 FIFO 顺序。
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