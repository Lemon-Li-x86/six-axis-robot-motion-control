/*
 * 文件：test_uart_driver.c
 *
 * 用途：
 * 使用 Unity 和 Host Register Backend
 * 验证 UART Driver 的：
 *
 * 1. 初始化；
 * 2. 参数检查；
 * 3. Interrupt Driven TX；
 * 4. TX Buffer 容量边界；
 * 5. RX Interrupt；
 * 6. RX Drop Counter。
 */

#include <stdint.h>

#include "unity.h"

#include "uart_driver.h"
#include "uart_driver_test_backend.h"
#include "error_code.h"


/* =========================================================
 * Test Register Bit Definitions
 * ========================================================= */

/*
 * 以下 Bit 与 uart_driver.c 中的
 * CMSDK UART CTRL / Interrupt 定义一致。
 *
 * Test Code 独立定义这些值，
 * 避免依赖 Driver 私有宏。
 */

#define TEST_UART_CTRL_TX_ENABLE \
    (1UL << 0)

#define TEST_UART_CTRL_RX_ENABLE \
    (1UL << 1)

#define TEST_UART_CTRL_TX_INTERRUPT_ENABLE \
    (1UL << 2)

#define TEST_UART_CTRL_RX_INTERRUPT_ENABLE \
    (1UL << 3)

#define TEST_UART_INTERRUPT_TX \
    (1UL << 0)

#define TEST_UART_INTERRUPT_RX \
    (1UL << 1)


/* =========================================================
 * Test State
 * ========================================================= */

static uint32_t rx_callback_count = 0U;


/* =========================================================
 * Test Helpers
 * ========================================================= */

/**
 * @brief 模拟 UART RX Event Callback。
 *
 * 每执行一次将 Callback Counter 加 1，
 * 用于验证 RX ISR 是否触发上层通知。
 */
static void test_rx_callback(void)
{
    rx_callback_count++;
}


/**
 * @brief 为每个 UART Unit Test
 *        重置 Host Backend 并初始化 Driver。
 */
static void initialize_uart(void)
{
    uart_driver_test_backend_reset();

    rx_callback_count = 0U;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        uart_driver_init()
    );
}


/* =========================================================
 * Initialization Tests
 * ========================================================= */

/**
 * @brief 验证 UART Driver 初始化后的
 *        UART Register 和内部状态。
 */
void test_uart_init_configures_hardware(void)
{
    initialize_uart();

    /*
     * 当前 CMSDK APB UART 配置：
     *
     * BAUDDIV = 16。
     *
     * 该值满足 CMSDK UART
     * BAUDDIV >= 16 的硬件要求。
     */
    TEST_ASSERT_EQUAL_UINT32(
        16U,
        uart_driver_test_uart0_bauddiv
    );

    TEST_ASSERT_BITS_HIGH(
        TEST_UART_CTRL_TX_ENABLE,
        uart_driver_test_uart0_ctrl
    );

    TEST_ASSERT_BITS_HIGH(
        TEST_UART_CTRL_RX_ENABLE,
        uart_driver_test_uart0_ctrl
    );

    /*
     * 初始化阶段只开启 UART TX / RX 功能，
     * RX / TX Interrupt 尚未开启。
     */
    TEST_ASSERT_BITS_LOW(
        TEST_UART_CTRL_TX_INTERRUPT_ENABLE,
        uart_driver_test_uart0_ctrl
    );

    TEST_ASSERT_BITS_LOW(
        TEST_UART_CTRL_RX_INTERRUPT_ENABLE,
        uart_driver_test_uart0_ctrl
    );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        uart_driver_is_tx_busy()
    );

    TEST_ASSERT_EQUAL_UINT32(
        0U,
        uart_driver_get_rx_drop_count()
    );
}


/* =========================================================
 * Argument Validation Tests
 * ========================================================= */

/**
 * @brief 验证 uart_driver_write()
 *        对 NULL 和 0 Length 返回正确错误码。
 */
void test_uart_write_rejects_invalid_arguments(void)
{
    uint8_t dummy = 0x55U;

    initialize_uart();

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        uart_driver_write(
            NULL,
            1U
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_INVALID_LENGTH,
        uart_driver_write(
            &dummy,
            0U
        )
    );
}


/* =========================================================
 * TX Tests
 * ========================================================= */

/**
 * @brief 验证 TX IRQ 可以依次发送
 *        TX Ring Buffer 中的所有 Byte。
 */
void test_uart_tx_interrupt_drains_buffer(void)
{
    const uint8_t data[] =
    {
        0x11U,
        0x22U,
        0x33U
    };

    initialize_uart();

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        uart_driver_write(
            data,
            sizeof(data)
        )
    );

    /*
     * uart_driver_write() 在 TX 空闲时
     * 会立即写出第一个 Byte，
     * 用于 kick-start Interrupt Driven TX。
     */
    TEST_ASSERT_EQUAL_HEX8(
        0x11U,
        uart_driver_test_uart0_data
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        uart_driver_is_tx_busy()
    );

    TEST_ASSERT_BITS_HIGH(
        TEST_UART_CTRL_TX_INTERRUPT_ENABLE,
        uart_driver_test_uart0_ctrl
    );

    /*
     * 模拟第一个 Byte 发送完成。
     *
     * ISR 应继续发送 0x22。
     */
    uart_driver_test_uart0_intstatus =
        TEST_UART_INTERRUPT_TX;

    UART0_TX_IRQHandler();

    TEST_ASSERT_EQUAL_HEX8(
        0x22U,
        uart_driver_test_uart0_data
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        uart_driver_is_tx_busy()
    );

    /*
     * 模拟第二个 Byte 发送完成。
     *
     * ISR 应继续发送 0x33。
     */
    uart_driver_test_uart0_intstatus =
        TEST_UART_INTERRUPT_TX;

    UART0_TX_IRQHandler();

    TEST_ASSERT_EQUAL_HEX8(
        0x33U,
        uart_driver_test_uart0_data
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        uart_driver_is_tx_busy()
    );

    /*
     * 模拟第三个 Byte 发送完成。
     *
     * TX Ring Buffer 已空，
     * Driver 应结束 TX 并关闭 TX Interrupt。
     */
    uart_driver_test_uart0_intstatus =
        TEST_UART_INTERRUPT_TX;

    UART0_TX_IRQHandler();

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        uart_driver_is_tx_busy()
    );

    TEST_ASSERT_BITS_LOW(
        TEST_UART_CTRL_TX_INTERRUPT_ENABLE,
        uart_driver_test_uart0_ctrl
    );
}


/**
 * @brief 验证超过 TX Ring Buffer
 *        可用容量的数据被整包拒绝。
 */
void test_uart_tx_rejects_frame_larger_than_buffer(void)
{
    uint8_t data[128];
    uint32_t i;

    initialize_uart();

    for (i = 0U; i < sizeof(data); i++)
    {
        data[i] = (uint8_t)i;
    }

    /*
     * Ring Buffer Physical Capacity = 128 Byte。
     *
     * 因 head == tail 表示 Empty，
     * Usable Capacity = 127 Byte。
     *
     * 因此 128 Byte 必须整体返回 BUFFER_FULL，
     * 不允许发生部分写入。
     */
    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_BUFFER_FULL,
        uart_driver_write(
            data,
            sizeof(data)
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        uart_driver_is_tx_busy()
    );
}


/* =========================================================
 * RX Tests
 * ========================================================= */

/**
 * @brief 验证 RX Interrupt 可以接收一个 Byte、
 *        写入 RX Buffer 并触发 Callback。
 */
void test_uart_rx_interrupt_receives_byte(void)
{
    uint8_t received = 0U;

    initialize_uart();

    uart_driver_set_rx_event_callback(
        test_rx_callback
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        uart_driver_enable_rx_interrupt()
    );

    TEST_ASSERT_BITS_HIGH(
        TEST_UART_CTRL_RX_INTERRUPT_ENABLE,
        uart_driver_test_uart0_ctrl
    );

    /*
     * 模拟 UART Hardware 收到：
     *
     * 0x5A。
     */
    uart_driver_test_uart0_data =
        0x5AU;

    uart_driver_test_uart0_intstatus =
        TEST_UART_INTERRUPT_RX;

    UART0_RX_IRQHandler();

    /*
     * RX ISR 应触发一次上层 Event Callback。
     */
    TEST_ASSERT_EQUAL_UINT32(
        1U,
        rx_callback_count
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        uart_driver_read_byte(
            &received
        )
    );

    TEST_ASSERT_EQUAL_HEX8(
        0x5AU,
        received
    );

    TEST_ASSERT_EQUAL_UINT32(
        0U,
        uart_driver_get_rx_drop_count()
    );

    /*
     * 唯一 Byte 已被消费，
     * 第二次读取应返回 Empty。
     */
    TEST_ASSERT_EQUAL_UINT8(
        0U,
        uart_driver_read_byte(
            &received
        )
    );
}


/**
 * @brief 验证 RX Ring Buffer 满后
 *        新输入 Byte 会增加 Drop Counter。
 */
void test_uart_rx_overflow_increments_drop_count(void)
{
    uint32_t i;

    initialize_uart();

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        uart_driver_enable_rx_interrupt()
    );

    /*
     * RX Ring Buffer 可用容量为 127 Byte。
     *
     * 前 127 Byte 应全部成功进入 Buffer。
     */
    for (i = 0U; i < 127U; i++)
    {
        uart_driver_test_uart0_data =
            (uint8_t)i;

        uart_driver_test_uart0_intstatus =
            TEST_UART_INTERRUPT_RX;

        UART0_RX_IRQHandler();
    }

    TEST_ASSERT_EQUAL_UINT32(
        0U,
        uart_driver_get_rx_drop_count()
    );

    /*
     * 第 128 Byte 已无可用空间。
     *
     * ISR 不覆盖已有数据，
     * 而是记录一次 RX Drop。
     */
    uart_driver_test_uart0_data =
        0xAAU;

    uart_driver_test_uart0_intstatus =
        TEST_UART_INTERRUPT_RX;

    UART0_RX_IRQHandler();

    TEST_ASSERT_EQUAL_UINT32(
        1U,
        uart_driver_get_rx_drop_count()
    );
}