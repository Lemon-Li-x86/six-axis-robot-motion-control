#include <stdint.h>

#include "unity.h"

#include "uart_driver.h"
#include "uart_driver_test_backend.h"
#include "error_code.h"


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


static uint32_t
    rx_callback_count = 0U;


static void test_rx_callback(void)
{
    rx_callback_count++;
}


static void initialize_uart(void)
{
    uart_driver_test_backend_reset();

    rx_callback_count = 0U;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        uart_driver_init()
    );
}


void test_uart_init_configures_hardware(void)
{
    initialize_uart();

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
     * uart_driver_write() 应立即 kick-start
     * 第一个字节。
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
     * 模拟第一个字节发送完成。
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
     * 第二个字节完成。
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
     * 第三个字节完成。
     *
     * Ring Buffer 此时为空，
     * Driver 应结束 TX。
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


void test_uart_tx_rejects_frame_larger_than_buffer(void)
{
    uint8_t data[128];
    uint32_t i;

    initialize_uart();

    for (i = 0U; i < sizeof(data); i++)
    {
        data[i] =
            (uint8_t)i;
    }

    /*
     * Ring Buffer 实际容量为 127 Byte。
     *
     * 128 Byte 必须整包拒绝，
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
     * 模拟 UART 收到一个 Byte。
     */
    uart_driver_test_uart0_data =
        0x5AU;

    uart_driver_test_uart0_intstatus =
        TEST_UART_INTERRUPT_RX;

    UART0_RX_IRQHandler();

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

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        uart_driver_read_byte(
            &received
        )
    );
}


void test_uart_rx_overflow_increments_drop_count(void)
{
    uint32_t i;

    initialize_uart();

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        uart_driver_enable_rx_interrupt()
    );

    /*
     * RX Ring Buffer 可容纳 127 Byte。
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
     * 第 128 Byte 无空间，
     * 应记录一次 Drop。
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