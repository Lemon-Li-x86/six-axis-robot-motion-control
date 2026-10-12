
/*
 * 文件：uart_tx_boundary_main.c
 *
 * 用途：
 * Cortex-M4 UART TX 软件队列容量与原子性测试。
 *
 * 使用生产版 UART Driver 和 MMIO。
 * 暂时屏蔽 CPU 可屏蔽中断，避免 TX ISR
 * 在本测试执行过程中消费队列。
 *
 * 不修改生产驱动源码。
 */

#include <stdint.h>

#include "uart_driver.h"
#include "ring_buffer.h"

#define TEST_COUNT 7U

typedef struct
{
    uint32_t total;
    uint32_t passed;
    uint32_t failed;
    uint32_t results[TEST_COUNT];

    int32_t preinit_status;
    int32_t init_status;
    int32_t first_status;
    int32_t fill_status;
    int32_t reject_frame_status;
    int32_t reject_byte_status;

} uart_tx_test_report_t;

/* GDB 读取此全局报告。 */
volatile uart_tx_test_report_t
    g_uart_tx_test_report;

static void record_result(
    uint32_t index,
    uint32_t passed
)
{
    g_uart_tx_test_report.results[index] =
        passed ? 1U : 0U;

    if (passed)
    {
        g_uart_tx_test_report.passed++;
    }
    else
    {
        g_uart_tx_test_report.failed++;
    }
}

/* GDB 测试完成断点。 */
__attribute__((noinline))
void qemu_uart_tx_test_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}

int main(void)
{
    uint8_t first_byte = 0x55U;
    uint8_t fill_data[RING_BUFFER_CAPACITY - 1U];
    uint8_t rejected_frame[17];

    uint32_t i;

    g_uart_tx_test_report.total = TEST_COUNT;

    /*
     * 屏蔽 CPU 可屏蔽中断。
     *
     * 驱动仍会配置真实 UART MMIO / NVIC，
     * 但 TX ISR 不会在测试期间消费队列。
     */
    __asm volatile ("cpsid i" ::: "memory");

    /* 1. 未初始化时必须拒绝发送。 */
    g_uart_tx_test_report.preinit_status =
        uart_driver_write(&first_byte, 1U);

    record_result(
        0U,
        g_uart_tx_test_report.preinit_status ==
            ROBOT_STATUS_ERROR_NOT_READY
    );

    /* 2. 初始化生产 UART Driver。 */
    g_uart_tx_test_report.init_status =
        uart_driver_init();

    record_result(
        1U,
        g_uart_tx_test_report.init_status ==
            ROBOT_STATUS_OK
    );

    /* 3. 首字节启动硬件发送。 */
    g_uart_tx_test_report.first_status =
        uart_driver_write(&first_byte, 1U);

    record_result(
        2U,
        g_uart_tx_test_report.first_status ==
            ROBOT_STATUS_OK
    );

    /* 4. TX 状态必须为 Busy。 */
    record_result(
        3U,
        uart_driver_is_tx_busy() == 1U
    );

    /*
     * 5. 填满软件发送队列。
     *
     * 第一个字节已写入 UART DATA。
     * 软件 Ring Buffer 此前为空。
     *
     * 填入 127 字节后，软件队列应达到容量上限。
     */
    for (i = 0U;
         i < RING_BUFFER_CAPACITY - 1U;
         i++)
    {
        fill_data[i] = (uint8_t)i;
    }

    g_uart_tx_test_report.fill_status =
        uart_driver_write(
            fill_data,
            RING_BUFFER_CAPACITY - 1U
        );

    record_result(
        4U,
        g_uart_tx_test_report.fill_status ==
            ROBOT_STATUS_OK
    );

    /*
     * 6. 尝试额外发送完整 17 字节协议帧。
     *
     * 空间不足时必须整帧拒绝。
     */
    for (i = 0U; i < sizeof(rejected_frame); i++)
    {
        rejected_frame[i] = 0xEEU;
    }

    g_uart_tx_test_report.reject_frame_status =
        uart_driver_write(
            rejected_frame,
            sizeof(rejected_frame)
        );

    record_result(
        5U,
        g_uart_tx_test_report.reject_frame_status ==
            ROBOT_STATUS_ERROR_BUFFER_FULL
    );

    /*
     * 7. 再尝试追加单字节。
     *
     * 满队列仍应拒绝写入。
     */
    g_uart_tx_test_report.reject_byte_status =
        uart_driver_write(
            &first_byte,
            1U
        );

    record_result(
        6U,
        g_uart_tx_test_report.reject_byte_status ==
            ROBOT_STATUS_ERROR_BUFFER_FULL
    );

    qemu_uart_tx_test_done();

    return 0;
}
