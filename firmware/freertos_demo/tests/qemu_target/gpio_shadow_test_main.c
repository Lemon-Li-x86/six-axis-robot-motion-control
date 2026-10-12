
/*
 * gpio_shadow_test_main.c
 *
 * Cortex-M4 / QEMU GPIO Shadow Backend Tests
 *
 * 使用真实生产 gpio_driver.c。
 * 仅验证软件 GPIO 驱动逻辑，
 * 不代表真实 GPIO MMIO 或物理引脚测试。
 */

#include <stddef.h>
#include <stdint.h>

#include "gpio_driver.h"
#include "error_code.h"

#define GPIO_TEST_COUNT 13U

#define CHECK(condition)           \
    do                             \
    {                              \
        if (!(condition))          \
        {                          \
            return 0U;             \
        }                          \
    } while (0)

typedef struct
{
    uint32_t total;
    uint32_t passed;
    uint32_t failed;
    uint32_t results[GPIO_TEST_COUNT];

} gpio_test_report_t;

volatile gpio_test_report_t g_gpio_test_report;


/* 0. 未初始化时拒绝操作 */
static uint32_t test_before_init(void)
{
    gpio_driver_level_t level;

    CHECK(
        gpio_driver_configure_output(0U)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        gpio_driver_write(0U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        gpio_driver_toggle(0U)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        gpio_driver_get_output_level(0U, &level)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    return 1U;
}


/* 1. 确认当前为 Shadow Backend */
static uint32_t test_shadow_backend(void)
{
    CHECK(gpio_driver_is_mmio_backend() == 0U);
    return 1U;
}


/* 2. 初始化后，引脚默认未配置 */
static uint32_t test_initial_state(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(0U, &level)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    return 1U;
}


/* 3. 最小有效引脚 Pin 0 */
static uint32_t test_pin_zero(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_configure_output(0U)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_get_output_level(0U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_LOW);

    return 1U;
}


/* 4. 最大有效引脚 Pin 31 */
static uint32_t test_pin_31(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_configure_output(31U)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_write(31U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_get_output_level(31U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_HIGH);

    return 1U;
}


/* 5. 非法引脚 32、255 */
static uint32_t test_invalid_pins(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_configure_output(32U)
        == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    CHECK(
        gpio_driver_configure_output(255U)
        == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    CHECK(
        gpio_driver_write(32U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    CHECK(
        gpio_driver_toggle(32U)
        == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    CHECK(
        gpio_driver_get_output_level(32U, &level)
        == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    return 1U;
}


/* 6. 未配置输出的引脚不能使用 */
static uint32_t test_unconfigured_pin(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_write(3U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        gpio_driver_toggle(3U)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        gpio_driver_get_output_level(3U, &level)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    return 1U;
}


/* 7. 拒绝非法电平枚举值 */
static uint32_t test_invalid_level(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);
    CHECK(gpio_driver_configure_output(2U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_write(
            2U,
            (gpio_driver_level_t)2
        )
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
    );

    CHECK(
        gpio_driver_get_output_level(2U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_LOW);

    return 1U;
}


/* 8. 高低电平写入与读回 */
static uint32_t test_write_readback(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);
    CHECK(gpio_driver_configure_output(3U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_write(3U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_get_output_level(3U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_HIGH);

    CHECK(
        gpio_driver_write(3U, GPIO_DRIVER_LEVEL_LOW)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_get_output_level(3U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_LOW);

    return 1U;
}


/* 9. 连续两次 Toggle */
static uint32_t test_toggle_twice(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);
    CHECK(gpio_driver_configure_output(1U) == ROBOT_STATUS_OK);

    CHECK(gpio_driver_toggle(1U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(1U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_HIGH);

    CHECK(gpio_driver_toggle(1U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(1U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_LOW);

    return 1U;
}


/* 10. 不同引脚的状态互不干扰 */
static uint32_t test_pin_isolation(void)
{
    gpio_driver_level_t level0;
    gpio_driver_level_t level31;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(gpio_driver_configure_output(0U) == ROBOT_STATUS_OK);
    CHECK(gpio_driver_configure_output(31U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_write(0U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_get_output_level(31U, &level31)
        == ROBOT_STATUS_OK
    );

    CHECK(level31 == GPIO_DRIVER_LEVEL_LOW);

    CHECK(
        gpio_driver_write(31U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_OK
    );

    CHECK(gpio_driver_toggle(0U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(0U, &level0)
        == ROBOT_STATUS_OK
    );

    CHECK(
        gpio_driver_get_output_level(31U, &level31)
        == ROBOT_STATUS_OK
    );

    CHECK(level0 == GPIO_DRIVER_LEVEL_LOW);
    CHECK(level31 == GPIO_DRIVER_LEVEL_HIGH);

    return 1U;
}


/* 11. 重新初始化清除配置和输出状态 */
static uint32_t test_reinitialization(void)
{
    gpio_driver_level_t level;

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);
    CHECK(gpio_driver_configure_output(0U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_write(0U, GPIO_DRIVER_LEVEL_HIGH)
        == ROBOT_STATUS_OK
    );

    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(0U, &level)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(gpio_driver_configure_output(0U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(0U, &level)
        == ROBOT_STATUS_OK
    );

    CHECK(level == GPIO_DRIVER_LEVEL_LOW);

    return 1U;
}


/* 12. NULL 指针防护 */
static uint32_t test_null_pointer(void)
{
    CHECK(gpio_driver_init() == ROBOT_STATUS_OK);
    CHECK(gpio_driver_configure_output(0U) == ROBOT_STATUS_OK);

    CHECK(
        gpio_driver_get_output_level(0U, NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    return 1U;
}


/* 记录单项结果 */
static void record_result(
    uint32_t index,
    uint32_t result
)
{
    g_gpio_test_report.results[index] = result;

    if (result == 1U)
    {
        g_gpio_test_report.passed++;
    }
    else
    {
        g_gpio_test_report.failed++;
    }
}


/* GDB 测试完成断点 */
__attribute__((noinline))
void qemu_gpio_test_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}


int main(void)
{
    g_gpio_test_report.total = GPIO_TEST_COUNT;
    g_gpio_test_report.passed = 0U;
    g_gpio_test_report.failed = 0U;

    record_result(0U, test_before_init());
    record_result(1U, test_shadow_backend());
    record_result(2U, test_initial_state());
    record_result(3U, test_pin_zero());
    record_result(4U, test_pin_31());
    record_result(5U, test_invalid_pins());
    record_result(6U, test_unconfigured_pin());
    record_result(7U, test_invalid_level());
    record_result(8U, test_write_readback());
    record_result(9U, test_toggle_twice());
    record_result(10U, test_pin_isolation());
    record_result(11U, test_reinitialization());
    record_result(12U, test_null_pointer());

    qemu_gpio_test_done();

    return 0;
}
