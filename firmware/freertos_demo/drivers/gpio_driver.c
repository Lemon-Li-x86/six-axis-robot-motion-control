/*
 * 文件：gpio_driver.c
 *
 * 用途：
 * 实现 GPIO0 输出控制抽象。
 *
 * 当前项目默认运行于：
 *
 * QEMU MPS2-AN386
 *
 * 当前 QEMU 环境下 GPIO 不执行真实 MMIO，
 * 因此默认采用 Shadow Backend。
 *
 * Shadow Backend 会完整保存：
 *
 * 1. Output Enable 状态；
 * 2. Output Data 状态。
 *
 * 从而可以验证 GPIO Driver 的：
 *
 * configure
 * write
 * toggle
 * read-back
 *
 * 等逻辑。
 *
 * 当未来迁移至支持 CMSDK AHB GPIO
 * 的真实硬件或仿真平台时，
 * 将 GPIO_DRIVER_USE_MMIO 设置为 1，
 * 即可启用真实 MMIO Backend。
 */

#include <stddef.h>
#include <stdint.h>

#include "gpio_driver.h"


/* =========================================================
 * Backend Configuration
 * ========================================================= */

/*
 * 当前 QEMU MPS2-AN386：
 *
 * 0 = 使用 Shadow Backend。
 *
 * 后续真实 CMSDK GPIO 平台可改为：
 *
 * 1
 */
#ifndef GPIO_DRIVER_USE_MMIO

#define GPIO_DRIVER_USE_MMIO 0U

#endif


/* =========================================================
 * CMSDK GPIO0 Register
 * ========================================================= */

#define CMSDK_GPIO0_BASE \
    0x40010000UL


#define GPIO_DATAOUT_OFFSET \
    0x004UL


#define GPIO_OUTENSET_OFFSET \
    0x010UL


#define GPIO_ALTFUNCCLR_OFFSET \
    0x01CUL


#define REG32(address) \
    (*(volatile uint32_t *)(address))


#define GPIO0_DATAOUT \
    REG32( \
        CMSDK_GPIO0_BASE \
        + \
        GPIO_DATAOUT_OFFSET \
    )


#define GPIO0_OUTENSET \
    REG32( \
        CMSDK_GPIO0_BASE \
        + \
        GPIO_OUTENSET_OFFSET \
    )


#define GPIO0_ALTFUNCCLR \
    REG32( \
        CMSDK_GPIO0_BASE \
        + \
        GPIO_ALTFUNCCLR_OFFSET \
    )


/* =========================================================
 * Driver State
 * ========================================================= */

static uint8_t
    gpio_driver_initialized = 0U;


/*
 * 保存当前已经配置为 Output
 * 的 GPIO Pin。
 *
 * 每一 bit 对应一个 GPIO Pin。
 */
static uint32_t
    gpio_output_enable_shadow = 0U;


/*
 * 保存当前 GPIO Output Data。
 *
 * 每一 bit 对应一个 GPIO Pin。
 */
static uint32_t
    gpio_output_data_shadow = 0U;


/* =========================================================
 * Pin Validation
 * ========================================================= */

static robot_status_t
gpio_driver_validate_pin(
    uint8_t pin
)
{
    if (
        pin
        >=
        GPIO_DRIVER_PIN_COUNT
    )
    {
        return
            ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Pin Mask
 * ========================================================= */

static uint32_t
gpio_driver_pin_mask(
    uint8_t pin
)
{
    return
        (uint32_t)(
            1UL
            <<
            pin
        );
}


/* =========================================================
 * Driver Init
 * ========================================================= */

robot_status_t
gpio_driver_init(void)
{
    /*
     * 清除软件 Output 状态。
     */
    gpio_output_enable_shadow =
        0U;


    gpio_output_data_shadow =
        0U;


    gpio_driver_initialized =
        1U;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Configure Output
 * ========================================================= */

robot_status_t
gpio_driver_configure_output(
    uint8_t pin
)
{
    uint32_t
        mask;


    robot_status_t
        status;


    /*
     * Driver 必须先初始化。
     */
    if (
        !gpio_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    status =
        gpio_driver_validate_pin(
            pin
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    mask =
        gpio_driver_pin_mask(
            pin
        );


    /*
     * 记录该 Pin 已配置为 Output。
     */
    gpio_output_enable_shadow |=
        mask;


#if GPIO_DRIVER_USE_MMIO

    /*
     * 清除 Alternate Function，
     * 将该 Pin 作为普通 GPIO 使用。
     */
    GPIO0_ALTFUNCCLR =
        mask;


    /*
     * 设置 Output Enable。
     */
    GPIO0_OUTENSET =
        mask;

#endif


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Write Output
 * ========================================================= */

robot_status_t
gpio_driver_write(
    uint8_t pin,
    gpio_driver_level_t level
)
{
    uint32_t
        mask;


    robot_status_t
        status;


    /*
     * Driver 尚未初始化。
     */
    if (
        !gpio_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    status =
        gpio_driver_validate_pin(
            pin
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    /*
     * Level 必须为合法枚举值。
     */
    if (
        level
        !=
        GPIO_DRIVER_LEVEL_LOW
        &&
        level
        !=
        GPIO_DRIVER_LEVEL_HIGH
    )
    {
        return
            ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }


    mask =
        gpio_driver_pin_mask(
            pin
        );


    /*
     * 未配置为 Output 的 Pin
     * 不允许直接写。
     */
    if (
        (
            gpio_output_enable_shadow
            &
            mask
        )
        ==
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    /*
     * 更新 Shadow Output Data。
     */
    if (
        level
        ==
        GPIO_DRIVER_LEVEL_HIGH
    )
    {
        gpio_output_data_shadow |=
            mask;
    }
    else
    {
        gpio_output_data_shadow &=
            ~mask;
    }


#if GPIO_DRIVER_USE_MMIO

    /*
     * 将完整 Output Shadow
     * 写入 GPIO DATAOUT。
     */
    GPIO0_DATAOUT =
        gpio_output_data_shadow;

#endif


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Toggle Output
 * ========================================================= */

robot_status_t
gpio_driver_toggle(
    uint8_t pin
)
{
    uint32_t
        mask;


    robot_status_t
        status;


    if (
        !gpio_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    status =
        gpio_driver_validate_pin(
            pin
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    mask =
        gpio_driver_pin_mask(
            pin
        );


    /*
     * 必须先配置为 Output。
     */
    if (
        (
            gpio_output_enable_shadow
            &
            mask
        )
        ==
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    /*
     * 翻转目标 Bit。
     */
    gpio_output_data_shadow ^=
        mask;


#if GPIO_DRIVER_USE_MMIO

    GPIO0_DATAOUT =
        gpio_output_data_shadow;

#endif


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Get Output Level
 * ========================================================= */

robot_status_t
gpio_driver_get_output_level(
    uint8_t pin,
    gpio_driver_level_t *level
)
{
    uint32_t
        mask;


    robot_status_t
        status;


    if (
        level
        ==
        NULL
    )
    {
        return
            ROBOT_STATUS_ERROR_NULL_POINTER;
    }


    if (
        !gpio_driver_initialized
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    status =
        gpio_driver_validate_pin(
            pin
        );


    if (
        status
        !=
        ROBOT_STATUS_OK
    )
    {
        return
            status;
    }


    mask =
        gpio_driver_pin_mask(
            pin
        );


    /*
     * 尚未配置为 Output。
     */
    if (
        (
            gpio_output_enable_shadow
            &
            mask
        )
        ==
        0U
    )
    {
        return
            ROBOT_STATUS_ERROR_NOT_READY;
    }


    /*
     * 从 Shadow State 读取当前输出。
     */
    if (
        gpio_output_data_shadow
        &
        mask
    )
    {
        *level =
            GPIO_DRIVER_LEVEL_HIGH;
    }
    else
    {
        *level =
            GPIO_DRIVER_LEVEL_LOW;
    }


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Backend Query
 * ========================================================= */

uint8_t
gpio_driver_is_mmio_backend(void)
{
#if GPIO_DRIVER_USE_MMIO

    return 1U;

#else

    return 0U;

#endif
}