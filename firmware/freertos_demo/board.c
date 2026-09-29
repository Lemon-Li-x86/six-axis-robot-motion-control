#include "board.h"


/* =========================================================
 * CMSDK AHB GPIO
 * ========================================================= */

/*
 * MPS2-AN386 GPIO0 基地址。
 *
 * 当前 QEMU 虽然保留了这个地址区域，
 * 但 CMSDK GPIO 外设本身尚未真正实现。
 *
 * 因此这里保留真实 CMSDK GPIO 的寄存器模板，
 * 当前 QEMU 构建默认不执行实际 MMIO 配置。
 */
#define CMSDK_GPIO0_BASE       0x40010000UL


/*
 * CMSDK AHB GPIO 寄存器偏移
 *
 * DATAOUT:
 *     GPIO 输出数据
 *
 * OUTENSET:
 *     将对应 GPIO 设置为输出
 *
 * ALTFUNCCLR:
 *     清除复用功能，使引脚工作为普通 GPIO
 */
#define GPIO_DATAOUT_OFFSET        0x004UL
#define GPIO_OUTENSET_OFFSET       0x010UL
#define GPIO_ALTFUNCCLR_OFFSET     0x01CUL


#define REG32(address) \
    (*(volatile uint32_t *)(address))


#define GPIO0_DATAOUT \
    REG32(CMSDK_GPIO0_BASE + GPIO_DATAOUT_OFFSET)

#define GPIO0_OUTENSET \
    REG32(CMSDK_GPIO0_BASE + GPIO_OUTENSET_OFFSET)

#define GPIO0_ALTFUNCCLR \
    REG32(CMSDK_GPIO0_BASE + GPIO_ALTFUNCCLR_OFFSET)


/*
 * 示例 GPIO：
 *
 * GPIO0 pin 0
 */
#define BOARD_GPIO_TEST_PIN    (1UL << 0)


/*
 * 当前设为 0。
 *
 * 原因：
 * QEMU mps2-an386 尚未真正实现 CMSDK AHB GPIO。
 *
 * 将来如果切换到实际硬件，
 * 或使用完整实现该 GPIO 的仿真平台，
 * 可以改为 1。
 */
#define BOARD_ENABLE_GPIO_MMIO    0


/* =========================================================
 * Clock 初始化
 * ========================================================= */

void board_clock_init(void)
{
    /*
     * 在当前 QEMU MPS2-AN386 中：
     *
     * SYSCLK = 25 MHz
     *
     * 时钟由 QEMU machine model 在启动时创建并固定，
     * 不需要固件通过 RCC / PLL 寄存器重新配置。
     *
     * 这个函数仍然保留，
     * 是为了建立标准的 BSP 初始化接口。
     *
     * 后续迁移到 STM32 时，
     * PLL、时钟源和分频器配置会放在这里。
     */
}


/* =========================================================
 * GPIO 初始化
 * ========================================================= */

void board_gpio_init(void)
{
#if BOARD_ENABLE_GPIO_MMIO

    /*
     * 关闭 pin 0 的 alternate function，
     * 使其作为普通 GPIO。
     */
    GPIO0_ALTFUNCCLR =
        BOARD_GPIO_TEST_PIN;


    /*
     * 将 pin 0 设置为输出。
     */
    GPIO0_OUTENSET =
        BOARD_GPIO_TEST_PIN;


    /*
     * 初始输出低电平。
     */
    GPIO0_DATAOUT &=
        ~BOARD_GPIO_TEST_PIN;

#else

    /*
     * 当前 QEMU MPS2-AN386 的 CMSDK AHB GPIO
     * 尚未实现实际行为。
     *
     * 因此仿真环境只保留初始化接口和寄存器模板，
     * 不进行真实 MMIO 访问。
     */
    (void)BOARD_GPIO_TEST_PIN;

#endif
}