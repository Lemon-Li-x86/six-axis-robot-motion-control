/*
 * 文件：board.h
 *
 * 用途：
 * 定义当前目标平台的 Board Support Package 公共接口。
 *
 * 当前目标：
 *
 * QEMU MPS2-AN386
 * ARM Cortex-M4
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>


/*
 * 当前 QEMU MPS2-AN386
 * 系统时钟固定为：
 *
 * 25 MHz。
 *
 * 后续迁移到真实 MCU 时，
 * 可以替换为实际 RCC / PLL 配置结果。
 */
#define BOARD_SYSCLK_HZ 25000000UL


/**
 * @brief 初始化系统时钟。
 *
 * 当前 QEMU 平台的 SYSCLK
 * 由 Machine Model 固定提供，
 * 因此当前实现不执行实际寄存器配置。
 *
 * 保留该接口用于后续迁移真实 MCU。
 */
void board_clock_init(void);


/**
 * @brief 初始化 Board GPIO。
 *
 * 当前 QEMU MPS2-AN386 环境
 * 默认不执行实际 GPIO MMIO 配置。
 *
 * 保留该接口用于后续真实硬件实现。
 */
void board_gpio_init(void);


#endif