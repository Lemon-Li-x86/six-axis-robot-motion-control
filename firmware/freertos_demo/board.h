/**
 * @file board.h
 * @brief MPS2-AN386 platform clock, NVIC priority and init interfaces.
 *
 * Hardware-level settings live here; application PID/task defaults live
 * in config/robot_defaults.json.
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#define BOARD_SYSCLK_HZ 25000000UL

/* UART RX/TX use the FreeRTOS FromISR syscall priority boundary (8 NVIC bits). */
#define BOARD_UART_IRQ_PRIORITY 0x80U

/** @brief Initialize the board clock (fixed by QEMU machine model). */
void board_clock_init(void);

/** @brief Initialize GPIO for the active board backend. */
void board_gpio_init(void);

#endif
