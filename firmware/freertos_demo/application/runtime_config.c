/*
 * 文件：runtime_config.c
 *
 * 用途：
 * 实现机器人固件运行时参数配置。
 *
 * 当前支持：
 *
 * ProtocolTX Period。
 *
 * 当前 Cortex-M4 为 32 bit 平台，
 * 对齐 uint32_t 的单次读写是原子的。
 *
 * 当前仅有一个简单配置值，
 * 因此暂不引入 Mutex。
 */

#include <stdint.h>

#include "runtime_config.h"
#include "task_config.h"


/* =========================================================
 * Runtime State
 * ========================================================= */

static volatile uint32_t
    runtime_protocol_tx_period_ms =
        TASK_PERIOD_PROTOCOL_TX_MS;


/* =========================================================
 * Init
 * ========================================================= */

robot_status_t
runtime_config_init(void)
{
    runtime_protocol_tx_period_ms =
        TASK_PERIOD_PROTOCOL_TX_MS;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Set Protocol TX Period
 * ========================================================= */

robot_status_t
runtime_config_set_protocol_tx_period_ms(
    uint32_t period_ms
)
{
    if (
        period_ms
        <
        RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MIN_MS
        ||
        period_ms
        >
        RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MAX_MS
    )
    {
        return
            ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }


    runtime_protocol_tx_period_ms =
        period_ms;


    return
        ROBOT_STATUS_OK;
}


/* =========================================================
 * Get Protocol TX Period
 * ========================================================= */

uint32_t
runtime_config_get_protocol_tx_period_ms(void)
{
    return
        runtime_protocol_tx_period_ms;
}