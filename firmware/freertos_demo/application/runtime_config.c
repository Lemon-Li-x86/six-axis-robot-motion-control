/*
 * 文件：runtime_config.c
 *
 * 用途：
 * 实现机器人固件运行时参数配置。
 *
 * 当前支持：
 *
 * ProtocolTX Period。
 */

#include <stdint.h>

#include "runtime_config.h"
#include "task_config.h"


/* =========================================================
 * Runtime State
 * ========================================================= */

/*
 * 当前 Cortex-M4 为 32 bit 平台。
 *
 * runtime_protocol_tx_period_ms 为对齐的 uint32_t，
 * 当前仅存在单一读写者模型，因此暂不额外引入 Mutex。
 *
 * 参数范围由 runtime_config.h 定义：
 *
 * RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MIN_MS
 * ~
 * RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MAX_MS
 */
static volatile uint32_t runtime_protocol_tx_period_ms =
    TASK_PERIOD_PROTOCOL_TX_MS;


/* =========================================================
 * Public API
 * ========================================================= */

robot_status_t runtime_config_init(void)
{
    /*
     * 每次初始化时恢复为 task_config.h
     * 中定义的默认 ProtocolTX 周期。
     */
    runtime_protocol_tx_period_ms =
        TASK_PERIOD_PROTOCOL_TX_MS;

    return ROBOT_STATUS_OK;
}


robot_status_t runtime_config_set_protocol_tx_period_ms(
    uint32_t period_ms
)
{
    if (
        period_ms < RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MIN_MS
        || period_ms > RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MAX_MS
    )
    {
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    }

    runtime_protocol_tx_period_ms = period_ms;

    return ROBOT_STATUS_OK;
}


uint32_t runtime_config_get_protocol_tx_period_ms(void)
{
    return runtime_protocol_tx_period_ms;
}