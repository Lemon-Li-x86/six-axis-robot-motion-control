/**
 * @file runtime_config.h
 * @brief 运行时配置接口。
 */

#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdint.h>

#include "error_code.h"

/* Protocol TX Period */

#define RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MIN_MS \
    1U

#define RUNTIME_CONFIG_PROTOCOL_TX_PERIOD_MAX_MS \
    60000U

/* Runtime Config API */

/**
 * @brief 初始化运行时配置。
 *
 * ProtocolTX Period
 * 恢复为 task_config.h 中的默认值。
 */
robot_status_t runtime_config_init(void);

/**
 * @brief 设置 ProtocolTX 周期。
 *
 * @param[in] period_ms
 * 单位：
 * millisecond。
 *
 * 有效范围：
 * 1 ~ 60000 ms。
 */
robot_status_t runtime_config_set_protocol_tx_period_ms(
    uint32_t period_ms
);

/**
 * @brief 获取当前 ProtocolTX 周期。
 *
 * @return
 * 当前周期，单位 ms。
 */
uint32_t runtime_config_get_protocol_tx_period_ms(void);

#endif
