/**
 * @file task_config.h
 * @brief FreeRTOS task priority, period and stack configuration aliases.
 *
 * Values originate from config/robot_defaults.json and are generated at
 * build time in firmware_defaults_generated.h. Higher numbers mean higher
 * FreeRTOS priorities; the generator validates the configured bounds.
 * Stack depths use StackType_t words, not bytes. ProtocolTX period is an
 * initial value that runtime configuration may change.
 */



#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H





#include "firmware_defaults_generated.h"







#define TASK_PRIORITY_CONTROL \
    FW_DEFAULT_TASK_PRIORITY_CONTROL



#define TASK_PRIORITY_PROTOCOL_RX \
    FW_DEFAULT_TASK_PRIORITY_PROTOCOL_RX



#define TASK_PRIORITY_TRAJECTORY \
    FW_DEFAULT_TASK_PRIORITY_TRAJECTORY



#define TASK_PRIORITY_PROTOCOL_TX \
    FW_DEFAULT_TASK_PRIORITY_PROTOCOL_TX



#define TASK_PRIORITY_MONITOR \
    FW_DEFAULT_TASK_PRIORITY_MONITOR







#define TASK_PERIOD_CONTROL_MS \
    FW_DEFAULT_TASK_PERIOD_CONTROL_MS



#define TASK_PERIOD_TRAJECTORY_MS \
    FW_DEFAULT_TASK_PERIOD_TRAJECTORY_MS



#define TASK_PERIOD_MONITOR_MS \
    FW_DEFAULT_TASK_PERIOD_MONITOR_MS


/* Scan stack watermarks less frequently than the monitor tick.
 * The scan walks allocated stack words; cached minima remain conservative.
 */
#define TASK_MONITOR_STACK_SCAN_INTERVAL \
    (FW_DEFAULT_STAGE5_MONITOR_STACK_SAMPLE_MS / TASK_PERIOD_MONITOR_MS)




#define TASK_PERIOD_PROTOCOL_TX_MS \
    FW_DEFAULT_TASK_PERIOD_PROTOCOL_TX_MS


#define PROTOCOL_FRAME_TIMEOUT_MS \
    FW_DEFAULT_PROTOCOL_FRAME_TIMEOUT_MS







#define TASK_STACK_DEPTH_CONTROL \
    FW_DEFAULT_TASK_STACK_CONTROL_WORDS



#define TASK_STACK_DEPTH_PROTOCOL_RX \
    FW_DEFAULT_TASK_STACK_PROTOCOL_RX_WORDS



#define TASK_STACK_DEPTH_TRAJECTORY \
    FW_DEFAULT_TASK_STACK_TRAJECTORY_WORDS



#define TASK_STACK_DEPTH_PROTOCOL_TX \
    FW_DEFAULT_TASK_STACK_PROTOCOL_TX_WORDS



#define TASK_STACK_DEPTH_MONITOR \
    FW_DEFAULT_TASK_STACK_MONITOR_WORDS


#endif 
