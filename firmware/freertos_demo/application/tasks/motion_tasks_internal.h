/* Private FreeRTOS task bindings for Stage 4. */
#ifndef ROBOT_MOTION_TASKS_INTERNAL_H
#define ROBOT_MOTION_TASKS_INTERNAL_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_pipeline.h"

typedef enum {
    MOTION_REQUEST_JOINT = 1,
    MOTION_REQUEST_CARTESIAN = 2,
    MOTION_REQUEST_POLYNOMIAL = 3
} motion_request_kind_t;
/* Exactly ONE request kind is active at a time. A union avoids wasting
 * the ProtocolRX 256-word stack on three mutually exclusive plans.
 * Keep public field access unchanged for Stage4 task consumers. */
typedef struct {
    motion_request_kind_t kind;
    union {
        robot_joint_positions_t joint_goal;
        cartesian_trajectory_plan_t cartesian;
        trajectory_polynomial_plan_t polynomial;
    };
} motion_request_t;
typedef struct {
    robot_joint_positions_t target;
    TickType_t created_tick;
    uint32_t sequence;
    uint8_t enabled; /* 0=cancel/disable command output */
} motion_reference_t;
typedef struct {
    QueueHandle_t request_queue;
    QueueHandle_t reference_queue;
    motion_stage4_config_t config;
    motion_stage4_diagnostics_t diagnostics;
    volatile uint8_t abort_requested; /* controller-to-planner cancellation */
    TaskHandle_t trajectory_handle;
    TaskHandle_t control_handle;
    TaskHandle_t monitor_handle;
} motion_runtime_t;

extern motion_runtime_t g_motion_runtime;
BaseType_t trajectory_task_create(void);
BaseType_t control_task_create(void);
BaseType_t monitor_task_create(void);

/* Shared periodic-task overrun policy (existing tasks only).
 * FreeRTOS vTaskDelayUntil tries to 'catch up' if the previous wake is
 * already overdue, which can spin a high-priority loop and starve planning
 * after a slow UART/QEMU burst. Drop missed activations instead: resume
 * from the current RTOS tick and always make the next period blocking.
 *
 * This is a scheduling policy, NOT a second owner of task periods. */
static inline void motion_task_wait_next_period(
    TickType_t *last_wake,
    TickType_t configured_period)
{
    TickType_t period=(configured_period==0U)?1U:configured_period;
    TickType_t now=xTaskGetTickCount();
    if((TickType_t)(now-*last_wake)>=period)
        *last_wake=now;
    vTaskDelayUntil(last_wake,period);
}

/* Diagnostic writers run in tasks only; critical sections protect snapshots. */
void motion_record_fault(robot_status_t status,uint8_t is_controller);
#endif
