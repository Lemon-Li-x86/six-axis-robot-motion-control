/* Private FreeRTOS task bindings for Stage 4. */
#ifndef ROBOT_MOTION_TASKS_INTERNAL_H
#define ROBOT_MOTION_TASKS_INTERNAL_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_pipeline.h"

typedef enum {
    MOTION_REQUEST_JOINT = 1,
    MOTION_REQUEST_CARTESIAN = 2
} motion_request_kind_t;
typedef struct {
    motion_request_kind_t kind;
    robot_joint_positions_t joint_goal;
    cartesian_trajectory_plan_t cartesian;
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
    TaskHandle_t trajectory_handle;
    TaskHandle_t control_handle;
    TaskHandle_t monitor_handle;
} motion_runtime_t;

extern motion_runtime_t g_motion_runtime;
BaseType_t trajectory_task_create(void);
BaseType_t control_task_create(void);
BaseType_t monitor_task_create(void);

/* Diagnostic writers run in tasks only; critical sections protect snapshots. */
void motion_record_fault(robot_status_t status,uint8_t is_controller);
#endif
