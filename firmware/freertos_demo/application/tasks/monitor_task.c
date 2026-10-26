/* Stage 4 monitor: no UART traffic and no motor commands.
 * Periodic diagnostics + high-water marks for real stack validation.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "motion_tasks_internal.h"
#include "task_config.h"

static void task_monitor(void *arg)
{
    motion_runtime_t *r=(motion_runtime_t *)arg;
    TickType_t wake=xTaskGetTickCount();
    TickType_t period=pdMS_TO_TICKS(TASK_PERIOD_MONITOR_MS);
    for(;;) {
        uint32_t control_free, trajectory_free,monitor_free;
        vTaskDelayUntil(&wake,(period==0U)?1U:period);
        control_free=(uint32_t)uxTaskGetStackHighWaterMark(r->control_handle);
        trajectory_free=(uint32_t)uxTaskGetStackHighWaterMark(r->trajectory_handle);
        monitor_free=(uint32_t)uxTaskGetStackHighWaterMark(NULL);
        taskENTER_CRITICAL();
        r->diagnostics.monitoring_ticks++;
        r->diagnostics.control_stack_free_words=control_free;
        r->diagnostics.trajectory_stack_free_words=trajectory_free;
        r->diagnostics.monitor_stack_free_words=monitor_free;
        taskEXIT_CRITICAL();
    }
}

BaseType_t monitor_task_create(void)
{
    return xTaskCreate(task_monitor,"Monitor",
        TASK_STACK_DEPTH_MONITOR,&g_motion_runtime,
        TASK_PRIORITY_MONITOR,&g_motion_runtime.monitor_handle);
}
