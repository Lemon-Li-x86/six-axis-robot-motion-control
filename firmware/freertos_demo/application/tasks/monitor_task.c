/* Stage 4 monitor: no UART traffic and no motor commands.
 * Periodic diagnostics + high-water marks for real stack validation.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "motion_tasks_internal.h"
#include "task_config.h"
#if ROBOT_ENABLE_STAGE5_CPU_TRACE
#include "stage5_cpu_trace.h"
#endif
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
#include "stage4_timing_probe.h"
#endif

static void task_monitor(void *arg)
{
    motion_runtime_t *r=(motion_runtime_t *)arg;
    TickType_t wake=xTaskGetTickCount();
    TickType_t period=pdMS_TO_TICKS(TASK_PERIOD_MONITOR_MS);
#if ROBOT_ENABLE_STAGE5_MONITOR_STACK_OPT
    uint32_t stack_scan_phase=0U;
    _Static_assert(TASK_MONITOR_STACK_SCAN_INTERVAL>0U,
                   "Invalid Stage5 JSON-owned watermark cadence");
#endif
    for(;;) {
        uint32_t control_free=0U, trajectory_free=0U,monitor_free=0U;
        uint8_t scan_stacks=1U;
        motion_task_wait_next_period(&wake,period);
#if ROBOT_ENABLE_STAGE5_MONITOR_STACK_OPT
        /* No scheduler-blocking scan on 9 out of 10 monitor periods.
         * Keep monitoring_ticks at its original JSON-owned cadence. */
        scan_stacks=(uint8_t)(stack_scan_phase==0U);
        stack_scan_phase++;
        if(stack_scan_phase>=TASK_MONITOR_STACK_SCAN_INTERVAL)
            stack_scan_phase=0U;
#endif
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
        uint32_t timing_start=stage4_timing_start(STAGE4_TIMING_MONITOR);
#endif
        if(scan_stacks) {
            control_free=(uint32_t)uxTaskGetStackHighWaterMark(r->control_handle);
            trajectory_free=(uint32_t)uxTaskGetStackHighWaterMark(r->trajectory_handle);
            monitor_free=(uint32_t)uxTaskGetStackHighWaterMark(NULL);
        }
        taskENTER_CRITICAL();
        r->diagnostics.monitoring_ticks++;
        if(scan_stacks) {
            r->diagnostics.control_stack_free_words=control_free;
            r->diagnostics.trajectory_stack_free_words=trajectory_free;
            r->diagnostics.monitor_stack_free_words=monitor_free;
        }
        taskEXIT_CRITICAL();
#if ROBOT_ENABLE_STAGE5_CPU_TRACE
        stage5_cpu_trace_snapshot();
#endif
#if ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST
        stage4_timing_end(STAGE4_TIMING_MONITOR,timing_start);
#endif
    }
}

BaseType_t monitor_task_create(void)
{
    return xTaskCreate(task_monitor,"Monitor",
        TASK_STACK_DEPTH_MONITOR,&g_motion_runtime,
        TASK_PRIORITY_MONITOR,&g_motion_runtime.monitor_handle);
}
