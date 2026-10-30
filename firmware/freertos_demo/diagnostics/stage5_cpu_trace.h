/* Stage5 QEMU-only task CPU trace. FreeRTOS switch/tick hooks sample the
 * existing 25 MHz Timer0; a 1ms tick heartbeat avoids 32-bit wrap across
 * the required long simulation windows. Not a physical-MCU CPU benchmark.
 */
#ifndef STAGE5_CPU_TRACE_H
#define STAGE5_CPU_TRACE_H
#include <stdint.h>
#define STAGE5_CPU_TRACE_CAPACITY 16U
typedef struct {
    uintptr_t task_handle;
    uint64_t active_ticks;
    uint32_t cpu_share_basis_points; /* 10000 = 100%, since boot */
    char name[16]; /* copied from existing FreeRTOS task name */
} stage5_cpu_task_sample_t;
typedef struct {
    uint64_t elapsed_ticks;
    uint32_t timer_frequency_hz;
    uint32_t observed_tasks;
    uint32_t overflow_count;
    stage5_cpu_task_sample_t tasks[STAGE5_CPU_TRACE_CAPACITY];
} stage5_cpu_trace_report_t;
/* GDB inspection snapshot refreshed by Monitor Task. */
extern volatile stage5_cpu_trace_report_t g_stage5_cpu_trace_report;
void stage5_cpu_trace_init(void);
void stage5_cpu_trace_switched_in(void *handle);
void stage5_cpu_trace_switched_out(void *handle);
void stage5_cpu_trace_tick(void);
void stage5_cpu_trace_snapshot(void);
#endif
