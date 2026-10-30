#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timer_driver.h"
#include "stage5_cpu_trace.h"

typedef struct {
    void *handle;
    uint64_t running_ticks;
} task_counter_t;

static task_counter_t counters[STAGE5_CPU_TRACE_CAPACITY];
static uint32_t used_count;
static uint32_t lost_handles;
static uint32_t last_timer_value;
static uint64_t elapsed_ticks;
static void *running_handle;
volatile stage5_cpu_trace_report_t g_stage5_cpu_trace_report;

static void account_current_time(void)
{
    uint32_t now=timer_driver_get_counter();
    uint32_t delta=timer_driver_elapsed_ticks(last_timer_value,now);
    uint32_t i;
    last_timer_value=now;
    elapsed_ticks+=(uint64_t)delta;
    if(running_handle==NULL) return;
    for(i=0U;i<used_count;++i) {
        if(counters[i].handle==running_handle) {
            counters[i].running_ticks+=(uint64_t)delta;
            return;
        }
    }
    if(used_count<STAGE5_CPU_TRACE_CAPACITY) {
        counters[used_count].handle=running_handle;
        counters[used_count].running_ticks=(uint64_t)delta;
        ++used_count;
    } else {
        ++lost_handles;
    }
}

void stage5_cpu_trace_init(void)
{
    uint32_t i;
    last_timer_value=timer_driver_get_counter();
    used_count=0U;
    lost_handles=0U;
    elapsed_ticks=0U;
    running_handle=NULL;
    for(i=0U;i<STAGE5_CPU_TRACE_CAPACITY;++i) {
        counters[i].handle=NULL;
        counters[i].running_ticks=0U;
    }
    g_stage5_cpu_trace_report.elapsed_ticks=0U;
    g_stage5_cpu_trace_report.timer_frequency_hz=
        timer_driver_get_frequency_hz();
    g_stage5_cpu_trace_report.observed_tasks=0U;
    g_stage5_cpu_trace_report.overflow_count=0U;
}
void stage5_cpu_trace_switched_out(void *handle)
{
    (void)handle;
    account_current_time();
    running_handle=NULL;
}
void stage5_cpu_trace_switched_in(void *handle)
{
    account_current_time();
    running_handle=handle;
}
void stage5_cpu_trace_tick(void)
{
    /* The RTOS tick is 1ms; ensure all delta arithmetic is short enough
     * to survive 32-bit Timer0 wraps throughout 30min+ QEMU experiments. */
    account_current_time();
}
void stage5_cpu_trace_snapshot(void)
{
    uint32_t i,j;
    /* A 64-bit sample must not tear against the tick hook on Cortex-M4. */
    taskENTER_CRITICAL();
    account_current_time();
    g_stage5_cpu_trace_report.elapsed_ticks=elapsed_ticks;
    g_stage5_cpu_trace_report.timer_frequency_hz=
        timer_driver_get_frequency_hz();
    g_stage5_cpu_trace_report.observed_tasks=used_count;
    g_stage5_cpu_trace_report.overflow_count=lost_handles;
    for(i=0U;i<used_count;++i) {
        stage5_cpu_task_sample_t *unused=NULL;
        const char *name=pcTaskGetName((TaskHandle_t)counters[i].handle);
        (void)unused;
        g_stage5_cpu_trace_report.tasks[i].task_handle=
            (uintptr_t)counters[i].handle;
        g_stage5_cpu_trace_report.tasks[i].active_ticks=
            counters[i].running_ticks;
        g_stage5_cpu_trace_report.tasks[i].cpu_share_basis_points=
            elapsed_ticks!=0U ?
            (uint32_t)((counters[i].running_ticks*10000ULL)/
                elapsed_ticks) : 0U;
        for(j=0U;j<15U;++j) {
            char ch=name!=NULL?name[j]:'\0';
            g_stage5_cpu_trace_report.tasks[i].name[j]=ch;
            if(ch=='\0') {
                ++j;
                for(;j<16U;++j)
                    g_stage5_cpu_trace_report.tasks[i].name[j]='\0';
                break;
            }
        }
        g_stage5_cpu_trace_report.tasks[i].name[15]='\0';
    }
    taskEXIT_CRITICAL();
}
/* FreeRTOS calls this only when configUSE_TICK_HOOK is opt-in. */
void vApplicationTickHook(void)
{
    stage5_cpu_trace_tick();
}
