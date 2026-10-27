/* Timer0-based QEMU-only scheduling and processing-window samples.
 * No heap, no libc sorting, no duplicate clock constants and no
 * potentially blocking diagnostic calls in the production task path.
 */
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "task_config.h"
#include "timer_driver.h"
#include "stage4_timing_probe.h"

typedef struct {
    uint32_t previous_start;
    uint32_t seen_start;
    uint32_t interval_count;
    uint32_t processing_count;
    uint32_t intervals[STAGE4_TIMING_CAPACITY];
    uint32_t processing[STAGE4_TIMING_CAPACITY];
} timing_samples_t;

static timing_samples_t g_samples[STAGE4_TIMING_TASK_COUNT];
static timing_samples_t g_snapshot[STAGE4_TIMING_TASK_COUNT];
static uint32_t g_sorted_scratch[STAGE4_TIMING_CAPACITY];
volatile stage4_timing_summary_t
    g_stage4_timing_summary[STAGE4_TIMING_TASK_COUNT];
volatile uint32_t g_stage4_timing_frequency_hz=0U;

uint32_t stage4_timing_start(uint32_t index)
{
    uint32_t now=timer_driver_get_counter();
    timing_samples_t *s;
    if(index>=STAGE4_TIMING_TASK_COUNT) return now;
    s=&g_samples[index];
    if(s->seen_start!=0U && s->interval_count<STAGE4_TIMING_CAPACITY) {
        s->intervals[s->interval_count++]=
            timer_driver_elapsed_ticks(s->previous_start,now);
    }
    s->previous_start=now;
    s->seen_start=1U;
    return now;
}

void stage4_timing_end(uint32_t index,uint32_t start)
{
    uint32_t end=timer_driver_get_counter();
    timing_samples_t *s;
    if(index>=STAGE4_TIMING_TASK_COUNT) return;
    s=&g_samples[index];
    if(s->processing_count<STAGE4_TIMING_CAPACITY)
        s->processing[s->processing_count++]=
            timer_driver_elapsed_ticks(start,end);
}

static uint32_t p99_of(uint32_t *values,uint32_t count)
{
    uint32_t i;
    uint32_t rank;
    if(count==0U) return 0U;
    for(i=1U;i<count;i++) {
        uint32_t j=i;
        uint32_t v=values[i];
        while(j>0U && values[j-1U]>v) {
            values[j]=values[j-1U];
            --j;
        }
        values[j]=v;
    }
    /* Nearest-rank empirical 99th percentile: ceil(0.99*N)-1. */
    rank=(99U*count+99U)/100U;
    return values[rank-1U];
}

void stage4_timing_capture(void)
{
    const uint32_t nominal_ms[STAGE4_TIMING_TASK_COUNT]={
        TASK_PERIOD_CONTROL_MS,
        TASK_PERIOD_TRAJECTORY_MS,
        TASK_PERIOD_MONITOR_MS
    };
    uint32_t clock=timer_driver_get_frequency_hz();
    uint32_t i,k;
    /* The test task takes a single consistent snapshot; the three
     * periodic writers otherwise own disjoint g_samples slots. */
    taskENTER_CRITICAL();
    memcpy(g_snapshot,g_samples,sizeof(g_snapshot));
    taskEXIT_CRITICAL();
    g_stage4_timing_frequency_hz=clock;
    for(i=0U;i<STAGE4_TIMING_TASK_COUNT;i++) {
        timing_samples_t *s=&g_snapshot[i];
        stage4_timing_summary_t result={0};
        uint32_t expected=(clock/1000U)*nominal_ms[i];
        if(s->interval_count>STAGE4_TIMING_CAPACITY ||
           s->processing_count>STAGE4_TIMING_CAPACITY)
            continue;
        result.expected_period_counter_ticks=expected;
        result.interval_sample_count=s->interval_count;
        result.execution_sample_count=s->processing_count;
        for(k=0U;k<s->interval_count;k++) {
            uint32_t elapsed=s->intervals[k];
            uint32_t jitter=elapsed>expected?
                elapsed-expected:expected-elapsed;
            g_sorted_scratch[k]=jitter;
            if(jitter>result.maximum_abs_period_jitter_ticks)
                result.maximum_abs_period_jitter_ticks=jitter;
        }
        result.p99_abs_period_jitter_ticks=
            p99_of(g_sorted_scratch,s->interval_count);
        for(k=0U;k<s->processing_count;k++) {
            uint32_t elapsed=s->processing[k];
            g_sorted_scratch[k]=elapsed;
            if(elapsed>result.maximum_processing_ticks)
                result.maximum_processing_ticks=elapsed;
        }
        result.p99_processing_ticks=
            p99_of(g_sorted_scratch,s->processing_count);
        g_stage4_timing_summary[i]=result;
    }
}
