/* QEMU-only task timing evidence. Not part of deployable firmware.
 * Shared timer source: existing CMSDK Timer0, no duplicated clock value.
 */
#ifndef STAGE4_TIMING_PROBE_H
#define STAGE4_TIMING_PROBE_H
#include <stdint.h>
#define STAGE4_TIMING_CAPACITY 128U
#define STAGE4_TIMING_CONTROL 0U
#define STAGE4_TIMING_TRAJECTORY 1U
#define STAGE4_TIMING_MONITOR 2U
#define STAGE4_TIMING_TASK_COUNT 3U

typedef struct {
    uint32_t interval_sample_count;
    uint32_t execution_sample_count;
    uint32_t expected_period_counter_ticks;
    uint32_t maximum_abs_period_jitter_ticks;
    uint32_t p99_abs_period_jitter_ticks;
    uint32_t maximum_processing_ticks;
    uint32_t p99_processing_ticks;
} stage4_timing_summary_t;

/* Written by QEMU smoke task after its healthy-motion interval. */
extern volatile stage4_timing_summary_t
    g_stage4_timing_summary[STAGE4_TIMING_TASK_COUNT];
extern volatile uint32_t g_stage4_timing_frequency_hz;

/* Called exactly once at the start/end of each periodic task activation.
 * Start is a Timer0 counter value (the hardware counts DOWN). */
uint32_t stage4_timing_start(uint32_t task_index);
void stage4_timing_end(uint32_t task_index, uint32_t start_counter);

/* Only invoked by QEMU-only test task; samples are chronological arrival
 * intervals and the observed processing window after each task wake. */
void stage4_timing_capture(void);
#endif
