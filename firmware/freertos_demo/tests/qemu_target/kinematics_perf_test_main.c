
/*
 * Stage 3: Cortex-M4 / QEMU kinematics performance diagnostic.
 *
 * Measures FK, analytic IK, and IK selection separately, on the same
 * deterministic 128-pose dataset as qemu_ik_batch.
 *
 * Timer: CMSDK APB Timer0 on MPS2-AN386, nominally 25 MHz.
 * Ticks in QEMU are EMULATED TIMER ticks, not real MCU CPU cycles.
 *
 * Results: inspect g_kin_perf_report and g_kin_perf_samples with GDB.
 * This executable runs without FreeRTOS, UART or printf.
 */
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include "timer_driver.h"
#include "kinematics.h"
#include "ik_solution_selector.h"

#define KIN_PERF_SAMPLE_COUNT 128U
#define KIN_PERF_REPETITIONS 8U
#define KIN_PERF_WRIST_SWEEP_COUNT 19U
#define KIN_PERF_CONTROLLED_COUNT (2U * KIN_PERF_WRIST_SWEEP_COUNT)

typedef struct
{
    uint32_t measured;
    uint32_t min_ticks;
    uint32_t max_ticks;
    uint64_t sum_ticks;
} kin_perf_statistics_t;

typedef struct
{
    uint32_t fk_ticks;
    uint32_t ik_ticks;
    uint32_t selection_ticks;
    int32_t fk_status;
    int32_t ik_status;
    int32_t selection_status;
    uint32_t ik_solution_count;
    int32_t roundtrip_status;
    float roundtrip_position_error_mm;
    float roundtrip_rotation_max_element_error;
} kin_perf_sample_t;

typedef struct
{
    uint32_t sample_count;
    uint32_t repetitions;
    uint32_t timer_frequency_hz;
    uint32_t timer_probe_ticks;
    uint32_t timer_read_pair_ticks;
    uint32_t fk_success;
    uint32_t ik_success;
    uint32_t selection_success;
    uint32_t roundtrip_passed;
    uint32_t roundtrip_failed;
    uint32_t roundtrip_skipped;
    float worst_position_error_mm;
    float worst_rotation_max_element_error;
    kin_perf_statistics_t fk;
    kin_perf_statistics_t ik;
    kin_perf_statistics_t selection;
} kin_perf_report_t;

volatile kin_perf_sample_t
    g_kin_perf_samples[KIN_PERF_SAMPLE_COUNT];

volatile kin_perf_report_t g_kin_perf_report;
volatile uint32_t g_kin_perf_finished = 0U;
volatile uint32_t g_kin_perf_sink = 0U;

static const int16_t wrist_sweep_raw[
    KIN_PERF_WRIST_SWEEP_COUNT
] =
{
    -500, -200, -100, -50, -20, -10, -5, -2, -1, 0,
    1, 2, 5, 10, 20, 50, 100, 200, 500
};

static uint32_t next_random(uint32_t *state)
{
    *state = 1664525UL * (*state) + 1013904223UL;
    return *state;
}

static void make_pose(
    uint32_t index,
    uint32_t *random_state,
    robot_joint_angles_t *joints)
{
    uint32_t j;

    if (index < KIN_PERF_CONTROLLED_COUNT)
    {
        joints->value[0] = 3000;
        joints->value[1] = -4500;
        joints->value[2] =
            (index < KIN_PERF_WRIST_SWEEP_COUNT)
                ? 6000 : 0;
        joints->value[3] = 2000;
        joints->value[4] = wrist_sweep_raw[
            index % KIN_PERF_WRIST_SWEEP_COUNT];
        joints->value[5] = -1000;
    }
    else
    {
        for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
        {
            uint32_t value = next_random(random_state);

            joints->value[j] =
                (int16_t)(
                    (int32_t)(value % 32001U) - 16000
                );
        }
    }
}

static void record_ticks(
    kin_perf_statistics_t *stats,
    uint32_t ticks)
{
    if (
        stats->measured == 0U
        || ticks < stats->min_ticks
    )
    {
        stats->min_ticks = ticks;
    }

    if (ticks > stats->max_ticks)
    {
        stats->max_ticks = ticks;
    }

    stats->sum_ticks += (uint64_t)ticks;
    stats->measured++;
}

static uint32_t sample_ticks(
    uint32_t start,
    uint32_t end)
{
    uint32_t total =
        timer_driver_elapsed_ticks(start, end);

    /*
     * Rounded average per operation.
     * Timer-read overhead is amortized
     * across repeated solver calls.
     */
    return (
        total + KIN_PERF_REPETITIONS / 2U
    ) / KIN_PERF_REPETITIONS;
}

__attribute__((noinline))
void qemu_kinematics_perf_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}

int main(void)
{
    uint32_t random_state = 20261012UL;
    uint32_t index;

    kin_perf_report_t report = {0};

    timer_driver_init();

    report.sample_count = KIN_PERF_SAMPLE_COUNT;
    report.repetitions = KIN_PERF_REPETITIONS;

    report.timer_frequency_hz =
        timer_driver_get_frequency_hz();

    /*
     * Measure the overhead of reading
     * the timer twice.
     */
    {
        uint32_t start = timer_driver_get_counter();
        uint32_t end = timer_driver_get_counter();

        report.timer_read_pair_ticks =
            timer_driver_elapsed_ticks(start, end);
    }

    /*
     * Sanity check: ensure the emulated
     * timer advances during execution.
     */
    {
        uint32_t start = timer_driver_get_counter();

        for (index = 0U; index < 4096U; ++index)
        {
            g_kin_perf_sink += index;
        }

        report.timer_probe_ticks =
            timer_driver_elapsed_ticks(
                start,
                timer_driver_get_counter()
            );
    }

    /*
     * Warm up math execution outside timing.
     */
    {
        robot_joint_angles_t warmup =
        {
            {3000, -4500, 6000, 2000, 500, -1000}
        };

        robot_transform_t target = {{{0}}};
        kinematics_ik_solutions_t solutions = {0};

        (void)kinematics_forward(&warmup, &target);
        (void)kinematics_inverse(&target, &solutions);
    }

    for (
        index = 0U;
        index < KIN_PERF_SAMPLE_COUNT;
        ++index
    )
    {
        robot_joint_angles_t input = {{0}};
        robot_joint_positions_t current = {{0}};

        robot_transform_t target = {{{0}}};
        kinematics_ik_solutions_t solutions = {0};
        ik_selection_result_t selected = {0};

        robot_status_t fk_status =
            ROBOT_STATUS_ERROR_NOT_READY;

        robot_status_t ik_status =
            ROBOT_STATUS_ERROR_NOT_READY;

        robot_status_t selection_status =
            ROBOT_STATUS_ERROR_NOT_READY;

        uint32_t start;
        uint32_t end;
        uint32_t ticks;
        uint32_t repeat;
        uint32_t j;

        make_pose(index, &random_state, &input);

        for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
        {
            current.value[j] =
                (robot_joint_position_t)input.value[j];
        }

        /*
         * FK benchmark
         */
        start = timer_driver_get_counter();

        for (
            repeat = 0U;
            repeat < KIN_PERF_REPETITIONS;
            ++repeat
        )
        {
            fk_status =
                kinematics_forward(&input, &target);
        }

        end = timer_driver_get_counter();
        ticks = sample_ticks(start, end);

        g_kin_perf_samples[index].fk_ticks = ticks;
        g_kin_perf_samples[index].fk_status =
            (int32_t)fk_status;

        record_ticks(&report.fk, ticks);

        if (fk_status != ROBOT_STATUS_OK)
        {
            g_kin_perf_samples[index].ik_status =
                ROBOT_STATUS_ERROR_NOT_READY;

            g_kin_perf_samples[index].selection_status =
                ROBOT_STATUS_ERROR_NOT_READY;

            continue;
        }

        report.fk_success++;

        g_kin_perf_sink ^=
            (uint32_t)((int32_t)target.matrix[0][3]);

        /*
         * IK benchmark
         */
        start = timer_driver_get_counter();

        for (
            repeat = 0U;
            repeat < KIN_PERF_REPETITIONS;
            ++repeat
        )
        {
            ik_status =
                kinematics_inverse(&target, &solutions);
        }

        end = timer_driver_get_counter();
        ticks = sample_ticks(start, end);

        g_kin_perf_samples[index].ik_ticks = ticks;
        g_kin_perf_samples[index].ik_status =
            (int32_t)ik_status;

        g_kin_perf_samples[index].ik_solution_count =
            solutions.count;

        record_ticks(&report.ik, ticks);

        if (ik_status != ROBOT_STATUS_OK)
        {
            g_kin_perf_samples[index].selection_status =
                ROBOT_STATUS_ERROR_NOT_READY;

            continue;
        }

        report.ik_success++;

        g_kin_perf_sink ^= (uint32_t)solutions.count;

        /*
         * IK solution selector benchmark
         */
        start = timer_driver_get_counter();

        for (
            repeat = 0U;
            repeat < KIN_PERF_REPETITIONS;
            ++repeat
        )
        {
            selection_status =
                ik_solution_selector_select(
                    &solutions,
                    &current,
                    &selected
                );
        }

        end = timer_driver_get_counter();
        ticks = sample_ticks(start, end);

        g_kin_perf_samples[index].selection_ticks = ticks;
        g_kin_perf_samples[index].selection_status =
            (int32_t)selection_status;

        record_ticks(&report.selection, ticks);

        if (selection_status == ROBOT_STATUS_OK)
        {
            report.selection_success++;

            g_kin_perf_sink ^=
                (uint32_t)selected.candidate_index;
            /* End-to-end verification, outside the benchmark time window. */
            {
                robot_joint_angles_t chosen = {{0}};
                robot_transform_t reconstructed = {{{0}}};
                robot_status_t rt_status;
                float dist2 = 0.0F;
                float max_r = 0.0F;
                uint32_t row, col;
                for (j = 0U; j < ROBOT_JOINT_COUNT; ++j)
                {
                    int32_t raw = selected.target_positions.value[j];
                    raw %= ROBOT_JOINT_FULL_TURN_RAW;
                    if (raw >= ROBOT_JOINT_HALF_TURN_RAW)
                        raw -= ROBOT_JOINT_FULL_TURN_RAW;
                    if (raw < -ROBOT_JOINT_HALF_TURN_RAW)
                        raw += ROBOT_JOINT_FULL_TURN_RAW;
                    chosen.value[j] = (robot_joint_angle_t)raw;
                }
                rt_status = kinematics_forward(&chosen, &reconstructed);
                g_kin_perf_samples[index].roundtrip_status = (int32_t)rt_status;
                if (rt_status == ROBOT_STATUS_OK)
                {
                    for (row = 0U; row < 3U; ++row)
                    {
                        float delta = reconstructed.matrix[row][3] - target.matrix[row][3];
                        dist2 += delta * delta;
                        for (col = 0U; col < 3U; ++col)
                        {
                            float delta_r = fabsf(reconstructed.matrix[row][col]
                                                  - target.matrix[row][col]);
                            if (delta_r > max_r) max_r = delta_r;
                        }
                    }
                    g_kin_perf_samples[index].roundtrip_position_error_mm = sqrtf(dist2);
                    g_kin_perf_samples[index].roundtrip_rotation_max_element_error = max_r;
                    if (sqrtf(dist2) > report.worst_position_error_mm)
                        report.worst_position_error_mm = sqrtf(dist2);
                    if (max_r > report.worst_rotation_max_element_error)
                        report.worst_rotation_max_element_error = max_r;
                }
                /* Matrix-entry bound is not an angle bound. */
                if (rt_status == ROBOT_STATUS_OK
                    && dist2 <= 0.25F && max_r <= 0.003F)
                    report.roundtrip_passed++;
                else
                    report.roundtrip_failed++;
            }
        }
        else
        {
            report.roundtrip_skipped++;
        }
    }

    /*
     * Save report for GDB inspection.
     */
    g_kin_perf_report.sample_count =
        report.sample_count;

    g_kin_perf_report.repetitions =
        report.repetitions;

    g_kin_perf_report.timer_frequency_hz =
        report.timer_frequency_hz;

    g_kin_perf_report.timer_probe_ticks =
        report.timer_probe_ticks;

    g_kin_perf_report.timer_read_pair_ticks =
        report.timer_read_pair_ticks;

    g_kin_perf_report.fk_success =
        report.fk_success;

    g_kin_perf_report.ik_success =
        report.ik_success;

    g_kin_perf_report.selection_success =
        report.selection_success;

    g_kin_perf_report.fk = report.fk;
    g_kin_perf_report.ik = report.ik;
    g_kin_perf_report.selection = report.selection;

    g_kin_perf_finished = 1U;

    qemu_kinematics_perf_done();

    return 0;
}
