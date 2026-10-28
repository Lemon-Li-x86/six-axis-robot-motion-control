/*
 * Stage4 QEMU-only RTOS closed-loop TASK PATH smoke.
 *
 * Tests feedback -> request -> trajectory -> control PID -> monitor.
 * This uses an intentionally IMMOBILE plant (feedback remains fixed),
 * with ROBOT_ENABLE_STAGE4_COMMAND_OUTPUT forcibly OFF at CMake level.
 *
 * Does not claim dynamic tracking or UART acceptance; those require
 * PyBullet/native bridge and later end-to-end UART integration.
 *
 * Compiled ONLY under ROBOT_ENABLE_STAGE4_QEMU_MOTION_SELF_TEST. Never
 * enable this in deployable production builds.
 */
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "motion_pipeline.h"
#include "motor_state.h"
#include "task_config.h"
#include "stage4_motion_config.h"
#include "stage4_timing_probe.h"

typedef struct
{
    uint32_t requests_accepted;
    uint32_t trajectory_samples;
    uint32_t controller_steps;
    uint32_t monitoring_ticks;
    uint32_t controller_faults;
    uint32_t planner_faults;
    uint32_t feedback_stale_count;
    uint32_t control_stack_free_words;
    uint32_t trajectory_stack_free_words;
    uint32_t monitor_stack_free_words;
    uint32_t checks;
    uint32_t failed;
    int32_t last_error;
} stage4_rtos_test_report_t;

volatile stage4_rtos_test_report_t g_stage4_rtos_test_report = {0};
volatile uint32_t g_stage4_rtos_test_finished = 0U;
volatile uint32_t g_stage4_rtos_healthy_duration_ms =
    ROBOT_STAGE4_QEMU_HEALTHY_RUN_MS;

/* Independent final phase; the original 29-check report and timing window
 * are captured BEFORE the deliberate polynomial planner rejection. */
typedef struct {
    uint32_t checks;
    uint32_t failed;
    uint32_t rejected_before_publish;
    uint32_t valid_execution_started;
    uint32_t trajectory_stack_free_words;
} polynomial_rtos_report_t;
volatile polynomial_rtos_report_t g_stage4_polynomial_rtos_report={0};

static void polynomial_check(uint8_t condition)
{
    g_stage4_polynomial_rtos_report.checks++;
    if(!condition) g_stage4_polynomial_rtos_report.failed++;
}

/* Keep measured feedback fresh while actual Trajectory/Control tasks run. */
static robot_status_t polynomial_feed_for(
    const robot_joint_angles_t *starting,uint32_t duration_ms)
{
    TickType_t begin=xTaskGetTickCount(),wake=begin;
    TickType_t period=pdMS_TO_TICKS(TASK_PERIOD_CONTROL_MS);
    for(;;) {
        TickType_t now=xTaskGetTickCount();
        robot_status_t s=motor_state_update_feedback(starting,
            (robot_real_t)TASK_PERIOD_CONTROL_MS/1000.0F,(uint32_t)now);
        if(s!=ROBOT_STATUS_OK) return s;
        if((TickType_t)(now-begin)>=pdMS_TO_TICKS(duration_ms)) break;
        vTaskDelayUntil(&wake,period==0U?1U:period);
    }
    return ROBOT_STATUS_OK;
}

static void polynomial_task_path_test(const robot_joint_angles_t *starting)
{
    robot_joint_positions_t start={{0}},goal={{0}};
    robot_joint_velocities_t velocity={{0}};
    trajectory_joint_accelerations_t acceleration={{0}};
    trajectory_polynomial_plan_t plan={0};
    motion_stage4_diagnostics_t before={0},rejected={0},running={0};
    robot_status_t status;
    uint32_t j;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) start.value[j]=starting->value[j];
    goal=start; goal.value[1]+=100;
    (void)polynomial_feed_for(starting,TASK_PERIOD_TRAJECTORY_MS*3U);
    (void)motion_pipeline_get_diagnostics(&before);

    /* Valid endpoints, zero endpoint rates, but unsafe interior rates. */
    status=trajectory_polynomial_init_quintic(&plan,&start,&goal,
        &velocity,&velocity,&acceleration,&acceleration,0.001);
    polynomial_check(status==ROBOT_STATUS_OK);
    status=motion_pipeline_submit_polynomial(&plan);
    polynomial_check(status==ROBOT_STATUS_OK);
    status=polynomial_feed_for(starting,TASK_PERIOD_TRAJECTORY_MS*5U);
    polynomial_check(status==ROBOT_STATUS_OK);
    status=motion_pipeline_get_diagnostics(&rejected);
    g_stage4_polynomial_rtos_report.rejected_before_publish=
        status==ROBOT_STATUS_OK &&
        rejected.planner_faults==before.planner_faults+1U &&
        rejected.last_fault==ROBOT_STATUS_ERROR_OUT_OF_RANGE &&
        rejected.trajectory_samples==before.trajectory_samples &&
        rejected.controller_steps==before.controller_steps;
    polynomial_check(g_stage4_polynomial_rtos_report.rejected_before_publish);

    /* A safe replacement must still start after the rejected request. */
    status=trajectory_polynomial_init_quintic(&plan,&start,&goal,
        &velocity,&velocity,&acceleration,&acceleration,1.0);
    polynomial_check(status==ROBOT_STATUS_OK);
    status=motion_pipeline_submit_polynomial(&plan);
    polynomial_check(status==ROBOT_STATUS_OK);
    status=polynomial_feed_for(starting,TASK_PERIOD_TRAJECTORY_MS*10U);
    if(status==ROBOT_STATUS_OK) status=motion_pipeline_get_diagnostics(&running);
    g_stage4_polynomial_rtos_report.valid_execution_started=
        status==ROBOT_STATUS_OK &&
        running.planner_faults==rejected.planner_faults &&
        running.controller_faults==rejected.controller_faults &&
        running.trajectory_samples>rejected.trajectory_samples &&
        running.controller_steps>rejected.controller_steps;
    polynomial_check(g_stage4_polynomial_rtos_report.valid_execution_started);
    g_stage4_polynomial_rtos_report.trajectory_stack_free_words=
        running.trajectory_stack_free_words;
    polynomial_check(running.trajectory_stack_free_words>=64U);
}

static void report_check(uint8_t condition)
{
    g_stage4_rtos_test_report.checks++;
    if (condition == 0U)
        g_stage4_rtos_test_report.failed++;
}

__attribute__((noinline))
void stage4_rtos_selftest_done(void)
{
    for(;;)
        __asm volatile ("nop");
}

static void stage4_rtos_test_task(void *argument)
{
    robot_joint_angles_t starting = {{3000,-4500,6000,2000,500,-1000}};
    robot_joint_positions_t goal = {{3000,-4400,6000,2000,500,-1000}};
    motion_stage4_diagnostics_t healthy={0};
    motion_stage4_diagnostics_t stale={0};
    motion_stage4_diagnostics_t after_stale={0};
    robot_status_t status=ROBOT_STATUS_OK;
    TickType_t last_wake=xTaskGetTickCount();
    const TickType_t cadence=pdMS_TO_TICKS(TASK_PERIOD_CONTROL_MS);
    const TickType_t total=pdMS_TO_TICKS(ROBOT_STAGE4_QEMU_HEALTHY_RUN_MS);
    TickType_t begin=xTaskGetTickCount();
    uint8_t submitted=0U;

    (void)argument;
    /*
     * This task is the only feedback writer in the QEMU serial-null test.
     * No position response is emulated: verify RTOS control computation,
     * not closed-loop tracking or actual motor target actuation.
     */
    for(;;)
    {
        TickType_t now=xTaskGetTickCount();
        status=motor_state_update_feedback(
            &starting,
            (robot_real_t)TASK_PERIOD_CONTROL_MS/1000.0F,
            (uint32_t)now);
        if(status!=ROBOT_STATUS_OK) break;
        if(submitted==0U)
        {
            status=motion_pipeline_submit_joint(&goal);
            if(status!=ROBOT_STATUS_OK) break;
            submitted=1U;
        }
        if((TickType_t)(now-begin)>=total) break;
        vTaskDelayUntil(&last_wake,(cadence==0U)?1U:cadence);
    }

    /*
     * Phase 2: stop ALL simulated feedback. Hold beyond the same
     * application-configured feedback timeout, without inventing a second
     * timeout parameter. Trajectory/Control must fault and stop producing
     * new commands even when the requested path hasn't converged.
     */
    if(status==ROBOT_STATUS_OK)
    {
        const motion_stage4_config_t *cfg=stage4_motion_default_config();
        if(motion_pipeline_get_diagnostics(&healthy)!=ROBOT_STATUS_OK)
            status=ROBOT_STATUS_ERROR_INTERNAL;
        else
        {
            /* Freeze the configurable healthy-window timing sample before
             * inducing the intentional stale-feedback safety stop. */
            stage4_timing_capture();
            vTaskDelay(pdMS_TO_TICKS(
                cfg->feedback_timeout_ms+
                (5U*TASK_PERIOD_TRAJECTORY_MS)));
            if(motion_pipeline_get_diagnostics(&stale)!=ROBOT_STATUS_OK)
                status=ROBOT_STATUS_ERROR_INTERNAL;
            else
            {
                vTaskDelay(pdMS_TO_TICKS(5U*TASK_PERIOD_TRAJECTORY_MS));
                if(motion_pipeline_get_diagnostics(&after_stale)!=ROBOT_STATUS_OK)
                    status=ROBOT_STATUS_ERROR_INTERNAL;
            }
        }
    }

    g_stage4_rtos_test_report.last_error=(int32_t)status;
    g_stage4_rtos_test_report.requests_accepted=
        after_stale.requests_accepted;
    g_stage4_rtos_test_report.trajectory_samples=
        after_stale.trajectory_samples;
    g_stage4_rtos_test_report.controller_steps=
        after_stale.controller_steps;
    g_stage4_rtos_test_report.monitoring_ticks=
        after_stale.monitoring_ticks;
    g_stage4_rtos_test_report.controller_faults=
        after_stale.controller_faults;
    g_stage4_rtos_test_report.planner_faults=
        after_stale.planner_faults;
    g_stage4_rtos_test_report.feedback_stale_count=
        after_stale.feedback_stale_count;
    g_stage4_rtos_test_report.control_stack_free_words=
        after_stale.control_stack_free_words;
    g_stage4_rtos_test_report.trajectory_stack_free_words=
        after_stale.trajectory_stack_free_words;
    g_stage4_rtos_test_report.monitor_stack_free_words=
        after_stale.monitor_stack_free_words;

    /* Healthy phase: real tasks ran without faults, safe stack margins. */
    report_check(status==ROBOT_STATUS_OK);
    report_check(healthy.requests_accepted==1U);
    report_check(healthy.trajectory_samples>0U);
    report_check(healthy.controller_steps>0U);
    report_check(healthy.monitoring_ticks>0U);
    report_check(healthy.controller_faults==0U);
    report_check(healthy.planner_faults==0U);
    report_check(healthy.feedback_stale_count==0U);
    report_check(healthy.control_stack_free_words>0U);
    report_check(healthy.trajectory_stack_free_words>0U);
    report_check(healthy.monitor_stack_free_words>0U);

    /* Fault phase: at least one safety guardian reports loss of feedback
     * and command computation no longer advances after shutdown. */
    report_check(
        (after_stale.controller_faults+after_stale.planner_faults)>0U);
    report_check(
        stale.controller_steps==after_stale.controller_steps);

    /* Tick counts are Timer0 25MHz-equivalent QEMU virtual time, not
     * physical WCET and NOT the GitHub runner's wall-clock latency. */
    report_check(g_stage4_timing_frequency_hz>0U);
    {
        uint32_t i;
        for(i=0U;i<STAGE4_TIMING_TASK_COUNT;i++) {
            const volatile stage4_timing_summary_t *timing=
                &g_stage4_timing_summary[i];
            report_check(timing->interval_sample_count>0U);
            report_check(timing->execution_sample_count>0U);
            report_check(timing->expected_period_counter_ticks>0U);
            report_check(timing->p99_abs_period_jitter_ticks<=
                         timing->maximum_abs_period_jitter_ticks);
            report_check(timing->p99_processing_ticks<=
                         timing->maximum_processing_ticks);
        }
    }
    polynomial_task_path_test(&starting);
    g_stage4_rtos_test_finished=1U;
    stage4_rtos_selftest_done();
}

BaseType_t stage4_rtos_selftest_start(void)
{
    return xTaskCreate(stage4_rtos_test_task,"S4RTOSCheck",
        TASK_STACK_DEPTH_TRAJECTORY,NULL,TASK_PRIORITY_MONITOR,NULL);
}
