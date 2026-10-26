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
    motion_stage4_diagnostics_t diagnostics={0};
    robot_status_t status=ROBOT_STATUS_OK;
    TickType_t last_wake=xTaskGetTickCount();
    const TickType_t cadence=pdMS_TO_TICKS(TASK_PERIOD_CONTROL_MS);
    const TickType_t total=pdMS_TO_TICKS(1200U);
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

    g_stage4_rtos_test_report.last_error=(int32_t)status;
    if(motion_pipeline_get_diagnostics(&diagnostics)!=ROBOT_STATUS_OK)
        g_stage4_rtos_test_report.failed++;
    else
    {
        g_stage4_rtos_test_report.requests_accepted=
            diagnostics.requests_accepted;
        g_stage4_rtos_test_report.trajectory_samples=
            diagnostics.trajectory_samples;
        g_stage4_rtos_test_report.controller_steps=
            diagnostics.controller_steps;
        g_stage4_rtos_test_report.monitoring_ticks=
            diagnostics.monitoring_ticks;
        g_stage4_rtos_test_report.controller_faults=
            diagnostics.controller_faults;
        g_stage4_rtos_test_report.planner_faults=
            diagnostics.planner_faults;
        g_stage4_rtos_test_report.feedback_stale_count=
            diagnostics.feedback_stale_count;
        g_stage4_rtos_test_report.control_stack_free_words=
            diagnostics.control_stack_free_words;
        g_stage4_rtos_test_report.trajectory_stack_free_words=
            diagnostics.trajectory_stack_free_words;
        g_stage4_rtos_test_report.monitor_stack_free_words=
            diagnostics.monitor_stack_free_words;
    }
    report_check(status==ROBOT_STATUS_OK);
    report_check(diagnostics.requests_accepted==1U);
    report_check(diagnostics.trajectory_samples>0U);
    report_check(diagnostics.controller_steps>0U);
    report_check(diagnostics.monitoring_ticks>0U);
    report_check(diagnostics.controller_faults==0U);
    report_check(diagnostics.planner_faults==0U);
    report_check(diagnostics.feedback_stale_count==0U);
    report_check(diagnostics.control_stack_free_words>0U);
    report_check(diagnostics.trajectory_stack_free_words>0U);
    report_check(diagnostics.monitor_stack_free_words>0U);
    g_stage4_rtos_test_finished=1U;
    stage4_rtos_selftest_done();
}

BaseType_t stage4_rtos_selftest_start(void)
{
    return xTaskCreate(stage4_rtos_test_task,"S4RTOSCheck",
        TASK_STACK_DEPTH_MONITOR,NULL,TASK_PRIORITY_MONITOR,NULL);
}
