/*
 * Stage 4 RTOS-facing request queue and task startup.
 * Data remains owned by callers; work is performed only in tasks.
 */
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motion_pipeline.h"
#include "motion_tasks_internal.h"
#include "motion_limits.h"
#include "joint_controller.h"
#include "task_config.h"

motion_runtime_t g_motion_runtime={0};

void motion_record_fault(robot_status_t status,uint8_t is_controller)
{
    taskENTER_CRITICAL();
    if(is_controller) g_motion_runtime.diagnostics.controller_faults++;
    else g_motion_runtime.diagnostics.planner_faults++;
    g_motion_runtime.diagnostics.last_fault=status;
    taskEXIT_CRITICAL();
}

robot_status_t motion_pipeline_start(const motion_stage4_config_t *cfg)
{
    motion_runtime_t *r=&g_motion_runtime;
    joint_controller_t validation;
    robot_status_t s;
    uint32_t j;
    if(cfg==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(r->request_queue!=NULL || r->reference_queue!=NULL)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    if(cfg->feedback_timeout_ms==0U ||
       cfg->feedback_timeout_ms > 0x7FFFFFFFU ||
       cfg->reference_timeout_ms==0U ||
       cfg->reference_timeout_ms>0x7FFFFFFFU ||
       !(cfg->cartesian_start_tolerance_mm>0.0F) ||
       !isfinite((double)cfg->cartesian_start_tolerance_mm) ||
       !(cfg->cartesian_start_rotation_matrix_tolerance>0.0F) ||
       !isfinite((double)cfg->cartesian_start_rotation_matrix_tolerance) ||
       !(cfg->cartesian_waypoint_position_tolerance_mm>0.0F) ||
       !isfinite((double)cfg->cartesian_waypoint_position_tolerance_mm) ||
       !(cfg->cartesian_waypoint_rotation_matrix_tolerance>0.0F) ||
       !isfinite((double)cfg->cartesian_waypoint_rotation_matrix_tolerance))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    s=joint_controller_init(&validation,cfg->pid);
    if(s!=ROBOT_STATUS_OK) return s;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
        if(!isfinite((double)cfg->max_velocity_deg_s.value[j]) ||
           !isfinite((double)cfg->max_acceleration_deg_s2.value[j]) ||
           !(cfg->max_velocity_deg_s.value[j]>0.0F) ||
           !(cfg->max_acceleration_deg_s2.value[j]>0.0F))
            return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    }
    r->config=*cfg;
    memset(&r->diagnostics,0,sizeof(r->diagnostics));
    r->request_queue=xQueueCreate(1U,sizeof(motion_request_t));
    r->reference_queue=xQueueCreate(1U,sizeof(motion_reference_t));
    if(r->request_queue==NULL || r->reference_queue==NULL)
        goto fail;
    if(trajectory_task_create()!=pdPASS) goto fail;
    if(control_task_create()!=pdPASS) goto fail;
    if(monitor_task_create()!=pdPASS) goto fail;
    return ROBOT_STATUS_OK;
fail:
    if(r->monitor_handle!=NULL) vTaskDelete(r->monitor_handle);
    if(r->control_handle!=NULL) vTaskDelete(r->control_handle);
    if(r->trajectory_handle!=NULL) vTaskDelete(r->trajectory_handle);
    if(r->request_queue!=NULL) vQueueDelete(r->request_queue);
    if(r->reference_queue!=NULL) vQueueDelete(r->reference_queue);
    memset(r,0,sizeof(*r));
    return ROBOT_STATUS_ERROR_INTERNAL;
}

robot_status_t motion_pipeline_submit_joint(const robot_joint_positions_t *goal)
{
    motion_request_t request;
    if(goal==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(g_motion_runtime.request_queue==NULL)
        return ROBOT_STATUS_ERROR_NOT_READY;
    if(motion_limits_check_positions(goal)!=ROBOT_STATUS_OK)
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    memset(&request,0,sizeof(request));
    request.kind=MOTION_REQUEST_JOINT;
    request.joint_goal=*goal;
    if(xQueueOverwrite(g_motion_runtime.request_queue,&request)!=pdPASS)
        return ROBOT_STATUS_ERROR_INTERNAL;
    taskENTER_CRITICAL();
    g_motion_runtime.diagnostics.requests_accepted++;
    taskEXIT_CRITICAL();
    return ROBOT_STATUS_OK;
}

robot_status_t motion_pipeline_submit_polynomial(
    const trajectory_polynomial_plan_t *plan)
{
    motion_request_t request;
    uint32_t j;
    if(plan==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(g_motion_runtime.request_queue==NULL)
        return ROBOT_STATUS_ERROR_NOT_READY;
    if((plan->degree!=3U && plan->degree!=5U) ||
       !(plan->duration_s>0.0) || !isfinite(plan->duration_s))
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    if(motion_limits_check_positions(&plan->start_positions)!=ROBOT_STATUS_OK ||
       motion_limits_check_positions(&plan->goal_positions)!=ROBOT_STATUS_OK)
        return ROBOT_STATUS_ERROR_OUT_OF_RANGE;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
        uint32_t k;
        for(k=0U;k<6U;k++) {
            if(!isfinite(plan->coefficients_deg[j][k]))
                return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
        }
    }
    memset(&request,0,sizeof(request));
    request.kind=MOTION_REQUEST_POLYNOMIAL;
    request.polynomial=*plan;
    if(xQueueOverwrite(g_motion_runtime.request_queue,&request)!=pdPASS)
        return ROBOT_STATUS_ERROR_INTERNAL;
    taskENTER_CRITICAL();
    g_motion_runtime.diagnostics.requests_accepted++;
    taskEXIT_CRITICAL();
    return ROBOT_STATUS_OK;
}

robot_status_t motion_pipeline_submit_cartesian(
    const cartesian_trajectory_plan_t *plan)
{
    motion_request_t request;
    if(plan==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(g_motion_runtime.request_queue==NULL)
        return ROBOT_STATUS_ERROR_NOT_READY;
    if((plan->kind!=CARTESIAN_PATH_LINE &&
        plan->kind!=CARTESIAN_PATH_ARC) ||
       plan->interval_count==0U || !(plan->duration_s>0.0) ||
       !isfinite(plan->period_s) ||
       fabs(plan->period_s-((double)TASK_PERIOD_TRAJECTORY_MS/1000.0))>1e-9)
        return ROBOT_STATUS_ERROR_INVALID_ARGUMENT;
    memset(&request,0,sizeof(request));
    request.kind=MOTION_REQUEST_CARTESIAN;
    request.cartesian=*plan;
    if(xQueueOverwrite(g_motion_runtime.request_queue,&request)!=pdPASS)
        return ROBOT_STATUS_ERROR_INTERNAL;
    taskENTER_CRITICAL();
    g_motion_runtime.diagnostics.requests_accepted++;
    taskEXIT_CRITICAL();
    return ROBOT_STATUS_OK;
}

robot_status_t motion_pipeline_get_diagnostics(motion_stage4_diagnostics_t *out)
{
    if(out==NULL) return ROBOT_STATUS_ERROR_NULL_POINTER;
    if(g_motion_runtime.request_queue==NULL)
        return ROBOT_STATUS_ERROR_NOT_READY;
    taskENTER_CRITICAL();
    *out=g_motion_runtime.diagnostics;
    taskEXIT_CRITICAL();
    return ROBOT_STATUS_OK;
}
