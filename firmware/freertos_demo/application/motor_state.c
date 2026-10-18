#include <stddef.h>
#include <stdint.h>
#include <limits.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "motor_state.h"

static SemaphoreHandle_t motor_state_mutex = NULL;
static uint32_t last_accepted_tick = 0U;

robot_status_t motor_state_init(void)
{
    if (motor_state_mutex != NULL)
    {
        return ROBOT_STATUS_OK;
    }

    motor_state_mutex = xSemaphoreCreateMutex();
    if (motor_state_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    last_accepted_tick = 0U;
    return ROBOT_STATUS_OK;
}

robot_status_t motor_state_update_feedback(
    const robot_joint_angles_t *feedback,
    robot_real_t delta_time_s,
    uint32_t received_tick
)
{
    robot_status_t status;

    if (feedback == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }
    if (motor_state_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }
    if (xSemaphoreTake(motor_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    status = motor_driver_update_feedback(feedback, delta_time_s);
    if (status == ROBOT_STATUS_OK)
    {
        last_accepted_tick = received_tick;
    }

    (void)xSemaphoreGive(motor_state_mutex);
    return status;
}

robot_status_t motor_state_get_feedback(motor_state_observation_t *observation)
{
    motor_feedback_snapshot_t feedback;
    robot_status_t status;
    uint32_t tick;

    if (observation == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }
    if (motor_state_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }
    if (xSemaphoreTake(motor_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    status = motor_driver_get_feedback_snapshot(&feedback);
    tick = last_accepted_tick;
    (void)xSemaphoreGive(motor_state_mutex);

    if (status == ROBOT_STATUS_OK)
    {
        observation->feedback = feedback;
        observation->accepted_tick = tick;
    }
    return status;
}

robot_status_t motor_state_set_targets(const robot_joint_angles_t *targets)
{
    robot_status_t status;

    if (targets == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }
    if (motor_state_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }
    if (xSemaphoreTake(motor_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    status = motor_driver_set_target_positions(targets);
    (void)xSemaphoreGive(motor_state_mutex);
    return status;
}

robot_status_t motor_state_get_targets(robot_joint_angles_t *targets)
{
    robot_status_t status;

    if (targets == NULL)
    {
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    }
    if (motor_state_mutex == NULL)
    {
        return ROBOT_STATUS_ERROR_NOT_READY;
    }
    if (xSemaphoreTake(motor_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        return ROBOT_STATUS_ERROR_INTERNAL;
    }

    status = motor_driver_get_target_positions(targets);
    (void)xSemaphoreGive(motor_state_mutex);
    return status;
}

uint8_t motor_state_feedback_is_fresh(
    const motor_state_observation_t *observation,
    uint32_t now_tick,
    uint32_t max_age_ticks
)
{
    uint32_t elapsed;

    if (observation == NULL || max_age_ticks == 0U ||
        max_age_ticks > (uint32_t)INT32_MAX ||
        observation->feedback.sample_sequence == 0U)
    {
        return 0U;
    }

    elapsed = now_tick - observation->accepted_tick;
    return (elapsed <= max_age_ticks) ? 1U : 0U;
}
