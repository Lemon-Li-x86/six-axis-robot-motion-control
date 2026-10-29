#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdint.h>

#include "error_code.h"
#include "motor_driver.h"

/*
 * Application-owned synchronization gateway for the Motor Driver.
 *
 * The driver stays FreeRTOS-independent. All task-time feedback/target
 * accesses must go through this module; calls before the scheduler may
 * initialize the driver and its initial targets directly.
 *
 * No API here is safe to call from an ISR.
 */
typedef struct
{
    motor_feedback_snapshot_t feedback;
    uint32_t accepted_tick;
} motor_state_observation_t;

/* Call once after motor_driver_init(), before starting application tasks. */
robot_status_t motor_state_init(void);

/* Only ProtocolRX currently writes feedback. Tick is captured on acceptance. */
robot_status_t motor_state_update_feedback(
    const robot_joint_angles_t *feedback,
    robot_real_t delta_time_s,
    uint32_t received_tick
);

/* Copy feedback and its acceptance tick under the same mutex. */
robot_status_t motor_state_get_feedback(motor_state_observation_t *observation);

/* Future Control Task must use these wrappers to read/write targets. */
robot_status_t motor_state_set_targets(const robot_joint_angles_t *targets);
robot_status_t motor_state_get_targets(robot_joint_angles_t *targets);

/*
 * Assess a previously copied observation without holding the mutex.
 * Clock wrap is supported for elapsed intervals less than half the
 * uint32_t tick range; callers must check freshness every control cycle.
 * max_age_ticks must be positive and <= INT32_MAX.
 * No automatic actuator stop or fault recovery exists yet.
 */
uint8_t motor_state_feedback_is_fresh(
    const motor_state_observation_t *observation,
    uint32_t now_tick,
    uint32_t max_age_ticks
);

/* Always take the RTOS clock sample AFTER copying the feedback snapshot.
 * Sampling time before acquiring the mutex can race ProtocolRX: its newer
 * accepted_tick then appears to be ~2^32 ticks old by unsigned subtraction.
 * This API returns NOT_READY for absent or stale feedback and never changes
 * the caller's output on failure. Task context only, never ISR. */
robot_status_t motor_state_get_fresh_feedback(
    motor_state_observation_t *observation,
    uint32_t max_age_ticks
);

#endif
