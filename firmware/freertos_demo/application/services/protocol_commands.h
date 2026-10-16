
#ifndef PROTOCOL_COMMANDS_H
#define PROTOCOL_COMMANDS_H

/*
 * Application protocol command handling.
 *
 * Responsibilities:
 * - Dispatch complete protocol frames.
 * - Handle joint-state feedback and ACK.
 * - Handle runtime parameter requests.
 * - Handle diagnostic requests.
 *
 * This interface does not depend on FreeRTOS types.
 * The caller supplies the current tick and tick frequency.
 */

#include <stdint.h>

#include "protocol.h"

/*
 * Feedback timing state.
 *
 * The state belongs to the receiving task, but its
 * contents are managed by the command handler.
 *
 * Tick values are unsigned 32-bit counters.
 */
typedef struct
{
    uint32_t previous_feedback_tick;
    uint8_t feedback_time_valid;

} protocol_command_context_t;


/*
 * Initialize command-handling state.
 *
 * A NULL context is ignored.
 */
void protocol_commands_init(
    protocol_command_context_t *context
);


/*
 * Dispatch a complete parsed protocol frame.
 *
 * frame:
 *   Parsed protocol frame.
 *
 * context:
 *   Persistent state for the receiving task.
 *
 * current_tick:
 *   Current monotonic RTOS tick counter.
 *
 * tick_rate_hz:
 *   Number of ticks per second. Must be nonzero
 *   for joint-state feedback processing.
 *
 * Unknown commands are ignored.
 */
void protocol_commands_dispatch_frame(
    const protocol_frame_t *frame,
    protocol_command_context_t *context,
    uint32_t current_tick,
    uint32_t tick_rate_hz
);

#endif
