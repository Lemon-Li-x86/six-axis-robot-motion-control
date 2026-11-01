/* Simulation-only on-firmware APF planning: C waypoint + C IK, NO actuation.
 * Physical all-link collision approval is REQUIRED before a separate
 * CMD_REQUEST_JOINT_MOVE. Never use this proposal alone on real hardware.
 */
#ifndef STAGE5_APF_PROPOSAL_H
#define STAGE5_APF_PROPOSAL_H
#include "protocol.h"
#include "motion_pipeline.h"
#include "potential_field.h"

typedef struct {
    robot_apf_point_t cartesian_mm;
    robot_joint_angles_t canonical_joint;
    uint8_t goal_reached;
} stage5_apf_proposal_t;

/* Called from FreeRTOS Trajectory task, not from 256-word ProtocolRX stack.
 * Reads no mutable globals and writes *out only after all safety checks.
 * Caller supplies fresh observed continuous joint state and validated limits.
 */
robot_status_t stage5_apf_propose_from_feedback(
    const robot_joint_positions_t *measured,
    const protocol_apf_step_request_t *request,
    const motion_stage4_config_t *limits,
    stage5_apf_proposal_t *out);
#endif
