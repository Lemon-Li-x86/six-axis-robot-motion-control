/**
 * @file stage5_apf_proposal.h
 * @brief FreeRTOS APF waypoint proposal interface.
 *
 * A proposal computes a joint target without commanding motion.
 * The caller validates swept-link clearance before issuing a separate move.
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

/**
 * @brief Compute one APF waypoint from measured joint positions.
 * @param measured Latest continuous joint positions.
 * @param request APF goal, obstacle and tuning parameters.
 * @param limits Joint motion constraints.
 * @param out Proposal, written only on success.
 * @return Robot status code.
 *
 * Called by the trajectory task; does not modify motor state.
 */
robot_status_t stage5_apf_propose_from_feedback(
    const robot_joint_positions_t *measured,
    const protocol_apf_step_request_t *request,
    const motion_stage4_config_t *limits,
    stage5_apf_proposal_t *out);
#endif
