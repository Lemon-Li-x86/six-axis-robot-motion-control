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
 * @brief 基于最新测量的关节位置生成一段 APF/IK 运动提案。
 * @param[in] measured 连续关节位置，0.01°。
 * @param[in] request APF 目标、障碍和策略参数。
 * @param[in] limits 关节运动约束与 FK 残差门限。
 * @param[out] out 提案数据，仅成功时写入。
 * @return ROBOT_STATUS_OK 或相应计算状态码。
 *
 * @note Trajectory Task 调用此接口；提案与运动执行是独立步骤。
 */
robot_status_t stage5_apf_propose_from_feedback(
    const robot_joint_positions_t *measured,
    const protocol_apf_step_request_t *request,
    const motion_stage4_config_t *limits,
    stage5_apf_proposal_t *out);
#endif
