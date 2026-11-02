/**
 * @file potential_field_path.h
 * @brief APF 路线接口。
 */
#ifndef ROBOT_POTENTIAL_FIELD_PATH_H
#define ROBOT_POTENTIAL_FIELD_PATH_H
#include "potential_field.h"

/**
 * @brief 在固定容量缓冲区生成连续 APF 航点。
 * @param[in] start 起点，mm。
 * @param[in] goal 终点，mm。
 * @param[in] obstacles 静态障碍数组。
 * @param[in] obstacle_count 障碍数量。
 * @param[in] config APF 策略配置。
 * @param[out] points 逐步航点，失败时保持不变。
 * @param[in] capacity 航点缓冲容量。
 * @param[out] count 输出航点数量；已到目标时为 0，失败时为 0。
 * @return ROBOT_STATUS_OK 或相应错误码。
 *
 * @note 有界循环检测局部停滞；使用两阶段检查保持输出原子性。
 */

robot_status_t robot_apf_plan_path(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *points,
    size_t capacity,
    size_t *count);

/* 单个相交 AABB 的确定性绕行候选；应用层负责 IK 和全连杆校验。 */
#define ROBOT_APF_BYPASS_MAX_CANDIDATES 12U
/** @brief 查询生产 C 绕行候选缓冲区容量。 */
size_t robot_apf_bypass_capacity(void);
/**
 * @brief 生成受静态 AABB 约束的笛卡尔绕行候选点。
 * @param[in] start 当前点，mm。
 * @param[in] goal 目标点，mm。
 * @param[in] obstacle 静态障碍。
 * @param[in] obstacle_count 当前支持的障碍数量。
 * @param[in] config APF 余量与步长策略。
 * @param[out] via_candidates 候选航点缓冲。
 * @param[in] capacity 缓冲区容量。
 * @param[out] out_count 实际生成的候选数量。
 * @return ROBOT_STATUS_OK 或相应错误码。
 */
robot_status_t robot_apf_generate_bypass_vias(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacle,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *via_candidates,
    size_t capacity,
    size_t *out_count);

#endif
