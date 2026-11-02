/**
 * @file potential_field.h
 * @brief APF 基础配置/障碍输入接口。
 */
#ifndef ROBOT_POTENTIAL_FIELD_H
#define ROBOT_POTENTIAL_FIELD_H

#include <stddef.h>
#include "error_code.h"

#define ROBOT_APF_MAX_OBSTACLES 8U

typedef struct {
    double mm[3];
} robot_apf_point_t;

typedef struct {
    robot_apf_point_t min_corner;
    robot_apf_point_t max_corner;
} robot_apf_aabb_t;

typedef struct {
    double attraction_gain;
    double repulsion_gain;
    double influence_distance_mm;
    double clearance_mm;
    double maximum_step_mm;
    double goal_tolerance_mm;
} robot_apf_config_t;

typedef struct {
    double nearest_surface_distance_mm; /* +INFINITY if no obstacles */
    double chosen_step_mm;
    unsigned int reached_goal;
} robot_apf_step_info_t;

/**
 * @brief 计算避开静态 AABB 的下一段笛卡尔航点。
 * @param[in] current 当前末端点，base_link 坐标系，mm。
 * @param[in] goal 目标末端点，mm。
 * @param[in] obstacles 障碍 AABB 数组。
 * @param[in] obstacle_count 障碍数量。
 * @param[in] config 调用者提供的 APF 增益、余量、步长和容差。
 * @param[out] next 输出航点，失败时保持原值。
 * @param[out] details 步长与到达标记，失败时保持原值。
 * @return ROBOT_STATUS_OK 或相应参数、碰撞、无解状态码。
 *
 * @note 检查当前点、目标点和扫掠线段；这是末端点级规划，
 *       完整连杆碰撞检查在后续门禁执行。
 */

robot_status_t robot_apf_step(
    const robot_apf_point_t *current,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    const robot_apf_config_t *config,
    robot_apf_point_t *next,
    robot_apf_step_info_t *details);

/**
 * @brief 检测一段笛卡尔直线与膨胀后的静态 AABB 是否相交。
 * @param[in] start 起点，base_link 坐标系，mm。
 * @param[in] goal 终点，base_link 坐标系，mm。
 * @param[in] obstacles 障碍 AABB 数组。
 * @param[in] obstacle_count 障碍数量。
 * @param[in] clearance_mm 额外碰撞余量，mm。
 * @param[out] blocked 相交返回 1，否则为 0；失败时不修改。
 * @return ROBOT_STATUS_OK 或相应参数状态码。
 */

robot_status_t robot_apf_segment_blocked(
    const robot_apf_point_t *start,
    const robot_apf_point_t *goal,
    const robot_apf_aabb_t *obstacles,
    size_t obstacle_count,
    double clearance_mm,
    unsigned int *blocked);

#endif
