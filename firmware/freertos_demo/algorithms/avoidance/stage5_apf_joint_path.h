/**
 * @file stage5_apf_joint_path.h
 * @brief 关节路线转换接口。
 */
#ifndef STAGE5_APF_JOINT_PATH_H
#define STAGE5_APF_JOINT_PATH_H
#include <stddef.h>
#include "potential_field_path.h"
#include "trajectory.h"
#include "robot_types.h"
#include "error_code.h"

typedef struct {
    robot_joint_positions_t target;
    /* Minimum rest-to-rest travel time from the existing trapezoid planner.
     * Units: seconds; no implicit 20ms-per-APF-point execution. */
    double segment_duration_s;
} stage5_apf_joint_segment_t;

/**
 * @brief 将笛卡尔 APF 航点转换为关节目标和梯形速度段。
 * @param[in] start_joints 初始连续关节位置，0.01°。
 * @param[in] cartesian_points 航点数组，base_link 坐标系，mm。
 * @param[in] point_count 航点数量，允许为 0。
 * @param[in] max_velocity_deg_s 六轴最大角速度，°/s。
 * @param[in] max_acceleration_deg_s2 六轴最大角加速度，°/s²。
 * @param[in] max_fk_position_error_mm FK 位置残差上限，mm。
 * @param[in] max_fk_rotation_element_error 旋转矩阵元素残差上限。
 * @param[out] segments 输出关节段，失败时保持原值。
 * @param[in] capacity 输出缓冲容量。
 * @param[out] out_count 已生成关节段数量，失败时为 0。
 * @return ROBOT_STATUS_OK 或 FK/IK、限位、容量等状态码。
 *
 * @note 航点姿态采用起点 FK；两阶段转换保证输出原子性。
 *       关节段执行前仍由调用层完成全连杆扫掠检查。
 */

robot_status_t stage5_apf_points_to_joint_route(
    const robot_joint_positions_t *start_joints,
    const robot_apf_point_t *cartesian_points,
    size_t point_count,
    const robot_joint_velocities_t *max_velocity_deg_s,
    const trajectory_joint_accelerations_t *max_acceleration_deg_s2,
    robot_real_t max_fk_position_error_mm,
    robot_real_t max_fk_rotation_element_error,
    stage5_apf_joint_segment_t *segments,
    size_t capacity,
    size_t *out_count);
/**
 * @brief 读取生产 IK 的位置和旋转残差验收上限。
 * @param[out] maximum_position_error_mm 最大位置残差，mm。
 * @param[out] maximum_rotation_element_error 最大旋转矩阵元素差。
 * @return ROBOT_STATUS_OK 或参数状态码。
 */

robot_status_t stage5_apf_joint_pose_limits(
    robot_real_t *maximum_position_error_mm,
    robot_real_t *maximum_rotation_element_error);

#endif
