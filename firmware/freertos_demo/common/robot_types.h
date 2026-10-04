/*
 * 文件：robot_types.h
 *
 * 用途：
 * 定义六轴机器人各模块共同使用的核心数据类型。
 *
 * 本文件统一：
 *
 * 1. 关节数量；
 * 2. 关节角数据格式、单位与规范范围；
 * 3. 连续关节位置数据格式；
 * 4. 关节速度数据格式；
 * 5. 笛卡尔位姿数据格式；
 * 6. 公共坐标系约定；
 * 7. 4 × 4 齐次变换矩阵格式。
 *
 * Driver、Communication、Algorithm、Control 等模块
 * 应优先使用这里定义的公共类型。
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>


/* =========================================================
 * 六轴机器人基本配置
 * ========================================================= */

#define ROBOT_JOINT_COUNT 6U


/* =========================================================
 * 通用计算标量
 * ========================================================= */

/*
 * 运动学、轨迹规划和 PID 内部计算
 * 统一使用单精度浮点数。
 */
typedef float robot_real_t;


/* =========================================================
 * 关节角公共表示
 * ========================================================= */

/*
 * 外部关节角统一采用 int16_t 定点表示：
 *
 * 1 unit = 0.01 degree
 *
 * 例如：
 *
 * 6000  =  60.00°
 * -9000 = -90.00°
 *
 * 公共关节角采用 Canonical Angle：
 *
 * [-180°, 180°)
 *
 * 即：
 *
 * -180.00° <= angle < 180.00°
 *
 * 因此：
 *
 * +190° -> -170°
 * +350° ->  -10°
 * +180° -> -180°
 *
 * 该表示用于：
 *
 * 1. UART Protocol；
 * 2. Algorithm Layer 的关节角输入输出；
 * 3. Motor Driver 的规范化位置接口。
 *
 * 它描述关节当前“角度状态”，
 * 不直接描述累计旋转圈数。
 */
typedef int16_t robot_joint_angle_t;


typedef struct
{
    robot_joint_angle_t value[
        ROBOT_JOINT_COUNT
    ];

} robot_joint_angles_t;


/*
 * 原始关节角转 degree：
 *
 * degree =
 * raw × ROBOT_JOINT_ANGLE_UNIT_DEG
 */
#define ROBOT_JOINT_ANGLE_UNIT_DEG 0.01F


/*
 * Canonical Angle 的整数范围。
 *
 * 单位：
 * 0.01 degree
 */
#define ROBOT_JOINT_FULL_TURN_RAW 36000L

#define ROBOT_JOINT_HALF_TURN_RAW 18000L

#define ROBOT_JOINT_ANGLE_MIN_RAW \
    (-ROBOT_JOINT_HALF_TURN_RAW)

#define ROBOT_JOINT_ANGLE_MAX_RAW \
    (ROBOT_JOINT_HALF_TURN_RAW - 1L)


/* =========================================================
 * 连续关节位置
 * ========================================================= */

/*
 * 连续关节位置使用 int32_t。
 *
 * 单位仍然为：
 *
 * 1 unit = 0.01 degree
 *
 * 与 robot_joint_angle_t 不同，
 * 该类型不在 ±180° 处回绕。
 *
 * 例如实际连续运动：
 *
 * 170°
 * 179°
 * 181°
 * 190°
 *
 * Canonical Angle 可能表现为：
 *
 * 170°
 * 179°
 * -179°
 * -170°
 *
 * 而 Continuous Position 保留：
 *
 * 170°
 * 179°
 * 181°
 * 190°
 *
 * 主要供：
 *
 * Motor Driver、
 * Control Layer、
 * Trajectory Planning
 *
 * 使用。
 *
 * 当前 UART Protocol 不直接传输此类型。
 */
typedef int32_t robot_joint_position_t;


typedef struct
{
    robot_joint_position_t value[
        ROBOT_JOINT_COUNT
    ];

} robot_joint_positions_t;


/* =========================================================
 * 关节速度
 * ========================================================= */

/*
 * 单位：
 *
 * degree / second
 */
typedef robot_real_t robot_joint_velocity_t;


typedef struct
{
    robot_joint_velocity_t value[
        ROBOT_JOINT_COUNT
    ];

} robot_joint_velocities_t;


/* =========================================================
 * 笛卡尔位姿
 * ========================================================= */

/*
 * 公共笛卡尔坐标系约定：
 *
 * 与当前 UR5 URDF / PyBullet 模型一致。
 *
 * Base Frame：
 *
 * URDF base_link
 *
 * End Frame：
 *
 * URDF ee_link
 *
 * 因此公共运动学接口描述：
 *
 * ee_link
 * 相对于
 * base_link
 * 的位置与姿态。
 *
 * Standard DH 坐标系仅作为
 * Kinematics 模块内部实现细节，
 * 不暴露给其他模块。
 *
 *
 * Position：
 *
 * x_mm
 * y_mm
 * z_mm
 *
 * 单位：
 *
 * mm
 *
 *
 * Orientation：
 *
 * rx_deg = Roll  about X
 * ry_deg = Pitch about Y
 * rz_deg = Yaw   about Z
 *
 * 单位：
 *
 * degree
 *
 * 姿态组合约定：
 *
 * R =
 * Rz(rz)
 * ×
 * Ry(ry)
 * ×
 * Rx(rx)
 *
 * 即 Roll-Pitch-Yaw 表示。
 */
typedef struct
{
    robot_real_t x_mm;

    robot_real_t y_mm;

    robot_real_t z_mm;


    robot_real_t rx_deg;

    robot_real_t ry_deg;

    robot_real_t rz_deg;

} robot_pose_t;


/* =========================================================
 * 齐次变换矩阵
 * ========================================================= */

/*
 * 4 × 4 Homogeneous Transform。
 *
 * 使用 Row-Major 存储：
 *
 * matrix[row][column]
 *
 * 形式：
 *
 * [ R00 R01 R02 Tx ]
 * [ R10 R11 R12 Ty ]
 * [ R20 R21 R22 Tz ]
 * [  0   0   0   1 ]
 *
 * 其中：
 *
 * R：
 * 3 × 3 Rotation Matrix
 *
 * T：
 * Translation，单位 mm
 *
 *
 * 公共含义：
 *
 * T_base_ee
 *
 * 即：
 *
 * ee_link 相对于 base_link
 * 的齐次变换。
 *
 * 更严格地说：
 *
 * p_base =
 * T_base_ee × p_ee
 *
 *
 * 注意：
 *
 * Kinematics 模块内部可以使用
 * Standard DH 的 T_0_6，
 * 但在返回本类型之前，
 * 必须转换为公共的
 *
 * base_link -> ee_link
 *
 * 坐标约定。
 */
typedef struct
{
    robot_real_t matrix[4][4];

} robot_transform_t;


#endif