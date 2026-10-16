
/*
 * 文件：ur5_model.c
 *
 * 用途：
 * 定义当前项目使用的 UR5
 * 静态机器人几何与关节限位模型。
 */

#include "robot_model.h"


/* =========================================================
 * Model Constants
 * ========================================================= */

#define UR5_PI_F \
    3.14159265358979323846F


/* =========================================================
 * UR5 Model
 * ========================================================= */

static const robot_model_t ur5_model =
{
    /*
     * Standard DH Parameter a。
     *
     * 单位：
     * mm。
     */
    .dh_a_mm =
    {
        0.0F,
        -425.0F,
        -392.25F,
        0.0F,
        0.0F,
        0.0F
    },


    /*
     * Standard DH Parameter d。
     *
     * 单位：
     * mm。
     */
    .dh_d_mm =
    {
        89.159F,
        0.0F,
        0.0F,
        109.15F,
        94.65F,
        82.3F
    },


    /*
     * Standard DH Parameter alpha。
     *
     * 单位：
     * radian。
     */
    .dh_alpha_rad =
    {
        UR5_PI_F / 2.0F,
        0.0F,
        0.0F,
        UR5_PI_F / 2.0F,
        -UR5_PI_F / 2.0F,
        0.0F
    },


    /*
     * Public Base Frame
     * ->
     * Standard DH Base Frame。
     *
     * 当前矩阵自身等于其逆矩阵，
     * 但仍显式保存 inverse。
     *
     * 这样 Robot Model API
     * 不依赖当前矩阵具有自逆这一偶然性质，
     * 后续替换机器人模型时无需修改算法层。
     */
    .base_frame_conversion =
    {
        .matrix =
        {
            {
                -1.0F,
                0.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                -1.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                1.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    },


    /*
     * Standard DH Base Frame
     * ->
     * Public Base Frame。
     */
    .base_frame_conversion_inverse =
    {
        .matrix =
        {
            {
                -1.0F,
                0.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                -1.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                1.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    },


    /*
     * Standard DH Tool Frame
     * ->
     * Public ee_link。
     */
    .tool_frame_conversion =
    {
        .matrix =
        {
            {
                0.0F,
                -1.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                -1.0F,
                0.0F
            },

            {
                1.0F,
                0.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    },


    /*
     * Public ee_link
     * ->
     * Standard DH Tool Frame。
     */
    .tool_frame_conversion_inverse =
    {
        .matrix =
        {
            {
                0.0F,
                0.0F,
                1.0F,
                0.0F
            },

            {
                -1.0F,
                0.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                -1.0F,
                0.0F,
                0.0F
            },

            {
                0.0F,
                0.0F,
                0.0F,
                1.0F
            }
        }
    },


    /* =========================================================
     * Joint Position Limits
     * ========================================================= */

    /*
     * 当前项目所使用 UR5 URDF 的关节限位。
     *
     * 来源：
     * simulation/pybullet_ur5/models/ur5/urdf/ur5.urdf
     *
     * 关节顺序：
     * J1 Shoulder Pan；
     * J2 Shoulder Lift；
     * J3 Elbow；
     * J4 Wrist 1；
     * J5 Wrist 2；
     * J6 Wrist 3。
     *
     * 范围：
     * J3：[-180°, +180°]；
     * 其他关节：[-360°, +360°]。
     *
     * 单位：
     * 0.01 degree。
     *
     * 注意：
     * 这些数值来自当前仿真模型，
     * 不应直接视为其他 UR5 实物的安全限位。
     * 本模块只存储限位，不执行目标关节检查。
     */
    .joint_min_positions =
    {
        .value =
        {
            -36000,
            -36000,
            -18000,
            -36000,
            -36000,
            -36000
        }
    },

    .joint_max_positions =
    {
        .value =
        {
            36000,
            36000,
            18000,
            36000,
            36000,
            36000
        }
    }
};


/* =========================================================
 * Public API
 * ========================================================= */

const robot_model_t *robot_model_get_active(void)
{
    return &ur5_model;
}
