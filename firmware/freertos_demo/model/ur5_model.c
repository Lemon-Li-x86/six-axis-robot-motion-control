/*
 * 文件：ur5_model.c
 *
 * 用途：
 * 定义当前项目使用的 UR5 静态机器人模型。
 */

#include "robot_model.h"


#define UR5_PI_F \
    3.14159265358979323846F


static const robot_model_t
    ur5_model =
{
    /*
     * Standard DH a
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
     * Standard DH d
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
     * Standard DH alpha
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
     * <->
     * Standard DH Base Frame
     *
     * 当前该矩阵自身就是逆矩阵，
     * 但仍显式保存 inverse，
     * 避免未来机器人模型依赖这种偶然性质。
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
     * Public ee_link
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
    }
};


const robot_model_t *
robot_model_get_active(void)
{
    return
        &ur5_model;
}