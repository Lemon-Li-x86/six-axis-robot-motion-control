/**
 * @file ur5_model.c
 * @brief UR5 geometry and continuous joint limits used by this simulation.
 *
 * DH lengths are millimetres; alpha angles are radians. Joint limits use
 * centidegrees and correspond to the bundled URDF, not a hardware datasheet.
 */



#include "robot_model.h"
#include "math_constants.h"









static const robot_model_t ur5_model =
{
    
    .dh_a_mm =
    {
        0.0F,
        -425.0F,
        -392.25F,
        0.0F,
        0.0F,
        0.0F
    },


    
    .dh_d_mm =
    {
        89.159F,
        0.0F,
        0.0F,
        109.15F,
        94.65F,
        82.3F
    },


    
    .dh_alpha_rad =
    {
        ROBOT_PI_F / 2.0F,
        0.0F,
        0.0F,
        ROBOT_PI_F / 2.0F,
        -ROBOT_PI_F / 2.0F,
        0.0F
    },
    /* Public base and DH base differ by a 180-degree Z rotation.
     * The explicit inverse keeps model replacement independent of symmetry.
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
    /* Map the DH tool frame to the public URDF ee_link frame. */


    
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
    },
    /* Continuous J3 range is +/-180 degrees; other joints +/-360 degrees. */


    

    
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




const robot_model_t *robot_model_get_active(void)
{
    return &ur5_model;
}
