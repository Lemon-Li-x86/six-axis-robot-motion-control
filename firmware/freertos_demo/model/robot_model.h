/**
 * @file robot_model.h
 * @brief Immutable robot geometry, transforms and joint position limits.
 *
 * The model describes static robot properties, not measured joint states,
 * controller state or trajectory runtime state.
 */



#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "robot_types.h"


/** @brief Static robot model with standard DH parameters and frame mappings. */
typedef struct
{
    /* Standard DH dimensions in mm; alpha angles are in radians. */
    
    robot_real_t dh_a_mm[
        ROBOT_JOINT_COUNT
    ];

    robot_real_t dh_d_mm[
        ROBOT_JOINT_COUNT
    ];

    robot_real_t dh_alpha_rad[
        ROBOT_JOINT_COUNT
    ];
    /* Public base frame <-> DH frame; inverse stored independently. */


    
    robot_transform_t
        base_frame_conversion;

    robot_transform_t
        base_frame_conversion_inverse;
    /* DH tool frame <-> public end-effector frame. */


    robot_transform_t
        tool_frame_conversion;

    robot_transform_t
        tool_frame_conversion_inverse;
    /* Inclusive continuous joint limits in 0.01 degrees (not canonical angles). */


    

    
    robot_joint_positions_t
        joint_min_positions;

    robot_joint_positions_t
        joint_max_positions;

} robot_model_t;



/**
 * @brief Return the active immutable model (currently UR5).
 * @return Pointer to the shared model; never modify its data.
 */
const robot_model_t *
robot_model_get_active(void);


#endif
