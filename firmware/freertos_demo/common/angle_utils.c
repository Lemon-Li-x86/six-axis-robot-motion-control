#include <stddef.h>
#include <stdint.h>
#include "angle_utils.h"
robot_status_t robot_angles_canonicalize(
    const robot_joint_positions_t *continuous,
    robot_joint_angles_t *canonical)
{
    robot_joint_angles_t result={{0}};
    uint32_t i;
    if(continuous==NULL || canonical==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    for(i=0U;i<ROBOT_JOINT_COUNT;i++) {
        int32_t raw=continuous->value[i] % ROBOT_JOINT_FULL_TURN_RAW;
        if(raw>=ROBOT_JOINT_HALF_TURN_RAW)
            raw-=ROBOT_JOINT_FULL_TURN_RAW;
        if(raw< -ROBOT_JOINT_HALF_TURN_RAW)
            raw+=ROBOT_JOINT_FULL_TURN_RAW;
        result.value[i]=(robot_joint_angle_t)raw;
    }
    *canonical=result;
    return ROBOT_STATUS_OK;
}
