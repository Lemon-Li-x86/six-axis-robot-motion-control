#include <stddef.h>
#include <stdint.h>
#include "angle_utils.h"

/* Canonical range is [-18000,18000), as robot_types.h specifies. */
robot_joint_angle_t robot_joint_angle_normalize_raw(int32_t raw_angle)
{
    int32_t raw=raw_angle % ROBOT_JOINT_FULL_TURN_RAW;
    if(raw>=ROBOT_JOINT_HALF_TURN_RAW)
        raw-=ROBOT_JOINT_FULL_TURN_RAW;
    if(raw< -ROBOT_JOINT_HALF_TURN_RAW)
        raw+=ROBOT_JOINT_FULL_TURN_RAW;
    return (robot_joint_angle_t)raw;
}

robot_status_t robot_angles_canonicalize(
    const robot_joint_positions_t *continuous,
    robot_joint_angles_t *canonical)
{
    robot_joint_angles_t result={{0}};
    uint32_t i;
    if(continuous==NULL || canonical==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    for(i=0U;i<ROBOT_JOINT_COUNT;i++)
        result.value[i]=robot_joint_angle_normalize_raw(continuous->value[i]);
    *canonical=result;
    return ROBOT_STATUS_OK;
}
