#include <stddef.h>
#include "joint_controller.h"
robot_status_t joint_controller_init(joint_controller_t *controller,
                                      const pid_config_t config[ROBOT_JOINT_COUNT])
{
    joint_controller_t next={0};
    uint32_t j;
    robot_status_t status;
    if(controller==NULL || config==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
        status=pid_init(&next.joints[j],&config[j]);
        if(status!=ROBOT_STATUS_OK) return status;
    }
    *controller=next;
    return ROBOT_STATUS_OK;
}
void joint_controller_reset(joint_controller_t *controller)
{
    uint32_t j;
    if(controller==NULL) return;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) pid_reset(&controller->joints[j]);
}
robot_status_t joint_controller_step(joint_controller_t *controller,
                                      const robot_joint_positions_t *setpoint,
                                      const robot_joint_positions_t *feedback,
                                      double dt_s,
                                      joint_control_corrections_t *out)
{
    joint_controller_t next;
    joint_control_corrections_t result={0};
    robot_status_t status;
    uint32_t j;
    if(controller==NULL || setpoint==NULL || feedback==NULL || out==NULL)
        return ROBOT_STATUS_ERROR_NULL_POINTER;
    next=*controller;
    for(j=0U;j<ROBOT_JOINT_COUNT;j++) {
        status=pid_step(&next.joints[j],
            (double)setpoint->value[j]*(double)ROBOT_JOINT_ANGLE_UNIT_DEG,
            (double)feedback->value[j]*(double)ROBOT_JOINT_ANGLE_UNIT_DEG,
            dt_s,&result.value[j]);
        if(status!=ROBOT_STATUS_OK) return status;
    }
    *controller=next;
    *out=result;
    return ROBOT_STATUS_OK;
}
