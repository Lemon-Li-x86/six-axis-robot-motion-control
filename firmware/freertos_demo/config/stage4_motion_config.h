/* Single ownership of Stage 4 simulation defaults.
 * Values are provisional, NOT calibrated UR5 hardware limits/PID gains.
 * Never copy these literals into algorithm or task modules.
 */
#ifndef ROBOT_STAGE4_MOTION_CONFIG_H
#define ROBOT_STAGE4_MOTION_CONFIG_H
#include "motion_pipeline.h"
const motion_stage4_config_t *stage4_motion_default_config(void);
#endif
