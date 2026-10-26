#include "stage4_motion_config.h"
#include "task_config.h"
#include "kinematics.h"

/* Simulation-only INITIAL configuration, not measured PID tuning.
 * Single definition site for all Stage 4 control strategy defaults.
 */
#define STAGE4_PID_STARTER_GAIN_P 0.8F
#define STAGE4_PID_STARTER_GAIN_I 0.0F
#define STAGE4_PID_STARTER_GAIN_D 0.0F
#define STAGE4_PID_INTEGRAL_CAP_DEG_S 10.0F
#define STAGE4_PID_COMMAND_CAP_DEG 3.0F
#define STAGE4_PID_DEADBAND_DEG 0.02F
#define STAGE4_SIM_VELOCITY_CAP_DEG_S 15.0F
#define STAGE4_SIM_ACCEL_CAP_DEG_S2 30.0F
#define STAGE4_FEEDBACK_TIMEOUT_MS 3000U
#define STAGE4_REFERENCE_TIMEOUT_MS (3U * TASK_PERIOD_TRAJECTORY_MS)
#define STAGE4_CARTESIAN_START_TOLERANCE_MM 0.5F
#define STAGE4_CARTESIAN_START_ROTATION_TOLERANCE 0.01F
#define STAGE4_CARTESIAN_WAYPOINT_TOLERANCE_MM KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM
#define STAGE4_CARTESIAN_WAYPOINT_ROTATION_TOLERANCE KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL

#define STAGE4_PID_ENTRY { \
    STAGE4_PID_STARTER_GAIN_P, STAGE4_PID_STARTER_GAIN_I, \
    STAGE4_PID_STARTER_GAIN_D, STAGE4_PID_INTEGRAL_CAP_DEG_S, \
    STAGE4_PID_COMMAND_CAP_DEG, STAGE4_PID_DEADBAND_DEG \
}
#define STAGE4_SIX_VALUES(x) {{x,x,x,x,x,x}}

static const motion_stage4_config_t g_stage4_config={
    .pid={ STAGE4_PID_ENTRY,STAGE4_PID_ENTRY,STAGE4_PID_ENTRY,
           STAGE4_PID_ENTRY,STAGE4_PID_ENTRY,STAGE4_PID_ENTRY },
    .max_velocity_deg_s=
        STAGE4_SIX_VALUES(STAGE4_SIM_VELOCITY_CAP_DEG_S),
    .max_acceleration_deg_s2=
        STAGE4_SIX_VALUES(STAGE4_SIM_ACCEL_CAP_DEG_S2),
    .feedback_timeout_ms=STAGE4_FEEDBACK_TIMEOUT_MS,
    .reference_timeout_ms=STAGE4_REFERENCE_TIMEOUT_MS,
    .cartesian_start_tolerance_mm=STAGE4_CARTESIAN_START_TOLERANCE_MM,
    .cartesian_start_rotation_matrix_tolerance=STAGE4_CARTESIAN_START_ROTATION_TOLERANCE,
    .cartesian_waypoint_position_tolerance_mm=STAGE4_CARTESIAN_WAYPOINT_TOLERANCE_MM,
    .cartesian_waypoint_rotation_matrix_tolerance=STAGE4_CARTESIAN_WAYPOINT_ROTATION_TOLERANCE
};

const motion_stage4_config_t *stage4_motion_default_config(void)
{
    return &g_stage4_config;
}
