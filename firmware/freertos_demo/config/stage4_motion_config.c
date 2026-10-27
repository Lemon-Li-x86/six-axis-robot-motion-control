#include "stage4_motion_config.h"
#include "task_config.h"
#include "kinematics.h"
#include "firmware_defaults_generated.h"

/* Assemble build-generated JSON policies and derived algorithm tolerances.
 * Simulation defaults remain provisional, not measured hardware PID tuning.
 */
#define STAGE4_REFERENCE_TIMEOUT_MS \
    (FW_DEFAULT_STAGE4_REFERENCE_TIMEOUT_TRAJECTORY_PERIODS * TASK_PERIOD_TRAJECTORY_MS)
#define STAGE4_CARTESIAN_WAYPOINT_TOLERANCE_MM KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM
#define STAGE4_CARTESIAN_WAYPOINT_ROTATION_TOLERANCE KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL

static const motion_stage4_config_t g_stage4_config={
    .pid=FW_DEFAULT_STAGE4_PID,
    .max_velocity_deg_s=
        FW_DEFAULT_STAGE4_MAX_VELOCITY_DEG_S,
    .max_acceleration_deg_s2=
        FW_DEFAULT_STAGE4_MAX_ACCELERATION_DEG_S2,
    .feedback_timeout_ms=FW_DEFAULT_STAGE4_FEEDBACK_TIMEOUT_MS,
    .reference_timeout_ms=STAGE4_REFERENCE_TIMEOUT_MS,
    .cartesian_start_tolerance_mm=FW_DEFAULT_STAGE4_CARTESIAN_START_TOLERANCE_MM,
    .cartesian_start_rotation_matrix_tolerance=FW_DEFAULT_STAGE4_CARTESIAN_START_ROTATION_MATRIX_TOLERANCE,
    .cartesian_waypoint_position_tolerance_mm=STAGE4_CARTESIAN_WAYPOINT_TOLERANCE_MM,
    .cartesian_waypoint_rotation_matrix_tolerance=STAGE4_CARTESIAN_WAYPOINT_ROTATION_TOLERANCE
};

const motion_stage4_config_t *stage4_motion_default_config(void)
{
    return &g_stage4_config;
}
