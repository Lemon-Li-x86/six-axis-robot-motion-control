#include "stage4_motion_config.h"

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
    .feedback_timeout_ms=STAGE4_FEEDBACK_TIMEOUT_MS
};

const motion_stage4_config_t *stage4_motion_default_config(void)
{
    return &g_stage4_config;
}
