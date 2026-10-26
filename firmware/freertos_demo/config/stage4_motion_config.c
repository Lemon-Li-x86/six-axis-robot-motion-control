#include "stage4_motion_config.h"

/* Simulation-only starting points; must be tuned against PyBullet feedback.
 * Separate from UR5 fixed geometry/position limits in Robot Model.
 * Requested max motion rates are conservative but NOT a safety certificate.
 */
static const motion_stage4_config_t g_stage4_config = {
    .pid = {
        { .kp=0.8F, .ki=0.0F, .kd=0.0F, .integral_limit_deg_s=10.0F,
          .output_limit_deg=3.0F, .deadband_deg=0.02F },
        { .kp=0.8F, .ki=0.0F, .kd=0.0F, .integral_limit_deg_s=10.0F,
          .output_limit_deg=3.0F, .deadband_deg=0.02F },
        { .kp=0.8F, .ki=0.0F, .kd=0.0F, .integral_limit_deg_s=10.0F,
          .output_limit_deg=3.0F, .deadband_deg=0.02F },
        { .kp=0.8F, .ki=0.0F, .kd=0.0F, .integral_limit_deg_s=10.0F,
          .output_limit_deg=3.0F, .deadband_deg=0.02F },
        { .kp=0.8F, .ki=0.0F, .kd=0.0F, .integral_limit_deg_s=10.0F,
          .output_limit_deg=3.0F, .deadband_deg=0.02F },
        { .kp=0.8F, .ki=0.0F, .kd=0.0F, .integral_limit_deg_s=10.0F,
          .output_limit_deg=3.0F, .deadband_deg=0.02F }
    },
    .max_velocity_deg_s = {{15.0F,15.0F,15.0F,15.0F,15.0F,15.0F}},
    .max_acceleration_deg_s2 = {{30.0F,30.0F,30.0F,30.0F,30.0F,30.0F}},
    .feedback_timeout_ms = 3000U
};

const motion_stage4_config_t *stage4_motion_default_config(void)
{
    return &g_stage4_config;
}
