/**
 * @file pid.h
 * @brief PID API。
 */
#ifndef ROBOT_PID_H
#define ROBOT_PID_H
#include "robot_types.h"
#include "error_code.h"
#include <stdint.h>

typedef struct {
    robot_real_t kp;
    robot_real_t ki;
    robot_real_t kd;
    robot_real_t integral_limit_deg_s;
    robot_real_t output_limit_deg;
    robot_real_t deadband_deg;
} pid_config_t;

typedef struct {
    pid_config_t config;
    double integral_deg_s;
    double prev_error_deg;
    double prev_prev_error_deg;
    double output_deg;
    uint8_t initialized;
} pid_controller_t;

/**
 * @brief 初始化增量式 PID 状态。
 * @param[out] state 控制器状态。
 * @param[in] config PID 增益、积分限幅及死区配置。
 * @return ROBOT_STATUS_OK 或参数状态码。
 */
robot_status_t pid_init(pid_controller_t *state,const pid_config_t *config);
/** @brief 重置控制器积分、误差历史和输出状态。 */
void pid_reset(pid_controller_t *state);
/**
 * @brief 根据采样时间计算一轮关节角度增量修正。
 * @param[in,out] state 控制器状态。
 * @param[in] setpoint_deg 目标角度，°。
 * @param[in] measured_deg 测量角度，°。
 * @param[in] dt_s 采样间隔，s。
 * @param[out] correction_deg 输出角度增量，°。
 * @return ROBOT_STATUS_OK 或相应状态码。
 */
robot_status_t pid_step(pid_controller_t *state,double setpoint_deg,
                         double measured_deg,double dt_s,
                         robot_real_t *correction_deg);
#endif
