cmake_minimum_required(VERSION 3.20)

# 六轴工业机器人运动控制项目
project(
    six_axis_robot_motion_control
    LANGUAGES C ASM
)

# 当前阶段先构建 FreeRTOS Cortex-M4 固件
add_subdirectory(
    firmware/freertos_demo
)