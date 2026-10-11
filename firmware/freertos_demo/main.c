
/*
 * 文件：main.c
 *
 * 用途：
 * 六轴机器人 Cortex-M4 / FreeRTOS 固件入口。
 *
 * main.c 负责系统生命周期：
 *
 * 1. Board 初始化；
 * 2. Driver 初始化；
 * 3. Runtime Configuration 初始化；
 * 4. UART TX Manager 初始化；
 * 5. Performance Monitor 初始化；
 * 6. 启动前 Performance Benchmark；
 * 7. 启动前 Kinematics Self Test；
 * 8. Motor Driver 初始化；
 * 9. 设置初始关节目标；
 * 10. 创建 Application Tasks；
 * 11. 启动 FreeRTOS Scheduler。
 *
 * 默认配置来源：
 *
 * config/robot_defaults.json
 *       |
 *       v
 * firmware_defaults_generated.h
 *
 * 注意：
 * main.c 不负责解析 JSON。
 * 所有配置参数在编译阶段已经转换为 C 宏。
 */

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"

#include "uart_driver.h"
#include "motor_driver.h"
#include "timer_driver.h"
#include "gpio_driver.h"

#include "robot_types.h"
#include "error_code.h"

#include "runtime_config.h"
#include "uart_tx_manager.h"
#include "protocol_tasks.h"

#include "performance_monitor.h"

#include "kinematics_self_test.h"

/*
 * 自动生成的固件默认配置。
 *
 * 包含：
 * 1. Timer 频率；
 * 2. Benchmark 样本数量；
 * 3. 初始六轴关节目标。
 */
#include "firmware_defaults_generated.h"


/* =========================================================
 * Firmware Entry
 * ========================================================= */

/**
 * @brief Cortex-M4 固件入口。
 *
 * @return 正常运行时不会返回。
 */
int main(void)
{
    /*
     * 初始关节目标角度。
     *
     * 数据来源：
     * robot_defaults.json
     *
     * 单位：
     * 0.01 degree。
     *
     * 例如：
     * 6000 = +60.00°。
     *
     * 当前仍使用 Canonical Joint Angle：
     * [-180°, 180°)。
     */
    const robot_joint_angles_t
        initial_joint_targets =
        {
            .value =
            {
                FW_DEFAULT_JOINT_1_TARGET_CENTIDEG,
                FW_DEFAULT_JOINT_2_TARGET_CENTIDEG,
                FW_DEFAULT_JOINT_3_TARGET_CENTIDEG,
                FW_DEFAULT_JOINT_4_TARGET_CENTIDEG,
                FW_DEFAULT_JOINT_5_TARGET_CENTIDEG,
                FW_DEFAULT_JOINT_6_TARGET_CENTIDEG
            }
        };


    /* =====================================================
     * 1. Board Initialization
     * ===================================================== */

    /*
     * 初始化系统时钟与板级 GPIO。
     *
     * 当前 QEMU MPS2-AN386 的系统时钟
     * 由模拟平台提供。
     */
    board_clock_init();

    board_gpio_init();


    /* =====================================================
     * 2. Driver Initialization
     * ===================================================== */

    /*
     * 初始化 UART。
     *
     * 后续 ProtocolRX Task 会开启 RX IRQ。
     */
    uart_driver_init();

    /*
     * 初始化 Timer Driver。
     */
    timer_driver_init();

    /*
     * 初始化 GPIO Driver。
     */
    if (
        gpio_driver_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 3. Timer Periodic Interrupt
     * ===================================================== */

    /*
     * Timer 周期中断验证。
     *
     * 频率来自 JSON：
     *
     * timer_frequency_hz
     *
     * 当前默认：
     * 100 Hz = 10 ms。
     *
     * Driver ISR 负责：
     *
     * 1. 响应 Timer IRQ；
     * 2. 清除中断状态；
     * 3. 累加 IRQ Counter。
     *
     * 当前没有注册业务 Callback。
     *
     * 后续 Control / Trajectory 模块
     * 可以复用定时器驱动接口。
     */
    if (
        timer_driver_start_periodic(
            FW_DEFAULT_TIMER_FREQUENCY_HZ,
            0
        )
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 4. Runtime Configuration
     * ===================================================== */

    /*
     * 初始化运行时参数。
     *
     * 当前包括 ProtocolTX Period。
     *
     * 默认值来自：
     *
     * JSON
     * -> Generated Header
     * -> task_config.h
     * -> runtime_config.c
     */
    if (
        runtime_config_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 5. UART TX Manager
     * ===================================================== */

    /*
     * 初始化 UART 发送管理器。
     *
     * 保证多个业务任务发送完整协议帧时，
     * 不会发生帧字节交错。
     */
    if (
        uart_tx_manager_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 6. Performance Monitor
     * ===================================================== */

    performance_monitor_init();


    /*
     * 启动前执行 Protocol Parser Benchmark。
     *
     * 样本数量来自 JSON：
     *
     * parser_benchmark_samples
     *
     * 当前默认值：
     * 1000 samples。
     *
     * 后续应将该测试改为
     * Development Build 可选功能，
     * 避免正式固件每次启动强制执行。
     */
    if (
        performance_monitor_run_parser_benchmark(
            FW_DEFAULT_PARSER_BENCHMARK_SAMPLES
        )
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 7. Kinematics Self Test
     * ===================================================== */

    /*
     * Scheduler 启动前执行运动学自检。
     *
     * 当前测试覆盖：
     *
     * 1. Forward Kinematics；
     * 2. Analytic IK Formula Chain；
     * 3. Public IK Solution Assembly；
     * 4. Canonical Angle；
     * 5. Duplicate Solution Detection；
     * 6. Wrist Singularity。
     *
     * 如果出现错误，固件停止启动。
     *
     * 注意：
     * 当前测试并不代表已完成
     * Python Robotics Toolbox 独立精度对标。
     */
    if (
        kinematics_self_test_run()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 8. Motor Driver Initialization
     * ===================================================== */

    if (
        motor_driver_init()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 9. Initial Joint Targets
     * ===================================================== */

    /*
     * 向 Motor Driver 设置初始目标。
     *
     * 目标角度已从 JSON 编译进入固件，
     * 不再在 main.c 中重复写死。
     *
     * 当前仅设置目标状态，
     * 后续由 ProtocolTX 将目标发送至
     * Python / PyBullet 仿真端。
     */
    if (
        motor_driver_set_target_positions(
            &initial_joint_targets
        )
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 10. Application Tasks
     * ===================================================== */

    /*
     * 当前创建：
     *
     * 1. ProtocolTX；
     * 2. ProtocolRX。
     *
     * ProtocolRX 使用事件驱动：
     *
     * UART RX ISR
     * ->
     * Task Notification
     * ->
     * Protocol Parser
     * ->
     * Application Dispatch。
     */
    if (
        protocol_tasks_start()
        != ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 11. FreeRTOS Scheduler
     * ===================================================== */

    vTaskStartScheduler();


    /*
     * 正常情况下 Scheduler 不会返回。
     *
     * 如果返回，可能意味着调度器无法启动。
     * 停留在此处，避免程序错误继续运行。
     */
    while (1)
    {
    }
}
