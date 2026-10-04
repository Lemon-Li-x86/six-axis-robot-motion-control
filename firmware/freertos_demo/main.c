/*
 * 文件：main.c
 *
 * 用途：
 * 六轴机器人 Cortex-M4 / FreeRTOS 固件入口。
 *
 * main.c 只负责系统级生命周期：
 *
 * 1. Board 初始化；
 * 2. Driver 初始化；
 * 3. UART TX Manager 初始化；
 * 4. Performance Monitor 初始化；
 * 5. 启动前 Performance Benchmark；
 * 6. 启动前 Kinematics Self Test；
 * 7. Motor Driver 初始化；
 * 8. 设置初始关节目标；
 * 9. 启动 Application Tasks；
 * 10. 启动 FreeRTOS Scheduler。
 *
 * Protocol 收发和 Application Dispatch
 * 不在 main.c 中实现。
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


/* =========================================================
 * Firmware Entry
 * ========================================================= */

/**
 * @brief 固件主入口。
 *
 * 完成系统底层初始化、
 * 启动前验证和 Application Task 创建，
 * 随后启动 FreeRTOS Scheduler。
 *
 * @return
 * 正常情况下不会返回。
 */
int
main(void)
{
    const robot_joint_angles_t
        initial_joint_targets =
        {
            .value =
            {
                6000,
                0,
                0,
                0,
                0,
                0
            }
        };


    /* =====================================================
     * Board
     * ===================================================== */

    board_clock_init();


    board_gpio_init();


    /* =====================================================
     * Low-Level Drivers
     * ===================================================== */

    uart_driver_init();


timer_driver_init();


if (
    gpio_driver_init()
    !=
    ROBOT_STATUS_OK
)
{
    while (1)
    {
    }
}


/*
 * 第2阶段 Timer 周期中断验证。
 *
 * Timer1：
 *
 * 100 Hz
 * =
 * 10 ms 周期。
 *
 * 当前暂时不注册业务 Callback，
 * Driver ISR 仍会正常：
 *
 * 1. 响应 Interrupt；
 * 2. 清除 IRQ；
 * 3. 累加 IRQ Counter。
 *
 * 后续 Control / Trajectory 模块
 * 可以注册实际 Callback。
 */
if (
    timer_driver_start_periodic(
        100U,
        0
    )
    !=
    ROBOT_STATUS_OK
)
{
    while (1)
    {
    }
}

/* =====================================================
 * Runtime Configuration
 * ===================================================== */

if (
    runtime_config_init()
    !=
    ROBOT_STATUS_OK
)
{
    while (1)
    {
    }
}


    /* =====================================================
     * UART TX Serialization
     * ===================================================== */

    if (
        uart_tx_manager_init()
        !=
        ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * Performance Monitor
     * ===================================================== */

    performance_monitor_init();


    /*
     * Scheduler 启动前执行完整
     * Protocol Frame Parser Benchmark。
     *
     * 当前保留 1000 个 Sample，
     * 用于建立通信性能基线。
     */
    if (
        performance_monitor_run_parser_benchmark(
            1000U
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * Kinematics Self Test
     * ===================================================== */

    /*
     * Scheduler 启动前执行运动学自检。
     *
     * 当前 Self Test 已覆盖：
     *
     * 1. Forward Kinematics；
     * 2. Analytic IK Formula Chain；
     * 3. Public IK Solution Assembly；
     * 4. Canonical Angle；
     * 5. Duplicate Solution Detection；
     * 6. Wrist Singularity。
     *
     * 如果运动学验证失败，
     * 固件停止启动，
     * 防止错误运动学结果进入
     * 后续 Control / Trajectory 模块。
     */
    if (
        kinematics_self_test_run()
        !=
        ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * Motor Driver
     * ===================================================== */

    if (
        motor_driver_init()
        !=
        ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /*
     * 当前 Stage 1 / Stage 2
     * 默认测试目标：
     *
     * Joint 1 = +60°
     * Joint 2~6 = 0°
     */
    if (
        motor_driver_set_target_positions(
            &initial_joint_targets
        )
        !=
        ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * Application Tasks
     * ===================================================== */

    /*
     * 创建：
     *
     * 1. ProtocolTX；
     * 2. ProtocolRX。
     *
     * Protocol Task 内部负责：
     *
     * UART Event
     * ->
     * Protocol
     * ->
     * Application Dispatch。
     */
    if (
        protocol_tasks_start()
        !=
        ROBOT_STATUS_OK
    )
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * Scheduler
     * ===================================================== */

    vTaskStartScheduler();


    /*
     * 正常情况下 Scheduler 不会返回。
     *
     * 如果返回，
     * 通常意味着 FreeRTOS 无法继续启动，
     * 因此停留在此处。
     */
    while (1)
    {
    }
}