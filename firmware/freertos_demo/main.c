
/*
 * 文件：main.c
 *
 * 用途：
 * Cortex-M4 / FreeRTOS 六轴机器人固件入口。
 *
 * 职责：
 * 1. 初始化板级环境和驱动；
 * 2. 初始化运行时配置与通信服务；
 * 3. 初始化性能监测；
 * 4. 按构建选项执行启动诊断；
 * 5. 初始化 Motor Driver 和关节目标；
 * 6. 创建任务并启动 FreeRTOS。
 *
 * 固件默认参数由 CMake 在构建时根据
 * config/robot_defaults.json 生成。
 *
 * 此文件不解析 JSON，也不实现具体运动算法。
 */

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"

#include "uart_driver.h"
#include "motor_driver.h"
#include "motor_state.h"
#include "timer_driver.h"
#include "gpio_driver.h"

#include "robot_types.h"
#include "error_code.h"

#include "runtime_config.h"
#include "uart_tx_manager.h"
#include "protocol_tasks.h"

#include "performance_monitor.h"

/*
 * 仅在启用 Kinematics Self-Test 时引入测试接口。
 * 对应选项由 CMake 定义，取值为 0 或 1。
 */
#if ROBOT_ENABLE_KINEMATICS_SELF_TEST
#include "kinematics_self_test.h"
#endif

/*
 * 自动生成的固件默认参数。
 *
 * 包含：
 * - Timer frequency
 * - Parser benchmark sample count
 * - Initial joint targets
 */
#include "firmware_defaults_generated.h"


/* =========================================================
 * Firmware Entry
 * ========================================================= */

int main(void)
{
    /*
     * 初始六轴关节目标。
     *
     * 单位：0.01 degree。
     *
     * 数据来源：
     * config/robot_defaults.json
     *
     * 当前使用 Canonical Joint Angle，
     * 范围为 [-180°, 180°)。
     *
     * 不在本文件硬编码具体目标角度。
     */
    const robot_joint_angles_t initial_joint_targets =
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

    board_clock_init();

    board_gpio_init();


    /* =====================================================
     * 2. Driver Initialization
     * ===================================================== */

    /*
     * 初始化 UART。
     * RX Interrupt 将在 ProtocolRX Task
     * 实际运行后开启。
     */
    uart_driver_init();

    /*
     * 初始化 Timer Driver。
     */
    timer_driver_init();

    /*
     * 初始化 GPIO Driver。
     */
    if (gpio_driver_init() != ROBOT_STATUS_OK)
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 3. Timer Periodic Interrupt
     * ===================================================== */

    /*
     * Timer 频率由生成配置指定。
     *
     * 当前仅执行定时中断及计数，
     * 尚未绑定 Control / Trajectory 业务回调。
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
     * 初始化可动态调整的运行参数。
     *
     * 当前主要包括：
     * ProtocolTX Period。
     */
    if (runtime_config_init() != ROBOT_STATUS_OK)
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 5. UART TX Manager
     * ===================================================== */

    /*
     * 保证多个业务 Task 发送协议帧时，
     * 帧内字节不会相互交错。
     */
    if (uart_tx_manager_init() != ROBOT_STATUS_OK)
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 6. Performance Monitor
     * ===================================================== */

    /*
     * 性能监测模块始终初始化。
     *
     * 即使关闭启动 Parser Benchmark，
     * 运行期的 UART ISR -> Task
     * 唤醒延迟统计仍可使用。
     */
    performance_monitor_init();


    /* =====================================================
     * 7. Optional Parser Benchmark
     * ===================================================== */

#if ROBOT_ENABLE_PARSER_BENCHMARK

    /*
     * 在 Scheduler 启动前执行协议解析性能测试。
     *
     * 测试次数来自生成配置。
     *
     * Benchmark 失败时停止启动，
     * 保持现有开发验证行为。
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

#endif


    /* =====================================================
     * 8. Optional Kinematics Self-Test
     * ===================================================== */

#if ROBOT_ENABLE_KINEMATICS_SELF_TEST

    /*
     * 执行完整运动学自测。
     *
     * 包括：
     * - Forward Kinematics
     * - Analytic IK Solver
     * - Solution Deduplication
     * - Public IK Integration
     * - Singularity / Boundary Tests
     *
     * 失败时停止启动。
     *
     * 当 CMake 关闭此选项时，
     * 测试代码不会链接进入正式固件。
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

#endif


    /* =====================================================
     * 9. Motor Driver Initialization
     * ===================================================== */

    if (motor_driver_init() != ROBOT_STATUS_OK)
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 10. Initial Joint Targets
     * ===================================================== */

    /*
     * 设置由 JSON 生成的初始关节目标。
     *
     * 当前仅更新 Motor Driver 中的目标状态。
     *
     * 后续 ProtocolTX Task 将目标角度
     * 发送给 Python / PyBullet 仿真端。
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


    /*
     * Initialize the shared Motor Driver access mutex before tasks start.
     */
    if (motor_state_init() != ROBOT_STATUS_OK)
    {
        while (1)
        {
        }
    }

    /* =====================================================
     * 11. Application Tasks
     * ===================================================== */

    /*
     * 当前实际创建：
     * - ProtocolTX Task
     * - ProtocolRX Task
     *
     * ProtocolRX 使用事件驱动：
     *
     * UART RX ISR
     *     -> Task Notification
     *     -> Protocol Parser
     *     -> Application Dispatch
     */
    if (protocol_tasks_start() != ROBOT_STATUS_OK)
    {
        while (1)
        {
        }
    }


    /* =====================================================
     * 12. FreeRTOS Scheduler
     * ===================================================== */

    vTaskStartScheduler();


    /*
     * 正常情况下 Scheduler 不应返回。
     *
     * 如果返回，停止执行，避免固件
     * 进入不可预期状态。
     */
    while (1)
    {
    }
}
