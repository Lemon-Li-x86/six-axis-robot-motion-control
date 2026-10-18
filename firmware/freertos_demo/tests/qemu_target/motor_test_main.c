
/*
 * 文件：motor_test_main.c
 *
 * 用途：
 * 在 QEMU Cortex-M4 上执行 Motor Driver 边界测试。
 *
 * 本测试使用生产版 motor_driver.c，
 * 不使用 Host Mock 或 Unity。
 *
 * 注意：
 * 当前 Motor Driver 是关节状态管理抽象层，
 * 本测试不代表真实伺服电机硬件测试。
 */

#include <stddef.h>
#include <stdint.h>

#include "motor_driver.h"
#include "robot_types.h"
#include "error_code.h"

#define MOTOR_TEST_COUNT 11U

#define CHECK(condition)       \
    do                         \
    {                          \
        if (!(condition))      \
        {                      \
            return 0U;         \
        }                      \
    } while (0)

typedef struct
{
    uint32_t total;
    uint32_t passed;
    uint32_t failed;
    uint32_t results[MOTOR_TEST_COUNT];
} motor_test_report_t;

/* GDB 读取此变量。 */
volatile motor_test_report_t g_motor_test_report;


/* =========================================================
 * Float Comparison
 * ========================================================= */

static uint32_t float_close(
    float actual,
    float expected
)
{
    float difference = actual - expected;

    return (
        difference >= -0.01F &&
        difference <= 0.01F
    ) ? 1U : 0U;
}


/* =========================================================
 * Test 0: Before Initialization
 * ========================================================= */

static uint32_t test_before_init(void)
{
    robot_joint_angles_t joints = {0};

    CHECK(motor_driver_has_feedback() == 0U);

    CHECK(
        motor_driver_get_target_positions(&joints)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        motor_driver_update_feedback(&joints, 0.1F)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    return 1U;
}


/* =========================================================
 * Test 1: Initialization
 * ========================================================= */

static uint32_t test_initial_state(void)
{
    robot_joint_angles_t targets;
    uint32_t i;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);
    CHECK(motor_driver_has_feedback() == 0U);

    CHECK(
        motor_driver_get_target_positions(&targets)
        == ROBOT_STATUS_OK
    );

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        CHECK(targets.value[i] == 0);
    }

    return 1U;
}


/* =========================================================
 * Test 2: Canonical Angle
 * ========================================================= */

static uint32_t test_target_normalization(void)
{
    robot_joint_angles_t input =
    {
        .value = {
            18000, 19000, -18000,
            -19000, 17999, -17999
        }
    };

    robot_joint_angles_t output;

    const int16_t expected[6] =
    {
        -18000, -17000, -18000,
        17000, 17999, -17999
    };

    uint32_t i;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_set_target_positions(&input)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_target_positions(&output)
        == ROBOT_STATUS_OK
    );

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        CHECK(output.value[i] == expected[i]);
    }

    return 1U;
}


/* =========================================================
 * Test 3: int16_t Input Boundaries
 * ========================================================= */

static uint32_t test_integer_boundaries(void)
{
    robot_joint_angles_t input =
    {
        .value = {
            32767, -32768, 18000,
            -18000, 0, 1
        }
    };

    robot_joint_angles_t output;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_set_target_positions(&input)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_target_positions(&output)
        == ROBOT_STATUS_OK
    );

    CHECK(output.value[0] == -3233);
    CHECK(output.value[1] == 3232);
    CHECK(output.value[2] == -18000);
    CHECK(output.value[3] == -18000);
    CHECK(output.value[4] == 0);
    CHECK(output.value[5] == 1);

    return 1U;
}


/* =========================================================
 * Test 4: NULL Pointer Contract
 * ========================================================= */

static uint32_t test_null_pointers(void)
{
    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_set_target_positions(NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        motor_driver_get_target_positions(NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        motor_driver_update_feedback(NULL, 0.1F)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        motor_driver_get_positions(NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        motor_driver_get_unwrapped_positions(NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        motor_driver_get_velocities(NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    return 1U;
}


/* =========================================================
 * Test 5: No Feedback Yet
 * ========================================================= */

static uint32_t test_feedback_not_ready(void)
{
    robot_joint_angles_t canonical;
    robot_joint_positions_t continuous;
    robot_joint_velocities_t velocities;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_get_positions(&canonical)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_ERROR_NOT_READY
    );

    return 1U;
}


/* =========================================================
 * Test 6: First Feedback
 * ========================================================= */

static uint32_t test_first_feedback(void)
{
    robot_joint_angles_t input =
    {
        .value = {1000, 2000, 3000, 4000, 5000, 6000}
    };

    robot_joint_angles_t canonical;
    robot_joint_positions_t continuous;
    robot_joint_velocities_t velocities;

    uint32_t i;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_update_feedback(&input, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(motor_driver_has_feedback() == 1U);

    CHECK(
        motor_driver_get_positions(&canonical)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_OK
    );

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        CHECK(canonical.value[i] == input.value[i]);
        CHECK(continuous.value[i] == input.value[i]);
        CHECK(float_close(velocities.value[i], 0.0F));
    }

    return 1U;
}


/* =========================================================
 * Test 7: Positive Wrap
 * ========================================================= */

static uint32_t test_forward_wrap(void)
{
    robot_joint_angles_t first =
    {
        .value = {17900, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t second =
    {
        .value = {-17900, 0, 0, 0, 0, 0}
    };

    robot_joint_positions_t continuous;
    robot_joint_velocities_t velocities;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_update_feedback(&first, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_update_feedback(&second, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_OK
    );

    /* 179° -> -179° 相当于正向移动 2°。 */
    CHECK(continuous.value[0] == 18100);
    CHECK(float_close(velocities.value[0], 20.0F));

    return 1U;
}


/* =========================================================
 * Test 8: Negative Wrap
 * ========================================================= */

static uint32_t test_backward_wrap(void)
{
    robot_joint_angles_t first =
    {
        .value = {-17900, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t second =
    {
        .value = {17900, 0, 0, 0, 0, 0}
    };

    robot_joint_positions_t continuous;
    robot_joint_velocities_t velocities;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_update_feedback(&first, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_update_feedback(&second, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_OK
    );

    /* -179° -> 179° 相当于反向移动 2°。 */
    CHECK(continuous.value[0] == -18100);
    CHECK(float_close(velocities.value[0], -20.0F));

    return 1U;
}


/* =========================================================
 * Test 9: Invalid Time Interval
 * ========================================================= */

static uint32_t test_nonpositive_delta_time(void)
{
    robot_joint_angles_t first =
    {
        .value = {1000, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t second =
    {
        .value = {1100, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t third =
    {
        .value = {1300, 0, 0, 0, 0, 0}
    };

    robot_joint_positions_t continuous;
    robot_joint_velocities_t velocities;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_update_feedback(&first, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_update_feedback(&second, 0.0F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_OK
    );

    CHECK(continuous.value[0] == 1100);

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_OK
    );

    CHECK(float_close(velocities.value[0], 0.0F));

    CHECK(
        motor_driver_update_feedback(&third, -0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_OK
    );

    CHECK(continuous.value[0] == 1300);
    CHECK(float_close(velocities.value[0], 0.0F));

    return 1U;
}


/* =========================================================
 * Test 10: Multiple Continuous Updates
 * ========================================================= */

static uint32_t test_multiple_wrap_updates(void)
{
    robot_joint_angles_t first =
    {
        .value = {17000, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t second =
    {
        .value = {-17000, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t third =
    {
        .value = {-15000, 0, 0, 0, 0, 0}
    };

    robot_joint_angles_t canonical;
    robot_joint_positions_t continuous;
    robot_joint_velocities_t velocities;

    CHECK(motor_driver_init() == ROBOT_STATUS_OK);

    CHECK(
        motor_driver_update_feedback(&first, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_update_feedback(&second, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_update_feedback(&third, 0.1F)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_positions(&canonical)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_unwrapped_positions(&continuous)
        == ROBOT_STATUS_OK
    );

    CHECK(
        motor_driver_get_velocities(&velocities)
        == ROBOT_STATUS_OK
    );

    /*
     * Canonical：
     * 170° -> -170° -> -150°
     *
     * Continuous：
     * 170° -> 190° -> 210°
     *
     * 最后一步：
     * 20° / 0.1 s = 200°/s。
     */
    CHECK(canonical.value[0] == -15000);
    CHECK(continuous.value[0] == 21000);
    CHECK(float_close(velocities.value[0], 200.0F));

    return 1U;
}


/* =========================================================
 * Test Runner
 * ========================================================= */

static void record_result(
    uint32_t index,
    uint32_t result
)
{
    g_motor_test_report.results[index] = result;

    if (result == 1U)
    {
        g_motor_test_report.passed++;
    }
    else
    {
        g_motor_test_report.failed++;
    }
}


/*
 * 在此函数上设置 GDB 断点，
 * 即可查看全部目标端测试结果。
 *
 * Windows Host Test 不编译此函数，
 * 避免 MSVC 无法识别 GCC 汇编语法。
 */
#ifndef MOTOR_DRIVER_HOST_TEST

__attribute__((noinline))
void qemu_motor_test_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}

#endif


int main(void)
{
    g_motor_test_report.total = MOTOR_TEST_COUNT;
    g_motor_test_report.passed = 0U;
    g_motor_test_report.failed = 0U;

    record_result(0U, test_before_init());
    record_result(1U, test_initial_state());
    record_result(2U, test_target_normalization());
    record_result(3U, test_integer_boundaries());
    record_result(4U, test_null_pointers());
    record_result(5U, test_feedback_not_ready());
    record_result(6U, test_first_feedback());
    record_result(7U, test_forward_wrap());
    record_result(8U, test_backward_wrap());
    record_result(9U, test_nonpositive_delta_time());
    record_result(10U, test_multiple_wrap_updates());

#ifdef MOTOR_DRIVER_HOST_TEST
    return (g_motor_test_report.failed == 0U) ? 0 : 1;
#else
    qemu_motor_test_done();
    return 0;
#endif
}
