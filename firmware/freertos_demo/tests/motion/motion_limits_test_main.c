/*
 * 文件：motion_limits_test_main.c
 *
 * 用途：
 * 在 Cortex-M4 / QEMU 上验证 Motion Limits：
 *
 * 1. 连续关节位置的边界检查；
 * 2. Canonical Angle 的合法圈数展开；
 * 3. 接近 +/-180° 的跨边界情况；
 * 4. 第三关节的特殊限位；
 * 5. 输入异常和失败时输出不变。
 *
 * 不使用标准输出函数。
 * 测试结果通过 GDB 从全局变量读取。
 */

#include <stddef.h>
#include <stdint.h>

#include "motion_limits.h"


/* =========================================================
 * Test Configuration
 * ========================================================= */

#define MOTION_LIMITS_TEST_CAPACITY 40U


typedef struct
{
    uint32_t total;
    uint32_t passed;
    uint32_t failed;
    uint32_t results[MOTION_LIMITS_TEST_CAPACITY];

} motion_limits_test_report_t;


/* =========================================================
 * GDB Test Report
 * ========================================================= */

volatile motion_limits_test_report_t g_motion_limits_test_report;
volatile uint32_t g_motion_limits_finished = 0U;


/* =========================================================
 * Test Helpers
 * ========================================================= */

static void record_test_result(uint32_t passed)
{
    uint32_t index;

    index = g_motion_limits_test_report.total;

    if (index >= MOTION_LIMITS_TEST_CAPACITY)
    {
        g_motion_limits_test_report.failed++;
        return;
    }

    g_motion_limits_test_report.results[index] =
        (passed != 0U) ? 1U : 0U;

    g_motion_limits_test_report.total++;

    if (passed != 0U)
    {
        g_motion_limits_test_report.passed++;
    }
    else
    {
        g_motion_limits_test_report.failed++;
    }
}


/*
 * 在给定关节赋值，其余关节保持零位，
 * 验证六轴位置检查结果。
 */
static uint32_t test_one_position(
    uint32_t joint,
    robot_joint_position_t position,
    robot_status_t expected_status
)
{
    robot_joint_positions_t positions = {{0}};

    positions.value[joint] = position;

    return (
        motion_limits_check_positions(&positions)
        == expected_status
    ) ? 1U : 0U;
}


/*
 * 只改变指定关节，
 * 检查展开后的完整六轴结果。
 */
static uint32_t test_one_unwrap(
    uint32_t joint,
    robot_joint_position_t current_position,
    robot_joint_angle_t canonical_angle,
    robot_joint_position_t expected_position
)
{
    robot_joint_positions_t current = {{0}};
    robot_joint_positions_t actual = {{0}};
    robot_joint_angles_t canonical = {{0}};
    robot_status_t status;
    uint32_t i;

    current.value[joint] = current_position;
    canonical.value[joint] = canonical_angle;

    status = motion_limits_unwrap_nearest(
        &canonical,
        &current,
        &actual
    );

    if (status != ROBOT_STATUS_OK)
    {
        return 0U;
    }

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        robot_joint_position_t expected;

        expected = (i == joint) ? expected_position : 0;

        if (actual.value[i] != expected)
        {
            return 0U;
        }
    }

    return 1U;
}


/* =========================================================
 * Boundary and Error Tests
 * ========================================================= */

static void run_check_tests(void)
{
    robot_joint_positions_t zero = {{0}};

    record_test_result(
        motion_limits_check_positions(NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    record_test_result(
        motion_limits_check_positions(&zero)
        == ROBOT_STATUS_OK
    );

    record_test_result(test_one_position(0U, 36000, ROBOT_STATUS_OK));
    record_test_result(test_one_position(0U, -36000, ROBOT_STATUS_OK));
    record_test_result(test_one_position(2U, 18000, ROBOT_STATUS_OK));
    record_test_result(test_one_position(2U, -18000, ROBOT_STATUS_OK));
    record_test_result(test_one_position(0U, 36001, ROBOT_STATUS_ERROR_OUT_OF_RANGE));
    record_test_result(test_one_position(0U, -36001, ROBOT_STATUS_ERROR_OUT_OF_RANGE));
    record_test_result(test_one_position(2U, 18001, ROBOT_STATUS_ERROR_OUT_OF_RANGE));
    record_test_result(test_one_position(2U, -18001, ROBOT_STATUS_ERROR_OUT_OF_RANGE));
}


static void run_unwrap_tests(void)
{
    robot_joint_positions_t current = {{0}};
    robot_joint_positions_t actual = {{0}};
    robot_joint_angles_t canonical = {{0}};
    uint32_t i;
    uint32_t all_zero;

    record_test_result(
        motion_limits_unwrap_nearest(NULL, &current, &actual)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    record_test_result(
        motion_limits_unwrap_nearest(&canonical, NULL, &actual)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    record_test_result(
        motion_limits_unwrap_nearest(&canonical, &current, NULL)
        == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    all_zero = (
        motion_limits_unwrap_nearest(&canonical, &current, &actual)
        == ROBOT_STATUS_OK
    ) ? 1U : 0U;

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        if (actual.value[i] != 0)
        {
            all_zero = 0U;
        }
    }

    record_test_result(all_zero);

    /* +/-180° 规范化回绕。 */
    record_test_result(test_one_unwrap(0U, 17500, -17000, 19000));
    record_test_result(test_one_unwrap(0U, -17500, 17000, -19000));

    /* 关节已经转过接近完整一圈。 */
    record_test_result(test_one_unwrap(0U, 35000, -1000, 35000));
    record_test_result(test_one_unwrap(0U, -35000, 1000, -35000));

    /* J3 仅允许 +/-180°：最近等效角越限时退回合法解。 */
    record_test_result(test_one_unwrap(2U, -17500, 17000, 17000));
    record_test_result(test_one_unwrap(2U, 17500, -17000, -17000));

    /* 半圈平局：优先较大的连续角。 */
    record_test_result(test_one_unwrap(2U, 0, -18000, 18000));
    record_test_result(test_one_unwrap(2U, -18000, -18000, -18000));

    /* 不接受非 Canonical 输入。 */
    canonical.value[0] = 18000;
    record_test_result(
        motion_limits_unwrap_nearest(&canonical, &current, &actual)
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
    );

    canonical.value[0] = -18001;
    record_test_result(
        motion_limits_unwrap_nearest(&canonical, &current, &actual)
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
    );

    canonical.value[0] = 0;
    current.value[2] = 18001;
    record_test_result(
        motion_limits_unwrap_nearest(&canonical, &current, &actual)
        == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    /* 出错时保持调用者已写入的输出不变。 */
    current.value[2] = 0;
    canonical.value[0] = 18000;
    actual.value[0] = 12345;
    record_test_result(
        motion_limits_unwrap_nearest(&canonical, &current, &actual)
            == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
        && actual.value[0] == 12345
    );

    /* 多个关节同时跨 +/-180°。 */
    {
        robot_joint_positions_t multi_current = {{0}};
        robot_joint_positions_t multi_result = {{0}};
        robot_joint_angles_t multi_target = {{0}};

        multi_current.value[0] = 17500;
        multi_current.value[1] = -17500;
        multi_target.value[0] = -17000;
        multi_target.value[1] = 17000;

        record_test_result(
            motion_limits_unwrap_nearest(
                &multi_target,
                &multi_current,
                &multi_result
            ) == ROBOT_STATUS_OK
            && multi_result.value[0] == 19000
            && multi_result.value[1] == -19000
            && multi_result.value[2] == 0
        );
    }

    /* 正负完整一圈边界与无跨界情形。 */
    record_test_result(test_one_unwrap(0U, 36000, 0, 36000));
    record_test_result(test_one_unwrap(0U, -36000, 0, -36000));
    record_test_result(test_one_unwrap(0U, 6500, 6500, 6500));

    /* 后面的关节非法时，前面关节的临时结果不得泄漏。 */
    {
        robot_joint_positions_t late_current = {{0}};
        robot_joint_positions_t late_output = {{0}};
        robot_joint_angles_t late_target = {{0}};

        late_current.value[0] = 17500;
        late_target.value[0] = -17000;
        late_target.value[5] = 18000;
        late_output.value[0] = 7777;

        record_test_result(
            motion_limits_unwrap_nearest(
                &late_target,
                &late_current,
                &late_output
            ) == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
            && late_output.value[0] == 7777
        );
    }
}


/* =========================================================
 * GDB Completion Breakpoint
 * ========================================================= */

__attribute__((noinline))
void qemu_motion_limits_done(void)
{
#ifndef MOTION_LIMITS_HOST_TEST
    for (;;)
    {
        __asm volatile ("nop");
    }
#endif
}


/* =========================================================
 * Main
 * ========================================================= */

int main(void)
{
    uint32_t i;

    g_motion_limits_finished = 0U;
    g_motion_limits_test_report.total = 0U;
    g_motion_limits_test_report.passed = 0U;
    g_motion_limits_test_report.failed = 0U;

    for (i = 0U; i < MOTION_LIMITS_TEST_CAPACITY; i++)
    {
        g_motion_limits_test_report.results[i] = 0U;
    }

    run_check_tests();
    run_unwrap_tests();

    g_motion_limits_finished = 1U;
    qemu_motion_limits_done();

#ifdef MOTION_LIMITS_HOST_TEST
    return (g_motion_limits_test_report.failed == 0U) ? 0 : 1;
#else
    return 0;
#endif
}
