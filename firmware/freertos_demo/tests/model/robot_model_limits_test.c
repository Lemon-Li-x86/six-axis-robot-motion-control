/**
 * @file robot_model_limits_test.c
 * @brief 回归测试或夹具：robot model limits。
 */

#include <stddef.h>
#include <stdint.h>

#include "robot_model.h"

/* Test Configuration */

/*
 * 1 项模型指针检查 + 每轴 4 项限位检查。
 *
 * 总计：
 * 1 + 6 * 4 = 25 项。
 */
#define MODEL_LIMITS_TEST_COUNT \
    (1U + ROBOT_JOINT_COUNT * 4U)

/* Expected Joint Limits */

/*
 * 当前仿真 UR5 URDF 的关节限位。
 *
 * 单位：
 * 0.01 degree。
 *
 * 顺序：
 * J1、J2、J3、J4、J5、J6。
 */
static const robot_joint_positions_t expected_min_positions =
{
    .value =
    {
        -36000,
        -36000,
        -18000,
        -36000,
        -36000,
        -36000
    }
};

static const robot_joint_positions_t expected_max_positions =
{
    .value =
    {
        36000,
        36000,
        18000,
        36000,
        36000,
        36000
    }
};

/* Test Report */

/*
 * 通过 GDB 读取：
 *
 * g_model_limits_finished
 * g_model_limits_test_report.total
 * g_model_limits_test_report.passed
 * g_model_limits_test_report.failed
 * g_model_limits_test_report.results
 *
 * results[i]：
 * 1：对应检查通过；
 * 0：对应检查失败。
 */
typedef struct
{
    uint32_t total;
    uint32_t passed;
    uint32_t failed;

    uint32_t results[
        MODEL_LIMITS_TEST_COUNT
    ];

} model_limits_test_report_t;

volatile model_limits_test_report_t
    g_model_limits_test_report;

volatile uint32_t
    g_model_limits_finished = 0U;

/* Record Test Result */

static void record_test_result(
    uint32_t condition
)
{
    uint32_t index;
    uint32_t passed;

    index = g_model_limits_test_report.total;

    if (index >= MODEL_LIMITS_TEST_COUNT)
    {
        return;
    }

    passed = (condition != 0U) ? 1U : 0U;

    g_model_limits_test_report.results[index] = passed;
    g_model_limits_test_report.total++;

    if (passed != 0U)
    {
        g_model_limits_test_report.passed++;
    }
    else
    {
        g_model_limits_test_report.failed++;
    }
}

/* Joint Limit Validation */

static void run_joint_limit_test(void)
{
    const robot_model_t *model;
    uint32_t joint_index;

    model = robot_model_get_active();

    /*
     * 第 1 项：
     * Active Robot Model 不为空。
     */
    record_test_result(
        model != NULL
    );

    if (model == NULL)
    {
        return;
    }

    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        int32_t lower;
        int32_t upper;

        lower =
            model->joint_min_positions.value[joint_index];

        upper =
            model->joint_max_positions.value[joint_index];

        /*
         * 每轴检查 4 项：
         *
         * 1. 下限与 URDF 一致；
         * 2. 上限与 URDF 一致；
         * 3. 下限严格小于上限；
         * 4. 零位位于闭区间内。
         */
        record_test_result(
            lower == expected_min_positions.value[joint_index]
        );

        record_test_result(
            upper == expected_max_positions.value[joint_index]
        );

        record_test_result(
            lower < upper
        );

        record_test_result(
            lower <= 0 && upper >= 0
        );
    }
}

/* GDB Completion Breakpoint */

/*
 * 所有测试执行完毕后停在这里。
 * GDB 可读取 RAM 中保存的测试结果。
 */
__attribute__((noinline))
void qemu_model_limits_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}

/* Main */

int main(void)
{
    uint32_t index;

    g_model_limits_test_report.total = 0U;
    g_model_limits_test_report.passed = 0U;
    g_model_limits_test_report.failed = 0U;
    g_model_limits_finished = 0U;

    for (
        index = 0U;
        index < MODEL_LIMITS_TEST_COUNT;
        index++
    )
    {
        g_model_limits_test_report.results[index] = 0U;
    }

    run_joint_limit_test();

    g_model_limits_finished = 1U;

    qemu_model_limits_done();

    return 0;
}
