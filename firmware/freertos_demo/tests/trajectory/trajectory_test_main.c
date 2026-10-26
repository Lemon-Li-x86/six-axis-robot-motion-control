/*
 * Stage 4 host-only regression for standalone trajectory algorithms.
 * No UR5 constants, FreeRTOS, QEMU or motor driver needed.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "trajectory.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { ++checks; if (!(cond)) {     ++failures; fprintf(stderr, "FAIL %s:%d: %s\n",         __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(actual, expected, tolerance)     CHECK(fabs((double)(actual) - (double)(expected)) <= (tolerance))

static void test_cubic(void)
{
    robot_joint_positions_t start = {{0, 17000, -9000, 0, 0, 0}};
    robot_joint_positions_t goal = {{10000, 19000, -9000, 0, 0, 0}};
    robot_joint_velocities_t zero = {{0}};
    trajectory_polynomial_plan_t plan = {0};
    trajectory_joint_sample_t sample = {0};

    CHECK(trajectory_polynomial_init_cubic(
        &plan, &start, &goal, &zero, &zero, 2.0) == ROBOT_STATUS_OK);
    CHECK(trajectory_polynomial_sample(&plan, 0.0, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 0);
    CHECK(sample.positions.value[1] == 17000);
    NEAR(sample.velocities.value[0], 0.0, 0.0001);

    CHECK(trajectory_polynomial_sample(&plan, 1.0, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 5000);
    CHECK(sample.positions.value[1] == 18000);
    NEAR(sample.velocities.value[0], 75.0, 0.0001);
    NEAR(sample.accelerations.value[0], 0.0, 0.0001);

    CHECK(trajectory_polynomial_sample(&plan, 5.0, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 10000);
    CHECK(sample.positions.value[1] == 19000);
    CHECK(sample.positions.value[2] == -9000);
    NEAR(sample.velocities.value[0], 0.0, 0.0001);
}

static void test_quintic(void)
{
    robot_joint_positions_t start = {{0}};
    robot_joint_positions_t goal = {{10000, -5000, 0, 0, 0, 0}};
    robot_joint_velocities_t v0 = {{2, 0, 0, 0, 0, 0}};
    robot_joint_velocities_t v1 = {{-3, 0, 0, 0, 0, 0}};
    trajectory_joint_accelerations_t a0 = {{4, 0, 0, 0, 0, 0}};
    trajectory_joint_accelerations_t a1 = {{-6, 0, 0, 0, 0, 0}};
    trajectory_polynomial_plan_t plan = {0};
    trajectory_joint_sample_t sample = {0};

    CHECK(trajectory_polynomial_init_quintic(
        &plan, &start, &goal, &v0, &v1, &a0, &a1, 4.0)
        == ROBOT_STATUS_OK);
    CHECK(trajectory_polynomial_sample(&plan, 0.0, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 0);
    NEAR(sample.velocities.value[0], 2.0, 0.0001);
    NEAR(sample.accelerations.value[0], 4.0, 0.0001);
    CHECK(trajectory_polynomial_sample(&plan, 4.0, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 10000);
    CHECK(sample.positions.value[1] == -5000);
    NEAR(sample.velocities.value[0], -3.0, 0.0001);
    NEAR(sample.accelerations.value[0], -6.0, 0.0001);

    CHECK(trajectory_polynomial_sample(&plan, 2.0, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] >= 0 && sample.positions.value[0] <= 10000);
}

static void test_trapezoid(void)
{
    robot_joint_positions_t start = {{0}};
    robot_joint_positions_t goal = {{10000, -5000, 0, 0, 0, 0}};
    robot_joint_velocities_t vmax = {{50, 25, 10, 10, 10, 10}};
    trajectory_joint_accelerations_t amax = {{100, 50, 20, 20, 20, 20}};
    trajectory_trapezoid_plan_t plan = {0};
    trajectory_joint_sample_t sample = {0};
    int i;
    CHECK(trajectory_trapezoid_init(
        &plan, &start, &goal, &vmax, &amax) == ROBOT_STATUS_OK);
    NEAR(plan.acceleration_time_s, 0.5, 1e-10);
    NEAR(plan.cruise_time_s, 1.5, 1e-10);
    NEAR(plan.total_time_s, 2.5, 1e-10);
    CHECK(trajectory_trapezoid_sample(&plan, 1.25, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 5000);
    CHECK(sample.positions.value[1] == -2500);
    NEAR(sample.velocities.value[0], 50.0, 0.0001);

    for (i = 0; i <= 50; ++i)
    {
        double t = plan.total_time_s * (double)i / 50.0;
        CHECK(trajectory_trapezoid_sample(&plan, t, &sample)
              == ROBOT_STATUS_OK);
        CHECK(sample.positions.value[0] >= 0 &&
              sample.positions.value[0] <= goal.value[0]);
        CHECK(sample.positions.value[1] <= 0 &&
              sample.positions.value[1] >= goal.value[1]);
        CHECK(fabs((double)sample.velocities.value[0]) <= 50.0001);
        CHECK(fabs((double)sample.velocities.value[1]) <= 25.0001);
        CHECK(fabs((double)sample.accelerations.value[0]) <= 100.0001);
        CHECK(fabs((double)sample.accelerations.value[1]) <= 50.0001);
    }
    CHECK(trajectory_trapezoid_sample(&plan, 100.0, &sample)
          == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 10000);
    CHECK(sample.positions.value[1] == -5000);
    NEAR(sample.velocities.value[0], 0.0, 0.0001);
    NEAR(sample.accelerations.value[0], 0.0, 0.0001);
}

static void test_triangular_and_stationary(void)
{
    robot_joint_positions_t a = {{0}};
    robot_joint_positions_t b = {{100, 0, 0, 0, 0, 0}};
    robot_joint_velocities_t vmax = {{1000, 100, 100, 100, 100, 100}};
    trajectory_joint_accelerations_t amax = {{4, 4, 4, 4, 4, 4}};
    trajectory_trapezoid_plan_t plan = {0};
    trajectory_joint_sample_t sample = {0};

    CHECK(trajectory_trapezoid_init(
        &plan, &a, &b, &vmax, &amax) == ROBOT_STATUS_OK);
    NEAR(plan.cruise_time_s, 0.0, 1e-10);
    NEAR(plan.total_time_s, 1.0, 1e-10);
    CHECK(trajectory_trapezoid_sample(&plan, 0.5, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 50);

    CHECK(trajectory_trapezoid_init(
        &plan, &a, &a, &vmax, &amax) == ROBOT_STATUS_OK);
    NEAR(plan.total_time_s, 0.0, 1e-10);
    CHECK(trajectory_trapezoid_sample(&plan, 1234, &sample) == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 0);
    NEAR(sample.velocities.value[0], 0.0, 1e-10);
}

static void test_invalid_inputs_and_preservation(void)
{
    robot_joint_positions_t start = {{0}};
    robot_joint_positions_t goal = {{10000, 0, 0, 0, 0, 0}};
    robot_joint_velocities_t v = {{0}};
    robot_joint_velocities_t vmax = {{50, 50, 50, 50, 50, 50}};
    trajectory_joint_accelerations_t a = {{0}};
    trajectory_joint_accelerations_t amax = {{100, 100, 100, 100, 100, 100}};
    trajectory_polynomial_plan_t poly = {0};
    trajectory_trapezoid_plan_t trapezoid = {0};
    trajectory_joint_sample_t sample = {0};

    poly.degree = 99U;
    CHECK(trajectory_polynomial_init_cubic(
        &poly, &start, &goal, &v, &v, -1.0)
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(poly.degree == 99U);
    CHECK(trajectory_polynomial_init_quintic(
        &poly, &start, &goal, &v, &v, &a, &a, 1.0)
        == ROBOT_STATUS_OK);
    sample.positions.value[0] = 123456;
    CHECK(trajectory_polynomial_sample(&poly, -0.1, &sample)
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(sample.positions.value[0] == 123456);
    CHECK(trajectory_polynomial_sample(NULL, 1.0, &sample)
        == ROBOT_STATUS_ERROR_NULL_POINTER);

    vmax.value[0] = 0;
    trapezoid.total_time_s = 123.0;
    CHECK(trajectory_trapezoid_init(
        &trapezoid, &start, &goal, &vmax, &amax)
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    NEAR(trapezoid.total_time_s, 123.0, 1e-10);
    CHECK(trajectory_trapezoid_sample(&trapezoid, -1.0, &sample)
        == ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(sample.positions.value[0] == 123456);
}

int main(void)
{
    test_cubic();
    test_quintic();
    test_trapezoid();
    test_triangular_and_stationary();
    test_invalid_inputs_and_preservation();
    printf("Trajectory checks: %d, failures: %d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
