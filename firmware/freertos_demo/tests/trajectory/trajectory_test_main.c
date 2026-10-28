/*
 * Stage 4 host-only regression for standalone trajectory algorithms.
 * No UR5 constants, FreeRTOS, QEMU or motor driver needed.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "trajectory.h"

/*
 * Public joint position scale is ROBOT_JOINT_ANGLE_UNIT_DEG (0.01F).
 * Its binary32 representation is slightly smaller than mathematical 0.01,
 * and conversion to double preserves that original rounding.
 * As a result, the expected 1.5s, 2.5s and 1.0s timings vary by about
 * 11-45 ns, so a 0.1 ns assertion is NOT valid for this public input type.
 *
 * One microsecond is still five orders of magnitude tighter than the
 * configured 10-20 ms control/trajectory periods, and checks that the
 * profile structure and total time remain mathematically correct.
 *
 * This constant applies to schedule time assertions only, never to
 * physical position, velocity, acceleration or IK accuracy gates.
 */
#define TRAJECTORY_TEST_TIME_TOLERANCE_S 1e-6

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
    NEAR(plan.cruise_time_s, 1.5, TRAJECTORY_TEST_TIME_TOLERANCE_S);
    NEAR(plan.total_time_s, 2.5, TRAJECTORY_TEST_TIME_TOLERANCE_S);
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
    NEAR(plan.total_time_s, 1.0, TRAJECTORY_TEST_TIME_TOLERANCE_S);
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

static void test_polynomial_whole_curve_limits(void)
{
    robot_joint_positions_t start = {{0}}, goal = {{100,0,0,0,0,0}};
    robot_joint_positions_t low = {{-1000,-1000,-1000,-1000,-1000,-1000}};
    robot_joint_positions_t high = {{1000,1000,1000,1000,1000,1000}};
    robot_joint_velocities_t zero = {{0}}, vmax = {{2,2,2,2,2,2}};
    trajectory_joint_accelerations_t azero = {{0}}, amax = {{6,6,6,6,6,6}};
    trajectory_polynomial_plan_t plan = {0}, saved;

    /* Rest-to-rest quintic: vmax=1.875 deg/s, |amax|=10/sqrt(3).
     * Both endpoint velocities and accelerations are zero: endpoint-only
     * checks cannot discover either of the following violations. */
    CHECK(trajectory_polynomial_init_quintic(&plan,&start,&goal,
        &zero,&zero,&azero,&azero,1.0)==ROBOT_STATUS_OK);
    saved=plan;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_OK);
    CHECK(memcmp(&saved,&plan,sizeof(plan))==0);
    vmax.value[0]=1.87F;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    vmax.value[0]=2.0F; amax.value[0]=5.77F;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    amax.value[0]=6.0F;
    plan.goal_positions.value[0]=101;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    plan=saved;
    plan.coefficients_deg[0][2]=NAN;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    plan=saved; plan.duration_s=1e-200;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    CHECK(trajectory_polynomial_validate_limits(NULL,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_NULL_POINTER);
    plan=saved; vmax.value[0]=0.0F;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_INVALID_ARGUMENT);

    /* Equal valid endpoints can still overshoot the position range. */
    goal=start; zero.value[0]=4.0F;
    vmax.value[0]=4.0F; amax.value[0]=8.0F;
    {
        robot_joint_velocities_t end=zero;
        end.value[0]=-4.0F;
        CHECK(trajectory_polynomial_init_cubic(&plan,&start,&goal,
            &zero,&end,1.0)==ROBOT_STATUS_OK);
    }
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_OK);
    high.value[0]=50;
    CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
        ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
}

/* Independent dense evaluation across varied boundary conditions checks
 * acceptance/rejection with a 2% margin around observed rate extrema.
 * This exercises both degree-three and degree-five interior root patterns. */
static void test_polynomial_extrema_against_dense_samples(void)
{
    robot_joint_positions_t start={{0}}, goal={{0}};
    robot_joint_positions_t low={{-1000000,-1000000,-1000000,-1000000,-1000000,-1000000}};
    robot_joint_positions_t high={{1000000,1000000,1000000,1000000,1000000,1000000}};
    robot_joint_velocities_t v0={{0}},v1={{0}},vmax={{1000,1000,1000,1000,1000,1000}};
    trajectory_joint_accelerations_t a0={{0}},a1={{0}},amax={{1000,1000,1000,1000,1000,1000}};
    uint32_t trial, seed=0x51A4U;
    for(trial=0U;trial<32U;trial++) {
        trajectory_polynomial_plan_t plan={0};
        trajectory_joint_sample_t sample;
        double peak_v=0.0,peak_a=0.0,duration=0.5+(double)trial/16.0;
        uint32_t i;
        seed=seed*1664525U+1013904223U;
        goal.value[0]=(int32_t)(seed%2001U)-1000;
        v0.value[0]=(robot_real_t)((int32_t)((seed>>8)%101U)-50);
        v1.value[0]=(robot_real_t)((int32_t)((seed>>16)%101U)-50);
        a0.value[0]=(robot_real_t)((int32_t)((seed>>4)%161U)-80);
        a1.value[0]=-a0.value[0];
        if(trial%2U==0U)
            CHECK(trajectory_polynomial_init_cubic(&plan,&start,&goal,
                &v0,&v1,duration)==ROBOT_STATUS_OK);
        else
            CHECK(trajectory_polynomial_init_quintic(&plan,&start,&goal,
                &v0,&v1,&a0,&a1,duration)==ROBOT_STATUS_OK);
        for(i=0U;i<=1000U;i++) {
            CHECK(trajectory_polynomial_sample(&plan,duration*(double)i/1000.0,
                &sample)==ROBOT_STATUS_OK);
            peak_v=fmax(peak_v,fabs((double)sample.velocities.value[0]));
            peak_a=fmax(peak_a,fabs((double)sample.accelerations.value[0]));
        }
        vmax.value[0]=(robot_real_t)(peak_v*1.02);
        amax.value[0]=(robot_real_t)(peak_a*1.02);
        CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
            ==ROBOT_STATUS_OK);
        vmax.value[0]=(robot_real_t)(peak_v*0.98);
        CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
            ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
        vmax.value[0]=(robot_real_t)(peak_v*1.02);
        amax.value[0]=(robot_real_t)(peak_a*0.98);
        CHECK(trajectory_polynomial_validate_limits(&plan,&low,&high,&vmax,&amax)
            ==ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    }
}

int main(void)
{
    test_cubic();
    test_quintic();
    test_trapezoid();
    test_triangular_and_stationary();
    test_invalid_inputs_and_preservation();
    test_polynomial_whole_curve_limits();
    test_polynomial_extrema_against_dense_samples();
    printf("Trajectory checks: %d, failures: %d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
