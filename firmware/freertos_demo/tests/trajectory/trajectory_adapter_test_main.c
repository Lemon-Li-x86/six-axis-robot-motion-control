/*
 * Integration-only host test: existing model limits + IK result -> stage 4.
 * Uses the production model, motion limits and trajectory implementations.
 */
#include <stdio.h>
#include "trajectory_request_adapter.h"

static int failures = 0;
#define CHECK(x) do { if (!(x)) { failures++;   fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); } } while (0)

int main(void)
{
    robot_joint_positions_t current = {{0}};
    ik_selection_result_t selection = {0};
    robot_joint_velocities_t vmax = {{90, 90, 90, 90, 90, 90}};
    trajectory_joint_accelerations_t amax = {{180, 180, 180, 180, 180, 180}};
    trajectory_trapezoid_plan_t plan = {0};
    trajectory_joint_sample_t sample = {0};

    selection.target_positions.value[0] = 1000;
    selection.target_positions.value[2] = -2000;
    CHECK(trajectory_request_from_ik(
        &current, &selection, &vmax, &amax, &plan) == ROBOT_STATUS_OK);
    CHECK(plan.total_time_s > 0.0);
    CHECK(trajectory_trapezoid_sample(&plan, plan.total_time_s, &sample)
          == ROBOT_STATUS_OK);
    CHECK(sample.positions.value[0] == 1000);
    CHECK(sample.positions.value[2] == -2000);

    plan.total_time_s = 111.0;
    selection.target_positions.value[2] = 19000;
    CHECK(trajectory_request_from_ik(
        &current, &selection, &vmax, &amax, &plan)
          == ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    CHECK(plan.total_time_s == 111.0);

    selection.target_positions.value[2] = -2000;
    current.value[2] = 19000;
    CHECK(trajectory_request_from_ik(
        &current, &selection, &vmax, &amax, &plan)
          == ROBOT_STATUS_ERROR_OUT_OF_RANGE);
    CHECK(plan.total_time_s == 111.0);

    current.value[2] = 0;
    vmax.value[0] = 0;
    CHECK(trajectory_request_from_ik(
        &current, &selection, &vmax, &amax, &plan)
          == ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(plan.total_time_s == 111.0);

    CHECK(trajectory_request_from_ik(
        NULL, &selection, &vmax, &amax, &plan)
          == ROBOT_STATUS_ERROR_NULL_POINTER);

    printf("Stage4 IK adapter failures: %d\n", failures);
    return failures == 0 ? 0 : 1;
}
