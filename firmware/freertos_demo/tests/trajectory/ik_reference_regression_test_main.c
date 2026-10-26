/*
 * Stage3 near-singular IK regression, executed on the host.
 * Reproduce the real QEMU J3=0, J5=+/-0.01 degree branch-loss pair.
 *
 * This tests a real joint reference, not a hidden oracle in production:
 * the reference is explicitly provided by the caller. It must never
 * loosen cosine geometry tolerance or bypass the motion selector.
 */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "kinematics.h"
#include "ik_reference_test_limits.h"

static int checks = 0, failures = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; \
    fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#expr); \
} } while (0)

static int close_to_reference(
    const robot_joint_angles_t *a,
    const robot_joint_angles_t *b)
{
    unsigned j;
    /* 0.25 degree is a search/matching window, NOT a trajectory gate. */
    for (j = 0; j < ROBOT_JOINT_COUNT; ++j)
    {
        int difference = (int)a->value[j] - (int)b->value[j];
        if (difference < -IK_REFERENCE_TEST_MATCH_WINDOW_RAW ||
            difference > IK_REFERENCE_TEST_MATCH_WINDOW_RAW) return 0;
    }
    return 1;
}

static int find_verified_reference(
    const robot_transform_t *requested,
    const kinematics_ik_solutions_t *solutions,
    const robot_joint_angles_t *reference)
{
    unsigned i, row, col;
    for (i = 0; i < solutions->count; ++i)
    {
        robot_transform_t actual = {{{0}}};
        double pos2 = 0.0, maxr = 0.0;
        if (!close_to_reference(&solutions->solutions[i], reference))
            continue;
        if (kinematics_forward(&solutions->solutions[i], &actual)
            != ROBOT_STATUS_OK) continue;
        for (row = 0; row < 3; ++row)
        {
            double dp = (double)actual.matrix[row][3] -
                (double)requested->matrix[row][3];
            pos2 += dp * dp;
            for (col = 0; col < 3; ++col)
            {
                double dr = fabs((double)actual.matrix[row][col] -
                    (double)requested->matrix[row][col]);
                if (dr > maxr) maxr = dr;
            }
        }
        if (pos2 <=
            (double)KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM *
            KINEMATICS_IK_MAX_POSITION_RESIDUAL_MM &&
            maxr <= KINEMATICS_IK_MAX_ROTATION_ELEMENT_RESIDUAL)
            return 1;
    }
    return 0;
}

static void run_case(robot_joint_angle_t wrist_raw)
{
    robot_joint_angles_t reference = {{3000,-4500,0,2000,0,-1000}};
    robot_joint_angles_t wrong = {{3000,-4500,0,2000,0,5000}};
    robot_joint_angles_t invalid = {{3000,-4500,0,2000,18000,-1000}};
    robot_transform_t pose = {{{0}}};
    kinematics_ik_solutions_t legacy = {0};
    kinematics_ik_solutions_t seeded = {0};
    kinematics_ik_solutions_t wrong_seeded = {0};
    robot_status_t st;

    reference.value[4] = wrist_raw;
    wrong.value[4] = wrist_raw;
    CHECK(kinematics_forward(&reference, &pose) == ROBOT_STATUS_OK);
    CHECK(kinematics_inverse(&pose, &legacy) == ROBOT_STATUS_OK);
    st = kinematics_inverse_with_reference(&pose, &reference, &seeded);
    CHECK(st == ROBOT_STATUS_OK);
    CHECK(seeded.count > 0 && seeded.count <= KINEMATICS_MAX_IK_SOLUTIONS);
    CHECK(find_verified_reference(&pose, &seeded, &reference));

    st = kinematics_inverse_with_reference(&pose, &wrong, &wrong_seeded);
    CHECK(st == ROBOT_STATUS_OK);
    /* Far q6 must not be imposed on the near-singular branch. */
    CHECK(wrong_seeded.count == legacy.count);
    CHECK(kinematics_inverse_with_reference(&pose, &invalid, &seeded)
          == ROBOT_STATUS_ERROR_INVALID_ARGUMENT);
    CHECK(kinematics_inverse_with_reference(&pose, NULL, &seeded)
          == ROBOT_STATUS_ERROR_NULL_POINTER);
}

int main(void)
{
    run_case(-1);
    run_case(1);
    printf("Seeded IK regression: checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
