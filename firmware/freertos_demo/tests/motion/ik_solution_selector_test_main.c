
/*
 * File: ik_solution_selector_test_main.c
 *
 * Native regression tests for IK candidate selection.
 *
 * Compatible with Windows MSVC and GCC.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ik_solution_selector.h"


#define CHECK(condition)                                  \
    do                                                    \
    {                                                     \
        if (!(condition))                                 \
        {                                                 \
            printf(                                       \
                "IK selection FAIL at line %d: %s\n",     \
                __LINE__,                                 \
                #condition                                \
            );                                            \
            return 1;                                     \
        }                                                 \
    } while (0)


/* =========================================================
 * Test Data
 * ========================================================= */

static kinematics_ik_solutions_t examples(
    uint8_t count
)
{
    kinematics_ik_solutions_t set = {0};

    uint32_t i;

    set.count = count;

    for (
        i = 0U;
        i < count && i < KINEMATICS_MAX_IK_SOLUTIONS;
        i++
    )
    {
        /*
         * Keep q3 and q5 away from the singularities
         * unless a test explicitly overrides them.
         */
        set.solutions[i].value[2] = 9000;
        set.solutions[i].value[4] = 9000;
    }

    return set;
}


/* =========================================================
 * Main
 * ========================================================= */

int main(void)
{
    kinematics_ik_solutions_t set;

    robot_joint_positions_t current = {{0}};

    ik_selection_result_t output;
    ik_selection_result_t saved;

    uint32_t checks = 0U;


    /* -----------------------------------------------------
     * NULL pointer validation
     * ----------------------------------------------------- */

    set = examples(1);

    CHECK(
        ik_solution_selector_select(
            NULL,
            &current,
            &output
        ) == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        ik_solution_selector_select(
            &set,
            NULL,
            &output
        ) == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            NULL
        ) == ROBOT_STATUS_ERROR_NULL_POINTER
    );

    checks += 3U;


    /* -----------------------------------------------------
     * Invalid solution counts
     * ----------------------------------------------------- */

    set = examples(0);

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_ERROR_NO_SOLUTION
    );

    set.count = KINEMATICS_MAX_IK_SOLUTIONS + 1U;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
    );

    checks += 2U;


    /* -----------------------------------------------------
     * Invalid current position / canonical input
     * ----------------------------------------------------- */

    set = examples(1);

    current.value[2] = 18001;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_ERROR_OUT_OF_RANGE
    );

    current.value[2] = 0;

    set.solutions[0].value[1] = 18000;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_ERROR_INVALID_ARGUMENT
    );

    checks += 2U;


    /* -----------------------------------------------------
     * Canonical angle -> continuous position
     *
     * Current J1: +175 degrees.
     * Target J1:  -170 degrees canonical.
     *
     * Correct continuous target: +190 degrees.
     * Actual movement: 15 degrees.
     * ----------------------------------------------------- */

    set = examples(2);

    current.value[0] = 17500;

    set.solutions[0].value[0] = -17000;
    set.solutions[1].value[0] = 0;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_OK
    );

    CHECK(output.candidate_index == 0U);

    CHECK(output.target_positions.value[0] == 19000);

    CHECK(
        output.total_travel_raw
        == 1500U + 9000U + 9000U
    );

    checks += 4U;


    /* -----------------------------------------------------
     * Joint-limit margin
     *
     * Candidate 0:
     *   J1 = +350 degrees, zero movement,
     *   but very close to the +360 degree limit.
     *
     * Candidate 1:
     *   J1 = +300 degrees, 50 degrees movement,
     *   but larger safety margin.
     *
     * Expected: candidate 1 under current weights.
     * ----------------------------------------------------- */

    set = examples(2);

    current.value[0] = 35000;
    current.value[2] = 9000;
    current.value[4] = 9000;

    set.solutions[0].value[0] = -1000;
    set.solutions[1].value[0] = -6000;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_OK
    );

    CHECK(output.candidate_index == 1U);

    checks += 2U;


    /* -----------------------------------------------------
     * Wrist singularity rejection
     *
     * Candidate 0: q5 = 0.05 degrees.
     * Candidate 1: q5 = 30 degrees.
     *
     * Candidate 0 must be rejected.
     * ----------------------------------------------------- */

    set = examples(2);

    current.value[0] = 0;
    current.value[4] = 100;

    set.solutions[0].value[4] = 5;
    set.solutions[1].value[4] = 3000;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_OK
    );

    CHECK(output.candidate_index == 1U);

    checks += 2U;


    /* -----------------------------------------------------
     * All candidates singular
     *
     * Output must remain unchanged on failure.
     * ----------------------------------------------------- */

    set = examples(1);

    set.solutions[0].value[2] = 0;

    memset(&output, 0x5A, sizeof(output));
    saved = output;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_ERROR_SINGULAR
    );

    CHECK(
        memcmp(
            &saved,
            &output,
            sizeof(output)
        ) == 0
    );

    checks += 2U;


    /* -----------------------------------------------------
     * Stable tie-breaking
     *
     * Identical valid candidates:
     * select the earlier index.
     * ----------------------------------------------------- */

    set = examples(2);

    current.value[4] = 9000;

    CHECK(
        ik_solution_selector_select(
            &set,
            &current,
            &output
        ) == ROBOT_STATUS_OK
    );

    CHECK(output.candidate_index == 0U);

    checks += 2U;


    /* -----------------------------------------------------
     * Final report
     * ----------------------------------------------------- */

    printf(
        "IK selector host: %u checks PASSED\n",
        checks
    );

    return 0;
}
