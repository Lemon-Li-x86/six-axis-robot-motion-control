
/*
 * File: ik_batch_test_main.c
 *
 * Cortex-M4 / QEMU
 * UR5 Inverse Kinematics Batch Diagnostic
 *
 * Test design:
 *
 * Group A: 19 poses
 *   J3 = +60 degrees
 *   J5 sweeps from -5 to +5 degrees
 *
 * Group B: 19 poses
 *   J3 = 0 degrees
 *   J5 uses the same sweep values
 *
 * Group C: 90 deterministic random poses
 *
 * Total: 128 poses
 *
 * For each pose:
 *   1. Generate input joint angles
 *   2. Compute Forward Kinematics
 *   3. Compute Analytic Inverse Kinematics
 *   4. Store return codes and candidate joints
 *
 * Results are exported through GDB.
 *
 * Important:
 * This program diagnoses IK behavior.
 * It does not reject singular cases automatically.
 */

#include <stdint.h>

#include "kinematics.h"
#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * Test Configuration
 * ========================================================= */

#define IK_SAMPLE_COUNT 128U

#define WRIST_SWEEP_COUNT 19U

#define CONTROLLED_SAMPLE_COUNT \
    (2U * WRIST_SWEEP_COUNT)


/*
 * J5 test values.
 *
 * Unit: 0.01 degree
 *
 * Covers:
 *
 * -5.00, -2.00, -1.00, -0.50, -0.20,
 * -0.10, -0.05, -0.02, -0.01, 0.00,
 * +0.01, +0.02, +0.05, +0.10, +0.20,
 * +0.50, +1.00, +2.00, +5.00
 */

static const int16_t wrist_sweep_raw[
    WRIST_SWEEP_COUNT
] =
{
    -500,
    -200,
    -100,
    -50,
    -20,
    -10,
    -5,
    -2,
    -1,
    0,
    1,
    2,
    5,
    10,
    20,
    50,
    100,
    200,
    500
};


/* =========================================================
 * Binary Result Record
 * ========================================================= */

/*
 * Binary layout:
 *
 * input_joints:
 *     6 x int16_t = 12 bytes
 *
 * fk_status:
 *     1 x int32_t = 4 bytes
 *
 * ik_status:
 *     1 x int32_t = 4 bytes
 *
 * solution_count:
 *     1 x uint32_t = 4 bytes
 *
 * solutions:
 *     8 x 6 x int16_t = 96 bytes
 *
 * Total:
 *     120 bytes
 *
 * Python parser:
 *     struct.Struct("<6hiiI48h")
 */

typedef struct
{
    int16_t input_joints[
        ROBOT_JOINT_COUNT
    ];

    int32_t fk_status;

    int32_t ik_status;

    uint32_t solution_count;

    int16_t solutions[
        KINEMATICS_MAX_IK_SOLUTIONS
    ][
        ROBOT_JOINT_COUNT
    ];

} ik_sample_t;


_Static_assert(
    sizeof(ik_sample_t) == 120U,
    "Unexpected IK record layout"
);


/* =========================================================
 * Global Test Results
 * ========================================================= */

/*
 * Volatile:
 *
 * Keep test results observable through GDB.
 */

volatile ik_sample_t g_ik_samples[
    IK_SAMPLE_COUNT
];

volatile uint32_t g_ik_finished = 0U;


/* =========================================================
 * Deterministic Random Generator
 * ========================================================= */

/*
 * Linear Congruential Generator.
 *
 * A fixed seed produces reproducible test cases.
 */

static uint32_t next_random(
    uint32_t *state
)
{
    *state =
        1664525UL * (*state)
        + 1013904223UL;

    return *state;
}


/* =========================================================
 * Generate Controlled Test Pose
 * ========================================================= */

/*
 * Index 0..18:
 *
 * Group A
 * J3 = +60 degrees
 *
 *
 * Index 19..37:
 *
 * Group B
 * J3 = 0 degrees
 *
 *
 * Other joints:
 *
 * J1 = +30 degrees
 * J2 = -45 degrees
 * J4 = +20 degrees
 * J6 = -10 degrees
 *
 * J5 varies according to wrist_sweep_raw.
 */

static void generate_controlled_pose(
    uint32_t index,
    robot_joint_angles_t *input
)
{
    uint32_t sweep_index;

    sweep_index =
        index % WRIST_SWEEP_COUNT;

    input->value[0] = 3000;

    input->value[1] = -4500;

    if (
        index < WRIST_SWEEP_COUNT
    )
    {
        /*
         * Group A:
         * J3 = +60 degrees
         */
        input->value[2] = 6000;
    }
    else
    {
        /*
         * Group B:
         * J3 = 0 degrees
         */
        input->value[2] = 0;
    }

    input->value[3] = 2000;

    input->value[4] =
        wrist_sweep_raw[sweep_index];

    input->value[5] = -1000;
}


/* =========================================================
 * Generate Random Test Pose
 * ========================================================= */

/*
 * Each joint:
 *
 * [-16000, +16000]
 *
 * Unit:
 *
 * 0.01 degree
 *
 * Actual angle range:
 *
 * [-160, +160] degrees
 */

static void generate_random_pose(
    uint32_t *state,
    robot_joint_angles_t *input
)
{
    uint32_t joint_index;

    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        uint32_t random_value;

        random_value =
            next_random(state);

        input->value[joint_index] =
            (int16_t)(
                (int32_t)(
                    random_value % 32001U
                )
                - 16000
            );
    }
}


/* =========================================================
 * Generate Input Pose
 * ========================================================= */

static void generate_test_pose(
    uint32_t index,
    uint32_t *random_state,
    robot_joint_angles_t *input
)
{
    if (
        index < CONTROLLED_SAMPLE_COUNT
    )
    {
        generate_controlled_pose(
            index,
            input
        );
    }
    else
    {
        generate_random_pose(
            random_state,
            input
        );
    }
}


/* =========================================================
 * Save Input Joint Angles
 * ========================================================= */

static void save_input_joints(
    uint32_t sample_index,
    const robot_joint_angles_t *input
)
{
    uint32_t joint_index;

    for (
        joint_index = 0U;
        joint_index < ROBOT_JOINT_COUNT;
        joint_index++
    )
    {
        g_ik_samples[
            sample_index
        ].input_joints[
            joint_index
        ] =
            input->value[
                joint_index
            ];
    }
}


/* =========================================================
 * Save IK Candidate Solutions
 * ========================================================= */

static void save_ik_solutions(
    uint32_t sample_index,
    const kinematics_ik_solutions_t *solutions
)
{
    uint32_t solution_index;

    uint32_t joint_index;

    uint32_t copy_count;

    /*
     * Preserve the actual returned count.
     */
    g_ik_samples[
        sample_index
    ].solution_count =
        (uint32_t)solutions->count;

    /*
     * Prevent the test program itself
     * from writing past the result array.
     */
    copy_count =
        (uint32_t)solutions->count;

    if (
        copy_count
        >
        KINEMATICS_MAX_IK_SOLUTIONS
    )
    {
        copy_count =
            KINEMATICS_MAX_IK_SOLUTIONS;
    }

    for (
        solution_index = 0U;
        solution_index < copy_count;
        solution_index++
    )
    {
        for (
            joint_index = 0U;
            joint_index < ROBOT_JOINT_COUNT;
            joint_index++
        )
        {
            g_ik_samples[
                sample_index
            ].solutions[
                solution_index
            ][
                joint_index
            ] =
                solutions->solutions[
                    solution_index
                ].value[
                    joint_index
                ];
        }
    }
}


/* =========================================================
 * Execute One IK Test
 * ========================================================= */

static void run_ik_test(
    uint32_t sample_index,
    uint32_t *random_state
)
{
    robot_joint_angles_t input =
    {
        {0}
    };

    robot_transform_t target =
    {
        {{0}}
    };

    kinematics_ik_solutions_t solutions =
    {
        0
    };

    robot_status_t fk_status;

    robot_status_t ik_status;

    /*
     * 1. Generate input joints.
     */
    generate_test_pose(
        sample_index,
        random_state,
        &input
    );

    /*
     * 2. Save original configuration.
     */
    save_input_joints(
        sample_index,
        &input
    );

    /*
     * 3. Forward Kinematics.
     */
    fk_status =
        kinematics_forward(
            &input,
            &target
        );

    g_ik_samples[
        sample_index
    ].fk_status =
        (int32_t)fk_status;

    /*
     * FK failed:
     *
     * Record the failure and skip IK.
     */
    if (
        fk_status != ROBOT_STATUS_OK
    )
    {
        g_ik_samples[
            sample_index
        ].ik_status =
            (int32_t)
            ROBOT_STATUS_ERROR_INTERNAL;

        g_ik_samples[
            sample_index
        ].solution_count = 0U;

        return;
    }

    /*
     * 4. Analytic Inverse Kinematics.
     */
    ik_status =
        kinematics_inverse(
            &target,
            &solutions
        );

    g_ik_samples[
        sample_index
    ].ik_status =
        (int32_t)ik_status;

    /*
     * 5. Save all returned candidates.
     */
    save_ik_solutions(
        sample_index,
        &solutions
    );
}


/* =========================================================
 * GDB Completion Breakpoint
 * ========================================================= */

/*
 * QEMU execution stops here.
 *
 * Results remain available in RAM.
 */

__attribute__((noinline))
void qemu_ik_batch_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}


/* =========================================================
 * Main
 * ========================================================= */

int main(void)
{
    uint32_t sample_index;

    /*
     * Fixed random seed.
     */
    uint32_t random_state = 20261012UL;

    g_ik_finished = 0U;

    /*
     * Execute 128 test poses.
     */
    for (
        sample_index = 0U;
        sample_index < IK_SAMPLE_COUNT;
        sample_index++
    )
    {
        run_ik_test(
            sample_index,
            &random_state
        );
    }

    /*
     * Mark the batch as completed.
     */
    g_ik_finished = 1U;

    /*
     * Stop for GDB inspection/export.
     */
    qemu_ik_batch_done();

    return 0;
}
