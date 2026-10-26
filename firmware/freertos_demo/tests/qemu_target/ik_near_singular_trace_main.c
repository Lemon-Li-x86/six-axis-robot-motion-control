/*
 * Focused Stage3 branch-loss diagnosis on real Cortex-M4/QEMU.
 *
 * Test-only trace of the six wrist/elbow near-singularity cases observed
 * in qemu_kinematics_perf: indices 8,9,10,27,28,29.
 *
 * We do not assert "eight solutions required": true singularities merge
 * branches and the 0.01 degree public IK quantization removes exact copies.
 *
 * This executable does NOT change production IK or motor outputs.
 * GDB: break qemu_ik_trace_done; p g_ik_trace_done;
 *      p g_ik_trace[0] ... p g_ik_trace[5]
 */
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "robot_model.h"
#include "kinematics.h"
#include "kinematics_private.h"
#include "error_code.h"

/* Implemented only in ur5_analytic_ik.c when the test macro is defined;
 * reuses the production p13 helper, avoiding a second copy of its formula.
 */
robot_status_t ur5_analytic_ik_trace_elbow_cosine(
    const robot_model_t *model,
    const robot_transform_t *dh_transform,
    robot_real_t q1_rad,
    robot_real_t q6_rad,
    double *p13_x,
    double *p13_y,
    double *raw_cos_q3);

typedef struct
{
    robot_real_t q5_rad;
    robot_real_t q6_rad;
    int32_t q6_status;
    int32_t q3_status;
    double p13_x_mm;
    double p13_y_mm;
    double raw_cos_q3;  /* un-clamped: values >1 or < -1 matter */
    /*
     * Counterfactual: for the INPUT shoulder/wrist branch only,
     * substitute the known fixture J6 into the exact production p13
     * helper. This isolates error propagated by recovered q6.
     * Other branches are deliberately not compared to fixture J6.
     */
    uint32_t matches_input_branch;
    double input_q6_reference_rad;
    double q6_error_vs_input_rad;
    double p13_x_at_input_q6_mm;
    double p13_y_at_input_q6_mm;
    double cos_q3_at_input_q6;
    robot_real_t q3_rad[2];
} ik_trace_wrist_t;

typedef struct
{
    robot_real_t q1_rad;
    int32_t q5_status;
    ik_trace_wrist_t wrists[2];
} ik_trace_shoulder_t;

typedef struct
{
    uint32_t source_sample_index;
    robot_joint_angles_t input;
    int32_t fk_status;
    int32_t ik_status;
    int32_t dh_status;
    int32_t q1_status;
    uint32_t final_solution_count;
    ik_trace_shoulder_t shoulders[2];
} ik_trace_sample_t;

volatile ik_trace_sample_t g_ik_trace[6];
volatile uint32_t g_ik_trace_done = 0U;

static const uint32_t source_indices[6] = {8U,9U,10U,27U,28U,29U};
static const robot_joint_angle_t test_q5_raw[6] =
    {-1,0,1,-1,0,1};

__attribute__((noinline))
void qemu_ik_trace_done(void)
{
    for (;;)
        __asm volatile ("nop");
}

int main(void)
{
    const robot_model_t *model = robot_model_get_active();
    uint32_t sample;
    for (sample = 0U; sample < 6U; ++sample)
    {
        ik_trace_sample_t result = {0};
        robot_transform_t target = {{{0}}};
        robot_transform_t dh = {{{0}}};
        kinematics_ik_solutions_t solutions = {0};
        robot_real_t q1[2] = {0.0F,0.0F};
        uint32_t shoulder,wrist;

        result.source_sample_index=source_indices[sample];
        result.input.value[0]=3000;
        result.input.value[1]=-4500;
        result.input.value[2]=(sample<3U)?6000:0;
        result.input.value[3]=2000;
        result.input.value[4]=test_q5_raw[sample];
        result.input.value[5]=-1000;
        for(shoulder=0U;shoulder<2U;shoulder++)
        {
            result.shoulders[shoulder].q5_status =
                (int32_t)ROBOT_STATUS_ERROR_NOT_READY;
            for(wrist=0U;wrist<2U;wrist++)
            {
                result.shoulders[shoulder].wrists[wrist].q6_status =
                    (int32_t)ROBOT_STATUS_ERROR_NOT_READY;
                result.shoulders[shoulder].wrists[wrist].q3_status =
                    (int32_t)ROBOT_STATUS_ERROR_NOT_READY;
            }
        }
        result.fk_status = (int32_t)kinematics_forward(&result.input,&target);
        if(result.fk_status!=(int32_t)ROBOT_STATUS_OK)
            goto record;
        result.ik_status =
            (int32_t)kinematics_inverse(&target,&solutions);
        result.final_solution_count = solutions.count;
        result.dh_status = (int32_t)kinematics_public_to_dh_transform(
            model,&target,&dh);
        if(result.dh_status!=(int32_t)ROBOT_STATUS_OK)
            goto record;
        result.q1_status = (int32_t)ur5_analytic_ik_solve_q1_candidates(
            model,&dh,q1);
        if(result.q1_status!=(int32_t)ROBOT_STATUS_OK)
            goto record;

        for(shoulder=0U;shoulder<2U;shoulder++)
        {
            robot_real_t q5[2] = {0.0F,0.0F};
            robot_status_t status;
            result.shoulders[shoulder].q1_rad=q1[shoulder];
            status=ur5_analytic_ik_solve_q5_candidates(
                model,&dh,q1[shoulder],q5);
            result.shoulders[shoulder].q5_status=(int32_t)status;
            if(status!=ROBOT_STATUS_OK)
                continue;
            for(wrist=0U;wrist<2U;wrist++)
            {
                ik_trace_wrist_t *branch=&result.shoulders[shoulder].wrists[wrist];
                robot_real_t q6=0.0F;
                robot_real_t q3[2] = {0.0F,0.0F};
                double px=0.0,py=0.0,cos3=0.0;
                branch->q5_rad=q5[wrist];
                status=ur5_analytic_ik_solve_q6(
                    model,&dh,q1[shoulder],q5[wrist],&q6);
                branch->q6_status=(int32_t)status;
                if(status!=ROBOT_STATUS_OK)
                    continue;
                branch->q6_rad=q6;
                status=ur5_analytic_ik_trace_elbow_cosine(
                    model,&dh,q1[shoulder],q6,&px,&py,&cos3);
                if(status!=ROBOT_STATUS_OK) {
                    branch->q3_status=(int32_t)status;
                    continue;
                }
                branch->p13_x_mm=px;
                branch->p13_y_mm=py;
                branch->raw_cos_q3=cos3;
                /*
                 * All six samples have the same input J1=+30 degrees
                 * and J6=-10 degrees. Match shoulder and the true
                 * wrist-sign branch, rather than comparing a distinct
                 * valid shoulder/wrist IK solution to this fixture.
                 */
                if (sample != 4U &&
                    fabs((double)q1[shoulder] -
                         (double)result.input.value[0] *
                         (double)ROBOT_JOINT_ANGLE_UNIT_DEG *
                         ((double)KINEMATICS_PI_F / 180.0)) < 0.01 &&
                    ((result.input.value[4] < 0 && q5[wrist] < 0.0F) ||
                     (result.input.value[4] > 0 && q5[wrist] > 0.0F)))
                {
                    double rx=0.0,ry=0.0,rc=0.0;
                    double expected_q6 =
                        (double)result.input.value[5] *
                        (double)ROBOT_JOINT_ANGLE_UNIT_DEG *
                        ((double)KINEMATICS_PI_F / 180.0);
                    robot_status_t ref_status =
                        ur5_analytic_ik_trace_elbow_cosine(
                            model,&dh,q1[shoulder],
                            (robot_real_t)expected_q6,&rx,&ry,&rc);
                    if (ref_status == ROBOT_STATUS_OK)
                    {
                        branch->matches_input_branch=1U;
                        branch->input_q6_reference_rad=expected_q6;
                        branch->q6_error_vs_input_rad=(double)q6-expected_q6;
                        branch->p13_x_at_input_q6_mm=rx;
                        branch->p13_y_at_input_q6_mm=ry;
                        branch->cos_q3_at_input_q6=rc;
                    }
                }
                status=ur5_analytic_ik_solve_q3_candidates(
                    model,&dh,q1[shoulder],q5[wrist],q6,q3);
                branch->q3_status=(int32_t)status;
                if(status==ROBOT_STATUS_OK) {
                    branch->q3_rad[0]=q3[0];
                    branch->q3_rad[1]=q3[1];
                }
            }
        }
record:
        g_ik_trace[sample]=result;
    }
    g_ik_trace_done=1U;
    qemu_ik_trace_done();
    return 0;
}
