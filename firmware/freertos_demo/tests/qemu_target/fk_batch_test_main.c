
#include <stdint.h>

#include "kinematics.h"
#include "robot_types.h"
#include "error_code.h"

#define FK_SAMPLE_COUNT 128U

/*
 * 固定二进制格式：
 * 6 x int16_t = 12 bytes
 * 16 x float  = 64 bytes
 * 1 x int32_t =  4 bytes
 * Total       = 80 bytes
 */
typedef struct
{
    int16_t joints_raw[6];
    float transform[16];
    int32_t status;
} fk_sample_t;

_Static_assert(sizeof(fk_sample_t) == 80U,
               "Unexpected FK sample layout");

volatile fk_sample_t g_fk_samples[FK_SAMPLE_COUNT];
volatile uint32_t g_fk_finished = 0U;

/* 可重复的伪随机整数序列 */
static uint32_t next_random(uint32_t *state)
{
    *state = 1664525UL * (*state) + 1013904223UL;
    return *state;
}

__attribute__((noinline))
void qemu_fk_batch_done(void)
{
    for (;;)
    {
        __asm volatile ("nop");
    }
}

int main(void)
{
    uint32_t state = 20261012UL;

    for (uint32_t i = 0; i < FK_SAMPLE_COUNT; i++)
    {
        robot_joint_angles_t joints = {{0}};
        robot_transform_t actual = {{{0}}};

        /*
         * 前三组为已有基准姿态。
         * 第四、第五组为接近 Wrist 奇异位姿。
         * 其余为确定性伪随机测试姿态。
         */
        if (i == 1U)
        {
            joints.value[0] = 6000;
        }
        else if (i == 2U)
        {
            joints.value[1] = -9000;
        }
        else if (i == 3U)
        {
            joints.value[4] = 1;
        }
        else if (i == 4U)
        {
            joints.value[4] = -1;
        }
        else if (i >= 5U)
        {
            for (uint32_t j = 0; j < 6U; j++)
            {
                uint32_t value = next_random(&state);

                joints.value[j] =
                    (int16_t)((int32_t)(value % 32001U)
                              - 16000);
            }
        }

        robot_status_t status =
            kinematics_forward(&joints, &actual);

        for (uint32_t j = 0; j < 6U; j++)
        {
            g_fk_samples[i].joints_raw[j] = joints.value[j];
        }

        g_fk_samples[i].status = (int32_t)status;

        for (uint32_t r = 0; r < 4U; r++)
        {
            for (uint32_t c = 0; c < 4U; c++)
            {
                g_fk_samples[i].transform[r * 4U + c] =
                    actual.matrix[r][c];
            }
        }
    }

    g_fk_finished = 1U;
    qemu_fk_batch_done();

    return 0;
}
