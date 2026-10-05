#include <stdint.h>

#include "unity.h"

#include "motor_driver.h"
#include "robot_types.h"
#include "error_code.h"


void test_motor_init_sets_zero_targets(void)
{
    robot_joint_angles_t targets;
    uint32_t i;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_init()
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_target_positions(
            &targets
        )
    );

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        TEST_ASSERT_EQUAL_INT16(
            0,
            targets.value[i]
        );
    }

    TEST_ASSERT_EQUAL_UINT8(
        0U,
        motor_driver_has_feedback()
    );
}


void test_motor_target_normalization(void)
{
    robot_joint_angles_t input =
    {
        .value =
        {
            18000,
            19000,
            -18000,
            -19000,
            17999,
            -17999
        }
    };

    robot_joint_angles_t output;

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_init()
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_set_target_positions(
            &input
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_target_positions(
            &output
        )
    );

    TEST_ASSERT_EQUAL_INT16(
        -18000,
        output.value[0]
    );

    TEST_ASSERT_EQUAL_INT16(
        -17000,
        output.value[1]
    );

    TEST_ASSERT_EQUAL_INT16(
        -18000,
        output.value[2]
    );

    TEST_ASSERT_EQUAL_INT16(
        17000,
        output.value[3]
    );

    TEST_ASSERT_EQUAL_INT16(
        17999,
        output.value[4]
    );

    TEST_ASSERT_EQUAL_INT16(
        -17999,
        output.value[5]
    );
}


void test_motor_first_feedback_has_zero_velocity(void)
{
    robot_joint_angles_t feedback =
    {
        .value =
        {
            1000,
            2000,
            3000,
            4000,
            5000,
            6000
        }
    };

    robot_joint_angles_t positions;
    robot_joint_positions_t unwrapped;
    robot_joint_velocities_t velocities;

    uint32_t i;

    motor_driver_init();

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_update_feedback(
            &feedback,
            0.1F
        )
    );

    TEST_ASSERT_EQUAL_UINT8(
        1U,
        motor_driver_has_feedback()
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_positions(
            &positions
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_unwrapped_positions(
            &unwrapped
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_velocities(
            &velocities
        )
    );

    for (i = 0U; i < ROBOT_JOINT_COUNT; i++)
    {
        TEST_ASSERT_EQUAL_INT16(
            feedback.value[i],
            positions.value[i]
        );

        TEST_ASSERT_EQUAL_INT32(
            feedback.value[i],
            unwrapped.value[i]
        );

        TEST_ASSERT_FLOAT_WITHIN(
            0.0001F,
            0.0F,
            velocities.value[i]
        );
    }
}


void test_motor_wrap_forward(void)
{
    robot_joint_angles_t first =
    {
        .value =
        {
            17900,
            0,
            0,
            0,
            0,
            0
        }
    };

    robot_joint_angles_t second =
    {
        .value =
        {
            -17900,
            0,
            0,
            0,
            0,
            0
        }
    };

    robot_joint_positions_t positions;
    robot_joint_velocities_t velocities;

    motor_driver_init();

    motor_driver_update_feedback(
        &first,
        0.1F
    );

    motor_driver_update_feedback(
        &second,
        0.1F
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_unwrapped_positions(
            &positions
        )
    );

    TEST_ASSERT_EQUAL_INT32(
        18100,
        positions.value[0]
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_OK,
        motor_driver_get_velocities(
            &velocities
        )
    );

    TEST_ASSERT_FLOAT_WITHIN(
        0.01F,
        20.0F,
        velocities.value[0]
    );
}


void test_motor_wrap_backward(void)
{
    robot_joint_angles_t first =
    {
        .value =
        {
            -17900,
            0,
            0,
            0,
            0,
            0
        }
    };

    robot_joint_angles_t second =
    {
        .value =
        {
            17900,
            0,
            0,
            0,
            0,
            0
        }
    };

    robot_joint_positions_t positions;
    robot_joint_velocities_t velocities;

    motor_driver_init();

    motor_driver_update_feedback(
        &first,
        0.1F
    );

    motor_driver_update_feedback(
        &second,
        0.1F
    );

    motor_driver_get_unwrapped_positions(
        &positions
    );

    motor_driver_get_velocities(
        &velocities
    );

    TEST_ASSERT_EQUAL_INT32(
        -18100,
        positions.value[0]
    );

    TEST_ASSERT_FLOAT_WITHIN(
        0.01F,
        -20.0F,
        velocities.value[0]
    );
}


void test_motor_null_pointer_errors(void)
{
    motor_driver_init();

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        motor_driver_set_target_positions(
            NULL
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        motor_driver_get_target_positions(
            NULL
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        motor_driver_update_feedback(
            NULL,
            0.1F
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        motor_driver_get_positions(
            NULL
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        motor_driver_get_unwrapped_positions(
            NULL
        )
    );

    TEST_ASSERT_EQUAL_INT(
        ROBOT_STATUS_ERROR_NULL_POINTER,
        motor_driver_get_velocities(
            NULL
        )
    );
}