/*
 * 文件：test_motor_driver.c
 *
 * 用途：
 * 使用 Unity 验证 Motor Driver 的：
 *
 * 1. 初始化状态；
 * 2. Canonical Angle 规范化；
 * 3. 首次 Feedback；
 * 4. ±180° Wrap Correction；
 * 5. Continuous Position；
 * 6. Velocity；
 * 7. Null Pointer 错误处理。
 */

#include <stdint.h>

#include "unity.h"

#include "motor_driver.h"
#include "robot_types.h"
#include "error_code.h"


/* =========================================================
 * Initialization Tests
 * ========================================================= */

/**
 * @brief 验证 Motor Driver 初始化后
 *        六轴目标位置为 0 且尚无 Feedback。
 */
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


/* =========================================================
 * Target Position Tests
 * ========================================================= */

/**
 * @brief 验证目标关节角被规范化到
 *        [-180°, 180°)。
 */
void test_motor_target_normalization(void)
{
    /*
     * 单位：
     * 0.01 degree。
     *
     * 重点覆盖：
     *
     * +180° -> -180°
     * +190° -> -170°
     * -190° -> +170°
     */
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


/* =========================================================
 * Feedback Tests
 * ========================================================= */

/**
 * @brief 验证第一组 Feedback 建立位置状态，
 *        但速度保持为 0。
 */
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

    /*
     * delta_time_s = 0.1 s。
     *
     * 由于这是第一组 Feedback，
     * 尚不存在上一组位置用于计算速度，
     * 所以所有速度应为 0。
     */
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


/* =========================================================
 * Angle Wrap Tests
 * ========================================================= */

/**
 * @brief 验证 +179° -> -179°
 *        被识别为正向 +2° 连续运动。
 */
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

    /*
     * 179° -> -179°
     *
     * Canonical Angle 看起来跳变 -358°，
     * 实际最短运动为 +2°。
     *
     * 0.1 s 内移动 +2°：
     *
     * velocity = +20 degree / second。
     */
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


/**
 * @brief 验证 -179° -> +179°
 *        被识别为反向 -2° 连续运动。
 */
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

    /*
     * -179° -> +179°
     *
     * 实际最短运动为 -2°。
     *
     * 0.1 s 内移动 -2°：
     *
     * velocity = -20 degree / second。
     */
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


/* =========================================================
 * Error Handling Tests
 * ========================================================= */

/**
 * @brief 验证所有主要 Motor Driver API
 *        对 NULL Pointer 返回统一错误码。
 */
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