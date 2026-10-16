cmake_minimum_required(VERSION 3.20)
project(robot_motion_host_tests LANGUAGES C)

# Native compiler build: no ARM toolchain, QEMU, FreeRTOS or Unity required.
# Reuse the exact same boundary-test source as the Cortex-M4 QEMU target.
set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

add_executable(motion_limits_host_test
    ../motion/motion_limits_test_main.c
    ${FIRMWARE_ROOT}/algorithms/motion/motion_limits.c
    ${FIRMWARE_ROOT}/model/ur5_model.c
)

target_include_directories(motion_limits_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/model
    ${FIRMWARE_ROOT}/algorithms/motion
)

target_compile_definitions(motion_limits_host_test PRIVATE
    MOTION_LIMITS_HOST_TEST=1
)

set_target_properties(motion_limits_host_test PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motion_limits_host_test PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

enable_testing()
add_test(NAME motion_limits_host COMMAND motion_limits_host_test)

# Run the production motor driver against the existing 11 QEMU boundary cases.
# The host macro only changes the test runner's exit path.
add_executable(motor_driver_host_test
    ../qemu_target/motor_test_main.c
    ${FIRMWARE_ROOT}/drivers/motor_driver.c
)

target_include_directories(motor_driver_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/drivers
)

target_compile_definitions(motor_driver_host_test PRIVATE
    MOTOR_DRIVER_HOST_TEST=1
)

set_target_properties(motor_driver_host_test PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(motor_driver_host_test PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

add_test(NAME motor_driver_host COMMAND motor_driver_host_test)
