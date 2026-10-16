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
