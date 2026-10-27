
cmake_minimum_required(VERSION 3.20)
project(robot_motion_host_tests LANGUAGES C)

# Native compiler build:
# no ARM toolchain, QEMU, FreeRTOS or Unity required.
set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")


# ==========================================================
# 1. Motion limits
# ==========================================================

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

add_test(
    NAME motion_limits_host
    COMMAND motion_limits_host_test
)


# ==========================================================
# 2. Motor driver boundary
# ==========================================================

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

add_test(
    NAME motor_driver_host
    COMMAND motor_driver_host_test
)


# ==========================================================
# 3. Motor feedback safety
# ==========================================================

add_executable(motor_feedback_safety_host_test
    motor_feedback_safety_test.c
    ${FIRMWARE_ROOT}/drivers/motor_driver.c
)

target_include_directories(
    motor_feedback_safety_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/drivers
)

set_target_properties(
    motor_feedback_safety_host_test PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(
        motor_feedback_safety_host_test PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

add_test(
    NAME motor_feedback_safety_host
    COMMAND motor_feedback_safety_host_test
)


# ==========================================================
# 4. IK solution selector
# ==========================================================

add_executable(ik_selector_host_test
    ../motion/ik_solution_selector_test_main.c
    ${FIRMWARE_ROOT}/algorithms/motion/ik_solution_selector.c
    ${FIRMWARE_ROOT}/algorithms/motion/motion_limits.c
    ${FIRMWARE_ROOT}/model/ur5_model.c
)

target_include_directories(ik_selector_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/model
    ${FIRMWARE_ROOT}/algorithms/motion
    ${FIRMWARE_ROOT}/algorithms/kinematics
)

set_target_properties(ik_selector_host_test PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

# GCC/Clang may need libm for sinf/fabsf.
# MSVC links the C runtime automatically.
if(NOT MSVC)
    target_link_libraries(ik_selector_host_test PRIVATE m)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ik_selector_host_test PRIVATE
        -Wall -Wextra -Wpedantic
    )
endif()

add_test(
    NAME ik_selector_host
    COMMAND ik_selector_host_test
)

# Stage2 motor implementation now references the Common normalizer.
target_sources(motor_driver_host_test PRIVATE
    ${FIRMWARE_ROOT}/common/angle_utils.c)
target_sources(motor_feedback_safety_host_test PRIVATE
    ${FIRMWARE_ROOT}/common/angle_utils.c)

# Stage5: production C wire parser resilience under partial/corrupt input.
add_executable(protocol_stream_fault_host_test
    protocol_stream_fault_test.c
    ${FIRMWARE_ROOT}/communications/protocol.c
)
target_include_directories(protocol_stream_fault_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/communications
)
set_target_properties(protocol_stream_fault_host_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(protocol_stream_fault_host_test PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
add_test(NAME protocol_stream_fault_host COMMAND protocol_stream_fault_host_test)
