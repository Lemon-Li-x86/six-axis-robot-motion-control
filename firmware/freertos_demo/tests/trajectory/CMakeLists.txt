cmake_minimum_required(VERSION 3.20)
project(robot_stage4_trajectory_host LANGUAGES C)

# Fully isolated: no edits to existing stage-3 host targets or ARM targets.
set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

add_executable(trajectory_host_test
    trajectory_test_main.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/polynomial_trajectory.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/trapezoidal_profile.c
)

target_include_directories(trajectory_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/algorithms/trajectory
)

set_target_properties(trajectory_host_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(trajectory_host_test PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(trajectory_host_test PRIVATE m)
endif()

enable_testing()
add_test(NAME stage4_trajectory_host COMMAND trajectory_host_test)
