cmake_minimum_required(VERSION 3.20)
project(robot_stage5_avoidance_host LANGUAGES C)
enable_testing()

set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

add_executable(stage5_potential_field_host
    potential_field_test_main.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field.c
)
target_include_directories(stage5_potential_field_host PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/algorithms/avoidance
)
set_target_properties(stage5_potential_field_host PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage5_potential_field_host PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(stage5_potential_field_host PRIVATE m)
endif()
add_test(NAME stage5_potential_field_host COMMAND stage5_potential_field_host)

# Host-only C boundary for PyBullet obstacle probes. No second APF
# implementation in Python and no shared library in the ARM firmware.
add_library(stage5_apf_bridge SHARED
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field_path.c
)
target_include_directories(stage5_apf_bridge PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/algorithms/avoidance
)
set_target_properties(stage5_apf_bridge PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage5_apf_bridge PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(stage5_apf_bridge PRIVATE m)
endif()

# Stage5 bounded multi-step path uses the same C APF primitive as PyBullet.
add_executable(stage5_potential_field_path_host
    potential_field_path_test_main.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field_path.c
)
target_include_directories(stage5_potential_field_path_host PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/algorithms/avoidance
)
set_target_properties(stage5_potential_field_path_host PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage5_potential_field_path_host PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(stage5_potential_field_path_host PRIVATE m)
endif()
add_test(NAME stage5_potential_field_path_host
    COMMAND stage5_potential_field_path_host)

# Stage5: actual production IK + selector + FK and trapezoid segment
# conversion for C APF waypoints. No PyBullet or FreeRTOS in this test.
add_executable(stage5_apf_joint_route_host
    stage5_apf_joint_path_test_main.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/potential_field_path.c
    ${FIRMWARE_ROOT}/algorithms/avoidance/stage5_apf_joint_path.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/kinematics_common.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/kinematics_forward.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/ur5_analytic_ik.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/kinematics_inverse.c
    ${FIRMWARE_ROOT}/algorithms/motion/ik_solution_selector.c
    ${FIRMWARE_ROOT}/algorithms/motion/motion_limits.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/trapezoidal_profile.c
    ${FIRMWARE_ROOT}/algorithms/math/matrix4.c
    ${FIRMWARE_ROOT}/common/angle_utils.c
    ${FIRMWARE_ROOT}/model/ur5_model.c
)
target_include_directories(stage5_apf_joint_route_host PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/model
    ${FIRMWARE_ROOT}/algorithms/avoidance
    ${FIRMWARE_ROOT}/algorithms/kinematics
    ${FIRMWARE_ROOT}/algorithms/motion
    ${FIRMWARE_ROOT}/algorithms/trajectory
    ${FIRMWARE_ROOT}/algorithms/math
)
set_target_properties(stage5_apf_joint_route_host PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage5_apf_joint_route_host PRIVATE
        -Wall -Wextra -Wpedantic -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(stage5_apf_joint_route_host PRIVATE m)
endif()
add_test(NAME stage5_apf_joint_route_host
    COMMAND stage5_apf_joint_route_host)
