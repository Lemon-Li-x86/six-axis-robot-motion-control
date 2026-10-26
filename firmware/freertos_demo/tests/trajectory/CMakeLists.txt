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

# Application adapter: reuses the authoritative Robot Model and Motion Limits.
add_executable(trajectory_adapter_host_test
    trajectory_adapter_test_main.c
    ${FIRMWARE_ROOT}/application/services/trajectory_request_adapter.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/trapezoidal_profile.c
    ${FIRMWARE_ROOT}/algorithms/motion/motion_limits.c
    ${FIRMWARE_ROOT}/model/ur5_model.c
)
target_include_directories(trajectory_adapter_host_test PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/application/services
    ${FIRMWARE_ROOT}/algorithms/trajectory
    ${FIRMWARE_ROOT}/algorithms/kinematics
    ${FIRMWARE_ROOT}/algorithms/motion
    ${FIRMWARE_ROOT}/model
)
set_target_properties(trajectory_adapter_host_test PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(trajectory_adapter_host_test PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(trajectory_adapter_host_test PRIVATE m)
endif()
add_test(NAME stage4_trajectory_adapter_host COMMAND trajectory_adapter_host_test)

# Cartesian geometry/SO(3) and incremental PID, no ARM/RTOS dependencies.
add_executable(stage4_cartesian_pid_host
    cartesian_control_test_main.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_common.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_line.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_arc.c
    ${FIRMWARE_ROOT}/algorithms/control/pid.c
    ${FIRMWARE_ROOT}/algorithms/control/joint_controller.c
)
target_include_directories(stage4_cartesian_pid_host PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/algorithms/trajectory
    ${FIRMWARE_ROOT}/algorithms/control
)
set_target_properties(stage4_cartesian_pid_host PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage4_cartesian_pid_host PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(stage4_cartesian_pid_host PRIVATE m)
endif()
add_test(NAME stage4_cartesian_pid_host COMMAND stage4_cartesian_pid_host)

# Full production algorithm chain, without queue/driver implementation:
# Cartesian sample -> IK -> selector -> continuous joint target -> FK.
add_executable(stage4_cartesian_ik_host
    cartesian_ik_test_main.c
    ${FIRMWARE_ROOT}/common/angle_utils.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_common.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_line.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_arc.c
    ${FIRMWARE_ROOT}/algorithms/trajectory/cartesian_waypoint.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/kinematics_common.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/kinematics_forward.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/kinematics_inverse.c
    ${FIRMWARE_ROOT}/algorithms/kinematics/ur5_analytic_ik.c
    ${FIRMWARE_ROOT}/algorithms/math/matrix4.c
    ${FIRMWARE_ROOT}/algorithms/motion/motion_limits.c
    ${FIRMWARE_ROOT}/algorithms/motion/ik_solution_selector.c
    ${FIRMWARE_ROOT}/model/ur5_model.c
)
target_include_directories(stage4_cartesian_ik_host PRIVATE
    ${FIRMWARE_ROOT}/common
    ${FIRMWARE_ROOT}/model
    ${FIRMWARE_ROOT}/algorithms/math
    ${FIRMWARE_ROOT}/algorithms/kinematics
    ${FIRMWARE_ROOT}/algorithms/motion
    ${FIRMWARE_ROOT}/algorithms/trajectory
)
set_target_properties(stage4_cartesian_ik_host PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage4_cartesian_ik_host PRIVATE
        -Wall -Wextra -Wpedantic -Werror
    )
endif()
if(NOT MSVC)
    target_link_libraries(stage4_cartesian_ik_host PRIVATE m)
endif()
add_test(NAME stage4_cartesian_ik_host COMMAND stage4_cartesian_ik_host)
