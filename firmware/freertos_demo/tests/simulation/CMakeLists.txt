cmake_minimum_required(VERSION 3.20)
project(stage4_native_pybullet_bridge LANGUAGES C)

# Single production source for model, controller, Stage4 defaults and limits.
# This is a host-only shared library, NOT added to the ARM FreeRTOS target.
set(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(GENERATED_HEADER "${GENERATED_DIR}/firmware_defaults_generated.h")
add_custom_command(OUTPUT "${GENERATED_HEADER}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${GENERATED_DIR}"
    COMMAND "${Python3_EXECUTABLE}"
       "${FIRMWARE_ROOT}/../../tools/generate_firmware_defaults.py"
       --input "${FIRMWARE_ROOT}/config/robot_defaults.json"
       --output "${GENERATED_HEADER}"
    DEPENDS
       "${FIRMWARE_ROOT}/../../tools/generate_firmware_defaults.py"
       "${FIRMWARE_ROOT}/config/robot_defaults.json"
    VERBATIM
)
add_custom_target(stage4_sim_generated_config DEPENDS "${GENERATED_HEADER}")

add_library(stage4_sim_bridge SHARED
    stage4_sim_bridge.c
    "${FIRMWARE_ROOT}/config/stage4_motion_config.c"
    "${FIRMWARE_ROOT}/algorithms/trajectory/trapezoidal_profile.c"
    "${FIRMWARE_ROOT}/algorithms/control/pid.c"
    "${FIRMWARE_ROOT}/algorithms/control/joint_controller.c"
    "${FIRMWARE_ROOT}/algorithms/control/joint_command_limiter.c"
    "${FIRMWARE_ROOT}/algorithms/motion/motion_limits.c"
    "${FIRMWARE_ROOT}/model/ur5_model.c"
)
add_dependencies(stage4_sim_bridge stage4_sim_generated_config)
target_include_directories(stage4_sim_bridge PRIVATE
    "${GENERATED_DIR}"
    "${FIRMWARE_ROOT}/common"
    "${FIRMWARE_ROOT}/application"
    "${FIRMWARE_ROOT}/application/services"
    "${FIRMWARE_ROOT}/config"
    "${FIRMWARE_ROOT}/algorithms/trajectory"
    "${FIRMWARE_ROOT}/algorithms/control"
    "${FIRMWARE_ROOT}/algorithms/motion"
    "${FIRMWARE_ROOT}/algorithms/kinematics"
    "${FIRMWARE_ROOT}/model"
)
set_target_properties(stage4_sim_bridge PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stage4_sim_bridge PRIVATE
       -Wall -Wextra -Wpedantic -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(stage4_sim_bridge PRIVATE m)
endif()
