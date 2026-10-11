
# ==========================================================
# 文件：firmware/freertos_demo/CMakeLists.txt
#
# 用途：
# 构建 Cortex-M4 / FreeRTOS 固件。
#
# 主要职责：
# 1. 查找 ARM 固件依赖；
# 2. 从 JSON 生成固件默认参数头文件；
# 3. 编译 Driver、Protocol、Model、Kinematics；
# 4. 编译 FreeRTOS Kernel；
# 5. 链接 Cortex-M4 ELF 固件。
#
# 注意：
# JSON 文件只在 Host 构建阶段使用。
# Cortex-M4 固件不需要 Python 或 JSON 解析器。
# ==========================================================


# ==========================================================
# 1. Dependencies
# ==========================================================

# FreeRTOS Kernel 所在目录。
set(
    FREERTOS_DIR
    "${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel"
)

# Python 只运行在开发计算机上，
# 用于生成编译时配置。
find_package(
    Python3
    REQUIRED
    COMPONENTS Interpreter
)


# ==========================================================
# 2. Firmware Configuration Generation
# ==========================================================

# JSON 默认参数文件。
set(
    FIRMWARE_CONFIG_JSON
    "${CMAKE_CURRENT_SOURCE_DIR}/config/robot_defaults.json"
)

# Python 配置生成脚本。
set(
    FIRMWARE_CONFIG_GENERATOR
    "${CMAKE_SOURCE_DIR}/tools/generate_firmware_defaults.py"
)

# 所有自动生成的头文件统一放在 Build 目录。
#
# 不在源代码目录写入自动生成结果，
# 避免生成文件混入 Git。
set(
    FIRMWARE_GENERATED_INCLUDE_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

set(
    FIRMWARE_GENERATED_HEADER
    "${FIRMWARE_GENERATED_INCLUDE_DIR}/firmware_defaults_generated.h"
)

# 自定义生成规则。
#
# OUTPUT:
#   生成的 .h 文件。
#
# COMMAND:
#   运行 Python 脚本。
#
# DEPENDS:
#   JSON 或脚本变化时，重新生成头文件。
#
# VERBATIM:
#   让 CMake 正确处理参数与路径中的特殊字符。
add_custom_command(
    OUTPUT
        "${FIRMWARE_GENERATED_HEADER}"

    COMMAND
        "${Python3_EXECUTABLE}"
        "${FIRMWARE_CONFIG_GENERATOR}"
        --input
        "${FIRMWARE_CONFIG_JSON}"
        --output
        "${FIRMWARE_GENERATED_HEADER}"

    DEPENDS
        "${FIRMWARE_CONFIG_JSON}"
        "${FIRMWARE_CONFIG_GENERATOR}"

    COMMENT
        "Validating JSON and generating firmware defaults"

    VERBATIM
)

# 独立配置生成目标。
#
# 可以单独执行：
# cmake --build build/cmake-arm
#       --target firmware_defaults_header
add_custom_target(
    firmware_defaults_header

    DEPENDS
        "${FIRMWARE_GENERATED_HEADER}"
)


# ==========================================================
# 3. Firmware Source Files
# ==========================================================

add_executable(
    freertos_demo

    # ------------------------------------------------------
    # System / BSP
    # ------------------------------------------------------

    startup.s
    main.c
    board.c
    memory.c


    # ------------------------------------------------------
    # Robot Model
    # ------------------------------------------------------

    model/ur5_model.c


    # ------------------------------------------------------
    # Driver Layer
    # ------------------------------------------------------

    drivers/uart_driver.c
    drivers/motor_driver.c
    drivers/timer_driver.c
    drivers/gpio_driver.c


    # ------------------------------------------------------
    # Communication Layer
    # ------------------------------------------------------

    communications/protocol.c
    communications/ring_buffer.c


    # ------------------------------------------------------
    # Application Layer
    # ------------------------------------------------------

    application/uart_tx_manager.c
    application/protocol_tasks.c
    application/runtime_config.c


    # ------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------

    diagnostics/performance_monitor.c


    # ------------------------------------------------------
    # Algorithm / Math
    # ------------------------------------------------------

    algorithms/math/matrix4.c


    # ------------------------------------------------------
    # Algorithm / Kinematics
    # ------------------------------------------------------

    algorithms/kinematics/kinematics_common.c
    algorithms/kinematics/kinematics_forward.c
    algorithms/kinematics/ur5_analytic_ik.c
    algorithms/kinematics/kinematics_inverse.c


    # ------------------------------------------------------
    # Tests / Kinematics
    # ------------------------------------------------------

    tests/kinematics/kinematics_test_support.c
    tests/kinematics/kinematics_fk_test.c
    tests/kinematics/kinematics_ik_solver_test.c
    tests/kinematics/kinematics_ik_test.c
    tests/kinematics/kinematics_self_test.c


    # ------------------------------------------------------
    # FreeRTOS Kernel
    # ------------------------------------------------------

    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c


    # ------------------------------------------------------
    # Cortex-M FreeRTOS Port
    # ------------------------------------------------------

    ${FREERTOS_DIR}/portable/GCC/ARM_CM3/port.c


    # ------------------------------------------------------
    # FreeRTOS Memory Management
    # ------------------------------------------------------

    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)


# ==========================================================
# 4. Build Dependency
# ==========================================================

# 必须先完成配置生成，
# 然后才允许编译固件。
#
# 避免并行构建时出现：
# 编译器尝试读取尚未生成的头文件。
add_dependencies(
    freertos_demo
    firmware_defaults_header
)


# ==========================================================
# 5. Include Paths
# ==========================================================

target_include_directories(
    freertos_demo

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}

    # 自动生成的配置头文件。
    "${FIRMWARE_GENERATED_INCLUDE_DIR}"

    # Common
    ${CMAKE_CURRENT_SOURCE_DIR}/common

    # Robot Model
    ${CMAKE_CURRENT_SOURCE_DIR}/model

    # Drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers

    # Communications
    ${CMAKE_CURRENT_SOURCE_DIR}/communications

    # Application
    ${CMAKE_CURRENT_SOURCE_DIR}/application

    # Diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics

    # Algorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics

    # Tests
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/kinematics

    # FreeRTOS
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM3
)


# ==========================================================
# 6. Compile Options
# ==========================================================

target_compile_options(
    freertos_demo

    PRIVATE

    -mcpu=cortex-m4
    -mthumb

    -ffreestanding
    -fno-builtin
)


# ==========================================================
# 7. Link Options
# ==========================================================

target_link_options(
    freertos_demo

    PRIVATE

    -mcpu=cortex-m4
    -mthumb

    -nostdlib

    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
)


# ==========================================================
# 8. Libraries
# ==========================================================

target_link_libraries(
    freertos_demo

    PRIVATE

    m
    gcc
)


# ==========================================================
# 9. Firmware Output
# ==========================================================

set_target_properties(
    freertos_demo

    PROPERTIES

    SUFFIX ".elf"
)
