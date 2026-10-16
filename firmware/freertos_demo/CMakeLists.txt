
# ==========================================================
# 文件：firmware/freertos_demo/CMakeLists.txt
#
# 用途：
# 构建 Cortex-M4 / FreeRTOS 固件和独立 QEMU 测试固件。
#
# 主要职责：
# 1. 查找构建依赖，生成固件默认参数头文件；
# 2. 定义 FreeRTOS 主固件的源码和编译参数；
# 3. 定义 QEMU 独立测试目标；
# 4. 统一 Cortex-M4 测试目标的公共编译和链接参数。
#
# 注意：
# JSON 仅在 Host 构建阶段解析。
# QEMU 测试目标使用独立 main()，不得加入 freertos_demo。
# ==========================================================


# ==========================================================
# 1. Dependencies
# ==========================================================

# FreeRTOS Kernel 所在目录。
set(
    FREERTOS_DIR
    "${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel"
)

# Python 仅在 Host 上生成配置头文件。
find_package(
    Python3
    REQUIRED
    COMPONENTS Interpreter
)


# ==========================================================
# 2. Firmware Configuration Generation
# ==========================================================

set(
    FIRMWARE_CONFIG_JSON
    "${CMAKE_CURRENT_SOURCE_DIR}/config/robot_defaults.json"
)

set(
    FIRMWARE_CONFIG_GENERATOR
    "${CMAKE_SOURCE_DIR}/tools/generate_firmware_defaults.py"
)

# 生成文件放在构建目录，避免写入源码仓库。
set(
    FIRMWARE_GENERATED_INCLUDE_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

set(
    FIRMWARE_GENERATED_HEADER
    "${FIRMWARE_GENERATED_INCLUDE_DIR}/firmware_defaults_generated.h"
)

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

add_custom_target(
    firmware_defaults_header

    DEPENDS
        "${FIRMWARE_GENERATED_HEADER}"
)


# ==========================================================
# 3. FreeRTOS Firmware Sources
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
    # 当前阶段通过主固件的 Self-Test 入口执行。
    # 后续生产版本可再将 Self-Test 独立为构建选项。
    tests/kinematics/kinematics_test_support.c
    tests/kinematics/kinematics_fk_test.c
    tests/kinematics/kinematics_ik_solver_test.c
    tests/kinematics/kinematics_ik_test.c
    tests/kinematics/kinematics_self_test.c
    tests/kinematics/kinematics_dedup_test.c

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
# 4. FreeRTOS Firmware Build Settings
# ==========================================================

# 参数生成必须先于固件编译完成。
add_dependencies(
    freertos_demo
    firmware_defaults_header
)

target_include_directories(
    freertos_demo

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    "${FIRMWARE_GENERATED_INCLUDE_DIR}"

    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/communications
    ${CMAKE_CURRENT_SOURCE_DIR}/application
    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/kinematics

    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM3
)

target_compile_options(
    freertos_demo

    PRIVATE

    -mcpu=cortex-m4
    -mthumb
    -ffreestanding
    -fno-builtin
)

target_link_options(
    freertos_demo

    PRIVATE

    -mcpu=cortex-m4
    -mthumb
    -nostdlib
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
)

target_link_libraries(
    freertos_demo

    PRIVATE

    m
    gcc
)

set_target_properties(
    freertos_demo

    PROPERTIES

    SUFFIX ".elf"
)


# ==========================================================
# 5. QEMU Test Common Settings
# ==========================================================

# @brief 为已有 QEMU 测试目标配置公共编译和链接选项。
#
# 使用方式：
# configure_qemu_target(qemu_xxx_tests)
# configure_qemu_target(qemu_xxx_batch m)
#
# 注意：
# 测试入口和生产源码在 add_executable() 中单独声明；
# 本函数不添加 main()，不修改源码归属。
# ARGN 为目标额外需要的链接库，例如 m。
function(configure_qemu_target target_name)
    target_compile_options(
        ${target_name}

        PRIVATE

        -mcpu=cortex-m4
        -mthumb
        -ffreestanding
        -fno-builtin
        -O0
        -g3
    )

    target_link_options(
        ${target_name}

        PRIVATE

        -mcpu=cortex-m4
        -mthumb
        -nostdlib
        "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
    )

    # m 必须排在 gcc 之前，保持原有库链接顺序。
    target_link_libraries(
        ${target_name}

        PRIVATE

        ${ARGN}
        gcc
    )

    set_target_properties(
        ${target_name}

        PROPERTIES

        SUFFIX ".elf"
    )
endfunction()


# ==========================================================
# 6. QEMU Protocol / Ring Buffer Tests
# ==========================================================

add_executable(
    qemu_protocol_tests

    tests/qemu_target/test_startup.s
    tests/qemu_target/protocol_test_main.c
    communications/protocol.c
    communications/ring_buffer.c
    memory.c
)

target_include_directories(
    qemu_protocol_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/communications
)

configure_qemu_target(qemu_protocol_tests)


# ==========================================================
# 7. QEMU UART TX Boundary Tests
# ==========================================================

add_executable(
    qemu_uart_tx_tests

    tests/qemu_target/test_startup.s
    tests/qemu_target/uart_tx_boundary_main.c
    drivers/uart_driver.c
    communications/ring_buffer.c
    memory.c
)

target_include_directories(
    qemu_uart_tx_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/communications
)

configure_qemu_target(qemu_uart_tx_tests)


# ==========================================================
# 8. QEMU Motor Driver Tests
# ==========================================================

add_executable(
    qemu_motor_tests

    tests/qemu_target/test_startup.s
    tests/qemu_target/motor_test_main.c
    drivers/motor_driver.c
    memory.c
)

target_include_directories(
    qemu_motor_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

configure_qemu_target(qemu_motor_tests)


# ==========================================================
# 9. QEMU GPIO Shadow Backend Tests
# ==========================================================

add_executable(
    qemu_gpio_tests

    tests/qemu_target/test_startup.s
    tests/qemu_target/gpio_shadow_test_main.c
    drivers/gpio_driver.c
    memory.c
)

target_include_directories(
    qemu_gpio_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

configure_qemu_target(qemu_gpio_tests)


# ==========================================================
# 10. QEMU FK Batch Cross Validation
# ==========================================================

add_executable(
    qemu_fk_batch

    tests/qemu_target/test_startup.s
    tests/qemu_target/fk_batch_test_main.c
    algorithms/kinematics/kinematics_forward.c
    algorithms/math/matrix4.c
    model/ur5_model.c
    memory.c
)

target_include_directories(
    qemu_fk_batch
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
)

configure_qemu_target(qemu_fk_batch m)


# ==========================================================
# 11. QEMU IK Batch Diagnostic
# ==========================================================

add_executable(
    qemu_ik_batch

    tests/qemu_target/test_startup.s
    tests/qemu_target/ik_batch_test_main.c
    model/ur5_model.c
    algorithms/math/matrix4.c
    algorithms/kinematics/kinematics_common.c
    algorithms/kinematics/kinematics_forward.c
    algorithms/kinematics/kinematics_inverse.c
    algorithms/kinematics/ur5_analytic_ik.c
    memory.c
)

target_include_directories(
    qemu_ik_batch
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
)

configure_qemu_target(qemu_ik_batch m)


# ==========================================================
# 12. QEMU Robot Model Limits Tests
# ==========================================================

# 此测试使用独立的 main()，不应链接进 freertos_demo。
add_executable(
    qemu_model_limits_tests

    tests/qemu_target/test_startup.s
    tests/model/robot_model_limits_test.c
    model/ur5_model.c
    memory.c
)

target_include_directories(
    qemu_model_limits_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
)

configure_qemu_target(qemu_model_limits_tests)

# ==========================================================
# Stage 3 - Motion Limits
# ==========================================================

# 生产固件使用的限位算法。
target_sources(
    freertos_demo
    PRIVATE
    algorithms/motion/motion_limits.c
)

target_include_directories(
    freertos_demo
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/motion
)


# ==========================================================
# Cortex-M4 Motion Limits Tests
# ==========================================================

# 独立 QEMU 测试，不与正式固件的 main() 链接。
add_executable(
    qemu_motion_limits_tests

    tests/qemu_target/test_startup.s
    tests/motion/motion_limits_test_main.c

    algorithms/motion/motion_limits.c
    model/ur5_model.c
    memory.c
)

target_include_directories(
    qemu_motion_limits_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/motion
)

# 兼容此前整理版 CMake 和原始分散目标配置。
if(COMMAND configure_qemu_target)
    configure_qemu_target(qemu_motion_limits_tests)
else()
    target_compile_options(
        qemu_motion_limits_tests PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -ffreestanding
        -fno-builtin
        -O0
        -g3
    )

    target_link_options(
        qemu_motion_limits_tests PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -nostdlib
        "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
    )

    target_link_libraries(
        qemu_motion_limits_tests PRIVATE gcc
    )

    set_target_properties(
        qemu_motion_limits_tests PROPERTIES SUFFIX ".elf"
    )
endif()