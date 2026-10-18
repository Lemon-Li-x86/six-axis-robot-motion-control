
# ==========================================================
# Cortex-M4 / FreeRTOS firmware and standalone QEMU tests
# File: firmware/freertos_demo/CMakeLists.txt
#
# CMake is the single authoritative build system.
# Production and test sources are listed explicitly.
# ==========================================================

# ----------------------------------------------------------
# 1. Dependencies and firmware configuration
# ----------------------------------------------------------

set(FREERTOS_DIR
    "${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel")

# Accept both the kernel repository layout and
# the older Source/ layout.
if(EXISTS "${FREERTOS_DIR}/tasks.c")
    set(FREERTOS_SOURCE_DIR "${FREERTOS_DIR}")
elseif(EXISTS "${FREERTOS_DIR}/Source/tasks.c")
    set(FREERTOS_SOURCE_DIR "${FREERTOS_DIR}/Source")
else()
    include(FetchContent)

    FetchContent_Declare(
        freertos_kernel
        GIT_REPOSITORY
            https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG V11.1.0
        GIT_SHALLOW TRUE
    )

    FetchContent_GetProperties(freertos_kernel)

    if(NOT freertos_kernel_POPULATED)
        FetchContent_Populate(freertos_kernel)
    endif()

    set(FREERTOS_SOURCE_DIR
        "${freertos_kernel_SOURCE_DIR}")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter)

option(
    ROBOT_ENABLE_KINEMATICS_SELF_TEST
    "Run kinematics self-tests before starting FreeRTOS"
    ON
)

option(
    ROBOT_ENABLE_PARSER_BENCHMARK
    "Run protocol parser benchmark before starting FreeRTOS"
    ON
)

set(FIRMWARE_CONFIG_JSON
    "${CMAKE_CURRENT_SOURCE_DIR}/config/robot_defaults.json")

set(FIRMWARE_CONFIG_GENERATOR
    "${CMAKE_SOURCE_DIR}/tools/generate_firmware_defaults.py")

set(FIRMWARE_GENERATED_INCLUDE_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(FIRMWARE_GENERATED_HEADER
    "${FIRMWARE_GENERATED_INCLUDE_DIR}/firmware_defaults_generated.h")

add_custom_command(
    OUTPUT "${FIRMWARE_GENERATED_HEADER}"
    COMMAND
        "${Python3_EXECUTABLE}"
        "${FIRMWARE_CONFIG_GENERATOR}"
        --input "${FIRMWARE_CONFIG_JSON}"
        --output "${FIRMWARE_GENERATED_HEADER}"
    DEPENDS
        "${FIRMWARE_CONFIG_JSON}"
        "${FIRMWARE_CONFIG_GENERATOR}"
    COMMENT
        "Validating JSON and generating firmware defaults"
    VERBATIM
)

add_custom_target(firmware_defaults_header
    DEPENDS "${FIRMWARE_GENERATED_HEADER}")


# ----------------------------------------------------------
# 2. Production firmware
# ----------------------------------------------------------

add_executable(freertos_demo

    # System / BSP
    startup.s
    main.c
    board.c
    memory.c

    # Robot model
    model/ur5_model.c

    # Drivers
    drivers/uart_driver.c
    drivers/motor_driver.c
    drivers/timer_driver.c
    drivers/gpio_driver.c

    # Communications
    communications/protocol.c
    communications/ring_buffer.c

    # Application / tasks and infrastructure
    application/uart_tx_manager.c
    application/protocol_tasks.c
    application/runtime_config.c
    application/motor_state.c

    # Application / command services
    application/services/protocol_commands.c

    # Diagnostics
    diagnostics/performance_monitor.c

    # Algorithms / math
    algorithms/math/matrix4.c

    # Algorithms / kinematics
    algorithms/kinematics/kinematics_common.c
    algorithms/kinematics/kinematics_forward.c
    algorithms/kinematics/ur5_analytic_ik.c
    algorithms/kinematics/kinematics_inverse.c

    # Algorithms / motion
    algorithms/motion/motion_limits.c

    # FreeRTOS kernel and Cortex-M port
    ${FREERTOS_SOURCE_DIR}/tasks.c
    ${FREERTOS_SOURCE_DIR}/list.c
    ${FREERTOS_SOURCE_DIR}/queue.c
    ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM3/port.c
    ${FREERTOS_SOURCE_DIR}/portable/MemMang/heap_4.c
)

add_dependencies(
    freertos_demo
    firmware_defaults_header
)

target_compile_definitions(freertos_demo PRIVATE
    ROBOT_ENABLE_KINEMATICS_SELF_TEST=$<BOOL:${ROBOT_ENABLE_KINEMATICS_SELF_TEST}>
    ROBOT_ENABLE_PARSER_BENCHMARK=$<BOOL:${ROBOT_ENABLE_PARSER_BENCHMARK}>
)

if(ROBOT_ENABLE_KINEMATICS_SELF_TEST)
    target_sources(freertos_demo PRIVATE
        tests/kinematics/kinematics_test_support.c
        tests/kinematics/kinematics_fk_test.c
        tests/kinematics/kinematics_ik_solver_test.c
        tests/kinematics/kinematics_ik_test.c
        tests/kinematics/kinematics_self_test.c
        tests/kinematics/kinematics_dedup_test.c
    )

    target_include_directories(freertos_demo PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/kinematics
    )
endif()

target_include_directories(freertos_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    "${FIRMWARE_GENERATED_INCLUDE_DIR}"

    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/communications

    ${CMAKE_CURRENT_SOURCE_DIR}/application
    ${CMAKE_CURRENT_SOURCE_DIR}/application/services

    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/motion

    ${FREERTOS_SOURCE_DIR}/include
    ${FREERTOS_SOURCE_DIR}/portable/GCC/ARM_CM3
)

target_compile_options(freertos_demo PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -ffreestanding
    -fno-builtin
)

target_link_options(freertos_demo PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -nostdlib
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
)

target_link_libraries(freertos_demo PRIVATE m gcc)

set_target_properties(
    freertos_demo
    PROPERTIES SUFFIX ".elf"
)


# ----------------------------------------------------------
# 3. Common QEMU test target configuration
# ----------------------------------------------------------

function(configure_qemu_target target_name)

    target_compile_options(${target_name} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -ffreestanding
        -fno-builtin
        -O0
        -g3
    )

    target_link_options(${target_name} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -nostdlib
        "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
    )

    target_link_libraries(
        ${target_name}
        PRIVATE
        ${ARGN}
        gcc
    )

    set_target_properties(
        ${target_name}
        PROPERTIES SUFFIX ".elf"
    )
endfunction()


# ----------------------------------------------------------
# 4. QEMU protocol and ring-buffer tests
# ----------------------------------------------------------

add_executable(qemu_protocol_tests
    tests/qemu_target/test_startup.s
    tests/qemu_target/protocol_test_main.c
    communications/protocol.c
    communications/ring_buffer.c
    memory.c
)

target_include_directories(qemu_protocol_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/communications
)

configure_qemu_target(qemu_protocol_tests)


# ----------------------------------------------------------
# 5. QEMU UART TX boundary tests
# ----------------------------------------------------------

add_executable(qemu_uart_tx_tests
    tests/qemu_target/test_startup.s
    tests/qemu_target/uart_tx_boundary_main.c
    drivers/uart_driver.c
    communications/ring_buffer.c
    memory.c
)

target_include_directories(qemu_uart_tx_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/communications
)

configure_qemu_target(qemu_uart_tx_tests)


# ----------------------------------------------------------
# 6. QEMU motor driver tests
# ----------------------------------------------------------

add_executable(qemu_motor_tests
    tests/qemu_target/test_startup.s
    tests/qemu_target/motor_test_main.c
    drivers/motor_driver.c
    memory.c
)

target_include_directories(qemu_motor_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

configure_qemu_target(qemu_motor_tests)


# ----------------------------------------------------------
# 7. QEMU GPIO shadow backend tests
# ----------------------------------------------------------

add_executable(qemu_gpio_tests
    tests/qemu_target/test_startup.s
    tests/qemu_target/gpio_shadow_test_main.c
    drivers/gpio_driver.c
    memory.c
)

target_include_directories(qemu_gpio_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

configure_qemu_target(qemu_gpio_tests)


# ----------------------------------------------------------
# 8. QEMU FK batch cross-validation
# ----------------------------------------------------------

add_executable(qemu_fk_batch
    tests/qemu_target/test_startup.s
    tests/qemu_target/fk_batch_test_main.c
    algorithms/kinematics/kinematics_forward.c
    algorithms/math/matrix4.c
    model/ur5_model.c
    memory.c
)

target_include_directories(qemu_fk_batch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
)

configure_qemu_target(qemu_fk_batch m)


# ----------------------------------------------------------
# 9. QEMU IK batch diagnostics
# ----------------------------------------------------------

add_executable(qemu_ik_batch
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

target_include_directories(qemu_ik_batch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
)

configure_qemu_target(qemu_ik_batch m)


# ----------------------------------------------------------
# 10. QEMU UR5 model limits tests
# ----------------------------------------------------------

add_executable(qemu_model_limits_tests
    tests/qemu_target/test_startup.s
    tests/model/robot_model_limits_test.c
    model/ur5_model.c
    memory.c
)

target_include_directories(qemu_model_limits_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
)

configure_qemu_target(qemu_model_limits_tests)


# ----------------------------------------------------------
# 11. QEMU motion limits tests
# ----------------------------------------------------------

add_executable(qemu_motion_limits_tests
    tests/qemu_target/test_startup.s
    tests/motion/motion_limits_test_main.c
    algorithms/motion/motion_limits.c
    model/ur5_model.c
    memory.c
)

target_include_directories(
    qemu_motion_limits_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/motion
)

configure_qemu_target(qemu_motion_limits_tests)


# ----------------------------------------------------------
# 12. Stage 3: kinematics performance benchmark (QEMU)
# ----------------------------------------------------------

add_executable(qemu_kinematics_perf
    tests/qemu_target/test_startup.s
    tests/qemu_target/kinematics_perf_test_main.c

    drivers/timer_driver.c
    model/ur5_model.c

    algorithms/math/matrix4.c

    algorithms/kinematics/kinematics_common.c
    algorithms/kinematics/kinematics_forward.c
    algorithms/kinematics/kinematics_inverse.c
    algorithms/kinematics/ur5_analytic_ik.c

    algorithms/motion/motion_limits.c
    algorithms/motion/ik_solution_selector.c

    memory.c
)

target_include_directories(qemu_kinematics_perf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/motion
)

configure_qemu_target(qemu_kinematics_perf m)

# Benchmark with optimization enabled.
# Overrides the test helper's earlier -O0.
target_compile_options(qemu_kinematics_perf PRIVATE
    -O2
)
