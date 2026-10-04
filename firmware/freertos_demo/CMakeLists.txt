# ==========================================================
# 文件：firmware/freertos_demo/CMakeLists.txt
#
# 用途：
# 配置 Cortex-M4 / FreeRTOS 固件工程。
# ==========================================================


# ==========================================================
# FreeRTOS Kernel 路径
# ==========================================================

set(
    FREERTOS_DIR
    "${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel"
)


# ==========================================================
# 固件源文件
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
# Include Paths
# ==========================================================

target_include_directories(
    freertos_demo

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}


    # ------------------------------------------------------
    # Common
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/common


    # ------------------------------------------------------
    # Robot Model
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/model


    # ------------------------------------------------------
    # Drivers
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/drivers


    # ------------------------------------------------------
    # Communications
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/communications


    # ------------------------------------------------------
    # Application
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/application


    # ------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/diagnostics


    # ------------------------------------------------------
    # Algorithms
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/math

    ${CMAKE_CURRENT_SOURCE_DIR}/algorithms/kinematics


    # ------------------------------------------------------
    # Tests
    # ------------------------------------------------------

    ${CMAKE_CURRENT_SOURCE_DIR}/tests/kinematics


    # ------------------------------------------------------
    # FreeRTOS
    # ------------------------------------------------------

    ${FREERTOS_DIR}/include

    ${FREERTOS_DIR}/portable/GCC/ARM_CM3
)


# ==========================================================
# Compile Options
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
# Link Options
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
# Libraries
# ==========================================================

target_link_libraries(
    freertos_demo

    PRIVATE

    m
    gcc
)


# ==========================================================
# Output
# ==========================================================

set_target_properties(
    freertos_demo

    PROPERTIES

    SUFFIX ".elf"
)