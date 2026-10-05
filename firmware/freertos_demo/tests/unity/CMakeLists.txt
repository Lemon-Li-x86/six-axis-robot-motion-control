cmake_minimum_required(VERSION 3.20)

project(
    stage2_unity_tests
    LANGUAGES C
)

set(
    REPO_ROOT
    "${CMAKE_CURRENT_LIST_DIR}/../../../.."
)

set(
    FIRMWARE_DIR
    "${REPO_ROOT}/firmware/freertos_demo"
)

set(
    UNITY_DIR
    "${REPO_ROOT}/third_party/Unity"
)

if(
    NOT EXISTS
    "${UNITY_DIR}/src/unity.c"
)
    message(
        FATAL_ERROR
        "Unity not found. Clone it into third_party/Unity first."
    )
endif()

add_executable(
    stage2_unity_tests

    test_main.c
    test_protocol.c
    test_motor_driver.c
    test_ring_buffer.c

    test_uart_driver.c
    uart_driver_test_backend.c

    ${UNITY_DIR}/src/unity.c

    ${FIRMWARE_DIR}/communications/protocol.c
    ${FIRMWARE_DIR}/communications/ring_buffer.c
    ${FIRMWARE_DIR}/drivers/motor_driver.c
    ${FIRMWARE_DIR}/drivers/uart_driver.c
)

target_include_directories(
    stage2_unity_tests

    PRIVATE

    ${CMAKE_CURRENT_LIST_DIR}

    ${UNITY_DIR}/src

    ${FIRMWARE_DIR}/common
    ${FIRMWARE_DIR}/communications
    ${FIRMWARE_DIR}/drivers
)

target_compile_definitions(
    stage2_unity_tests

    PRIVATE

    UART_DRIVER_HOST_TEST=1
)

set_target_properties(
    stage2_unity_tests

    PROPERTIES

    C_STANDARD 11
    C_STANDARD_REQUIRED YES
)

enable_testing()

add_test(
    NAME stage2_unity_tests
    COMMAND stage2_unity_tests
)