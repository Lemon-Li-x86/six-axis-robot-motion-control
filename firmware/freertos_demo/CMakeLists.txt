# ==========================================================
# 文件：firmware/freertos_demo/CMakeLists.txt
#
# 用途：
# 配置 Cortex-M4 / FreeRTOS 固件工程。
#
# 主要负责：
# 1. 指定项目源文件；
# 2. 指定 FreeRTOS Kernel 源文件；
# 3. 设置各模块头文件搜索路径；
# 4. 配置 Cortex-M4 编译参数；
# 5. 配置链接脚本；
# 6. 生成 freertos_demo.elf。
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
    # 系统入口 / 板级代码
    # ------------------------------------------------------

    startup.s
    main.c
    board.c
    memory.c


    # ------------------------------------------------------
    # Driver 层
    # ------------------------------------------------------

    drivers/uart_driver.c
    drivers/motor_driver.c


    # ------------------------------------------------------
    # Communication 层
    # ------------------------------------------------------

    communications/protocol.c
    communications/ring_buffer.c


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
    # FreeRTOS 动态内存管理
    # ------------------------------------------------------

    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)


# ==========================================================
# 头文件搜索路径
# ==========================================================

target_include_directories(
    freertos_demo

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}

    # Common 公共类型与错误码
    ${CMAKE_CURRENT_SOURCE_DIR}/common

    # Driver 层头文件
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers

    # Communication 层头文件
    ${CMAKE_CURRENT_SOURCE_DIR}/communications

    ${FREERTOS_DIR}/include

    ${FREERTOS_DIR}/portable/GCC/ARM_CM3
)


# ==========================================================
# 编译选项
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
# 链接选项
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
# GCC 底层运行库
# ==========================================================

target_link_libraries(
    freertos_demo

    PRIVATE

    gcc
)


# ==========================================================
# 输出文件
# ==========================================================

set_target_properties(
    freertos_demo

    PROPERTIES

    SUFFIX ".elf"
)