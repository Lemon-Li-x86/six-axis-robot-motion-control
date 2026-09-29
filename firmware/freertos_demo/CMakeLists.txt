# ==========================================================
# FreeRTOS Cortex-M4 Demo
# ==========================================================

# FreeRTOS Kernel 路径
set(
    FREERTOS_DIR
    "${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel"
)


# ==========================================================
# 固件源文件
# ==========================================================

add_executable(
    freertos_demo

    # 自己的代码
    startup.s
    main.c
    board.c
    protocol.c
    memory.c

    # FreeRTOS Kernel
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c

    # Cortex-M port
    ${FREERTOS_DIR}/portable/GCC/ARM_CM3/port.c

    # 动态内存管理
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)


# ==========================================================
# 头文件路径
# ==========================================================

target_include_directories(
    freertos_demo

    PRIVATE

    # FreeRTOSConfig.h
    ${CMAKE_CURRENT_SOURCE_DIR}

    # FreeRTOS 公共头文件
    ${FREERTOS_DIR}/include

    # Cortex-M port 头文件
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

    # 使用自己的 linker script
    "-T${CMAKE_CURRENT_SOURCE_DIR}/linker.ld"
)


# GCC 自己需要的底层运算库
target_link_libraries(
    freertos_demo

    PRIVATE

    gcc
)


# ==========================================================
# 输出文件扩展名
# ==========================================================

set_target_properties(
    freertos_demo

    PROPERTIES

    SUFFIX ".elf"
)