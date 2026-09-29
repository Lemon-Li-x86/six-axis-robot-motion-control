# ==========================================================
# ARM Cortex-M4 交叉编译工具链
# ==========================================================

# 目标不是 Windows / Linux 主机，而是裸机系统
set(CMAKE_SYSTEM_NAME Generic)

# 目标处理器
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

# CMake 测试编译器时不要尝试生成可执行文件运行
set(
    CMAKE_TRY_COMPILE_TARGET_TYPE
    STATIC_LIBRARY
)

# ARM GNU Toolchain
set(
    CMAKE_C_COMPILER
    arm-none-eabi-gcc
)

set(
    CMAKE_ASM_COMPILER
    arm-none-eabi-gcc
)