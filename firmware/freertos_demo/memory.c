/*
 * 文件：memory.c
 *
 * 用途：
 * 为 freestanding 固件提供最小内存操作实现。
 *
 * 当前固件使用 -nostdlib，
 * 因此需要自行提供 FreeRTOS Kernel
 * 所依赖的基础内存函数：
 *
 * 1. memset()；
 * 2. memcpy()。
 */

#include <stddef.h>


/**
 * @brief 将指定内存区域设置为同一字节值。
 *
 * @param[out] ptr
 * 目标内存起始地址。
 *
 * @param[in] value
 * 填充值。
 *
 * 实际使用其低 8 bit。
 *
 * @param[in] num
 * 需要设置的 Byte 数量。
 *
 * @return
 * 原始目标地址 ptr。
 */
void *memset(
    void *ptr,
    int value,
    size_t num
)
{
    unsigned char *destination =
        (unsigned char *)ptr;

    unsigned char byte_value =
        (unsigned char)value;

    size_t i;

    for (i = 0U; i < num; i++)
    {
        destination[i] = byte_value;
    }

    return ptr;
}


/**
 * @brief 将指定数量的字节复制到目标内存。
 *
 * @param[out] destination
 * 目标内存起始地址。
 *
 * @param[in] source
 * 源内存起始地址。
 *
 * @param[in] num
 * 需要复制的 Byte 数量。
 *
 * @return
 * 原始目标地址 destination。
 *
 * @note
 * 当前为最小 memcpy 实现，
 * 不处理源区域和目标区域重叠的情况。
 *
 * 如果需要处理重叠内存区域，
 * 应使用 memmove() 语义。
 */
void *memcpy(
    void *destination,
    const void *source,
    size_t num
)
{
    unsigned char *dst =
        (unsigned char *)destination;

    const unsigned char *src =
        (const unsigned char *)source;

    size_t i;

    for (i = 0U; i < num; i++)
    {
        dst[i] = src[i];
    }

    return destination;
}