#include <stddef.h>

/*
 * 最小版 memset()
 *
 * 将从 ptr 开始的 num 个字节，
 * 全部设置为 value 的低 8 位。
 *
 * FreeRTOS 的 tasks.c 和 heap_4.c
 * 会使用 memset() 清空部分内存。
 */
void *memset(void *ptr, int value, size_t num)
{
    unsigned char *destination = (unsigned char *)ptr;
    unsigned char byte_value = (unsigned char)value;

    for (size_t i = 0; i < num; i++)
    {
        destination[i] = byte_value;
    }

    return ptr;
}


/*
 * 最小版 memcpy()
 *
 * 将 source 中的 num 个字节
 * 复制到 destination。
 *
 * FreeRTOS 的 queue.c 会使用 memcpy()
 * 在队列中复制数据。
 */
void *memcpy(void *destination, const void *source, size_t num)
{
    unsigned char *dst = (unsigned char *)destination;
    const unsigned char *src = (const unsigned char *)source;

    for (size_t i = 0; i < num; i++)
    {
        dst[i] = src[i];
    }

    return destination;
}