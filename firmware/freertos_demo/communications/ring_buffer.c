/*
 * 文件：ring_buffer.c
 *
 * 用途：
 * 实现一个固定长度的字节环形缓冲区。
 *
 * 环形缓冲区使用 head 和 tail
 * 分别记录写入位置和读取位置。
 *
 * 当索引到达数组末尾以后，
 * 会重新回到数组起点，
 * 从而循环利用整块存储空间。
 */

#include "ring_buffer.h"


/* =========================================================
 * 初始化
 * ========================================================= */

void ring_buffer_init(
    ring_buffer_t *ring_buffer
)
{
    ring_buffer->head = 0U;

    ring_buffer->tail = 0U;

    ring_buffer->count = 0U;
}


/* =========================================================
 * 写入一个字节
 * ========================================================= */

uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
)
{
    /*
     * 缓冲区已满时不能继续写入，
     * 防止覆盖尚未处理的数据。
     */
    if (
        ring_buffer->count >=
        RING_BUFFER_CAPACITY
    )
    {
        return 0U;
    }


    /*
     * 在当前 head 位置写入数据。
     */
    ring_buffer->buffer[
        ring_buffer->head
    ] = data;


    /*
     * head 前进一个位置。
     *
     * 到达数组末尾以后，
     * 通过取模重新回到 0。
     */
    ring_buffer->head =
        (
            ring_buffer->head + 1U
        )
        % RING_BUFFER_CAPACITY;


    ring_buffer->count++;


    return 1U;
}


/* =========================================================
 * 读取一个字节
 * ========================================================= */

uint8_t ring_buffer_read(
    ring_buffer_t *ring_buffer,
    uint8_t *data
)
{
    /*
     * 缓冲区为空，没有数据可读。
     */
    if (ring_buffer->count == 0U)
    {
        return 0U;
    }


    /*
     * 从当前 tail 位置读取数据。
     */
    *data =
        ring_buffer->buffer[
            ring_buffer->tail
        ];


    /*
     * tail 前进一个位置。
     */
    ring_buffer->tail =
        (
            ring_buffer->tail + 1U
        )
        % RING_BUFFER_CAPACITY;


    ring_buffer->count--;


    return 1U;
}


/* =========================================================
 * 判断是否为空
 * ========================================================= */

uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
)
{
    if (ring_buffer->count == 0U)
    {
        return 1U;
    }


    return 0U;
}


/* =========================================================
 * 判断是否已满
 * ========================================================= */

uint8_t ring_buffer_is_full(
    const ring_buffer_t *ring_buffer
)
{
    if (
        ring_buffer->count >=
        RING_BUFFER_CAPACITY
    )
    {
        return 1U;
    }


    return 0U;
}