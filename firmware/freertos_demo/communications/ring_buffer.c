/*
 * 文件：ring_buffer.c
 *
 * 用途：
 * 实现单生产者、单消费者环形缓冲区。
 *
 * 当前主要用于：
 *
 * UART RX ISR
 *      ↓
 * Ring Buffer
 *      ↓
 * ProtocolRX Task
 *
 * Producer 只修改 head。
 * Consumer 只修改 tail。
 *
 * 因此不再使用共享 count 变量。
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
}


/* =========================================================
 * 写入一个字节
 * ========================================================= */

uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
)
{
    uint32_t current_head;

    uint32_t next_head;


    current_head =
        ring_buffer->head;


    next_head =
        (
            current_head + 1U
        )
        % RING_BUFFER_CAPACITY;


    /*
     * 如果下一个 head
     * 已经追上 tail，
     * 表示缓冲区已满。
     */
    if (
        next_head
        == ring_buffer->tail
    )
    {
        return 0U;
    }


    /*
     * 写入当前 head 位置。
     */
    ring_buffer->buffer[
        current_head
    ] = data;


    /*
     * 最后更新 head。
     *
     * 对 Consumer 来说，
     * head 更新以后才代表
     * 新数据已经可读。
     */
    ring_buffer->head =
        next_head;


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
    uint32_t current_tail;


    current_tail =
        ring_buffer->tail;


    /*
     * head == tail：
     * 缓冲区为空。
     */
    if (
        current_tail
        == ring_buffer->head
    )
    {
        return 0U;
    }


    /*
     * 读取当前 tail。
     */
    *data =
        ring_buffer->buffer[
            current_tail
        ];


    /*
     * Consumer 更新 tail。
     */
    ring_buffer->tail =
        (
            current_tail + 1U
        )
        % RING_BUFFER_CAPACITY;


    return 1U;
}


/* =========================================================
 * 判断是否为空
 * ========================================================= */

uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
)
{
    if (
        ring_buffer->head
        == ring_buffer->tail
    )
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
    uint32_t next_head;


    next_head =
        (
            ring_buffer->head + 1U
        )
        % RING_BUFFER_CAPACITY;


    if (
        next_head
        == ring_buffer->tail
    )
    {
        return 1U;
    }


    return 0U;
}