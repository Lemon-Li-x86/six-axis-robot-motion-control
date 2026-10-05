/*
 * 文件：ring_buffer.c
 *
 * 用途：
 * 实现单生产者、单消费者
 * Byte Ring Buffer。
 *
 * 当前主要用于：
 *
 * UART RX ISR
 *      ↓
 * Ring Buffer
 *      ↓
 * ProtocolRX Task
 *
 * Producer 只修改 head；
 * Consumer 只修改 tail。
 *
 * 因此不需要共享 count 变量。
 */

#include "ring_buffer.h"


/* =========================================================
 * Public API
 * ========================================================= */

void ring_buffer_init(
    ring_buffer_t *ring_buffer
)
{
    ring_buffer->head = 0U;
    ring_buffer->tail = 0U;
}


uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
)
{
    uint32_t current_head;
    uint32_t next_head;

    current_head = ring_buffer->head;

    next_head =
        (current_head + 1U)
        % RING_BUFFER_CAPACITY;

    /*
     * head == tail 用于表示 Empty，
     * 因此必须保留一个 Slot 不使用。
     *
     * 当 next_head 追上 tail 时，
     * Ring Buffer 已满。
     */
    if (next_head == ring_buffer->tail)
    {
        return 0U;
    }

    /*
     * 先写入数据，再发布新的 head。
     *
     * 对 Consumer 而言，
     * head 更新后才表示新 Byte 已经可读。
     */
    ring_buffer->buffer[current_head] = data;
    ring_buffer->head = next_head;

    return 1U;
}


uint8_t ring_buffer_read(
    ring_buffer_t *ring_buffer,
    uint8_t *data
)
{
    uint32_t current_tail;

    current_tail = ring_buffer->tail;

    /*
     * head == tail 表示当前没有可读数据。
     */
    if (current_tail == ring_buffer->head)
    {
        return 0U;
    }

    /*
     * 先读取数据，再推进 tail。
     *
     * Producer 只读取 tail，
     * 不会修改该变量。
     */
    *data = ring_buffer->buffer[current_tail];

    ring_buffer->tail =
        (current_tail + 1U)
        % RING_BUFFER_CAPACITY;

    return 1U;
}


uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
)
{
    return (
        ring_buffer->head == ring_buffer->tail
    ) ? 1U : 0U;
}


uint8_t ring_buffer_is_full(
    const ring_buffer_t *ring_buffer
)
{
    uint32_t next_head;

    next_head =
        (ring_buffer->head + 1U)
        % RING_BUFFER_CAPACITY;

    return (
        next_head == ring_buffer->tail
    ) ? 1U : 0U;
}