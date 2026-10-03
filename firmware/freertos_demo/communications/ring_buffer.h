/*
 * 文件：ring_buffer.h
 *
 * 用途：
 * 定义单生产者、单消费者字节环形缓冲区。
 *
 * 当前使用方式：
 *
 * Producer：
 * UART RX ISR
 *
 * Consumer：
 * ProtocolRX Task
 *
 * 通过分别维护 head 和 tail，
 * 避免 ISR 和 Task 同时修改同一个 count 变量。
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>


/* =========================================================
 * Ring Buffer 容量
 * ========================================================= */

/*
 * 数组物理容量为 128 Byte。
 *
 * 为了通过 head == tail 表示空，
 * 会保留一个位置不用。
 *
 * 因此实际最大可保存：
 *
 * 127 Byte。
 */
#define RING_BUFFER_CAPACITY 128U


/* =========================================================
 * Ring Buffer 数据结构
 * ========================================================= */

typedef struct
{
    /*
     * 实际数据存储区。
     */
    volatile uint8_t buffer[
        RING_BUFFER_CAPACITY
    ];


    /*
     * 下一个写入位置。
     *
     * 只由 Producer 修改。
     */
    volatile uint32_t head;


    /*
     * 下一个读取位置。
     *
     * 只由 Consumer 修改。
     */
    volatile uint32_t tail;

} ring_buffer_t;


/* =========================================================
 * 接口
 * ========================================================= */

/**
 * @brief 初始化 Ring Buffer。
 */
void ring_buffer_init(
    ring_buffer_t *ring_buffer
);


/**
 * @brief 写入一个字节。
 *
 * @return 1：成功。
 *         0：缓冲区已满。
 */
uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
);


/**
 * @brief 读取一个字节。
 *
 * @return 1：成功。
 *         0：缓冲区为空。
 */
uint8_t ring_buffer_read(
    ring_buffer_t *ring_buffer,
    uint8_t *data
);


/**
 * @brief 判断 Ring Buffer 是否为空。
 */
uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
);


/**
 * @brief 判断 Ring Buffer 是否已满。
 */
uint8_t ring_buffer_is_full(
    const ring_buffer_t *ring_buffer
);


#endif