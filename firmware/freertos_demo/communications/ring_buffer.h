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
 * Producer 只修改 head，
 * Consumer 只修改 tail。
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>


/*
 * 数组物理容量：
 *
 * 128 Byte。
 *
 * 为了通过 head == tail 表示空，
 * 保留一个位置不用。
 *
 * 实际最大可保存：
 *
 * 127 Byte。
 */
#define RING_BUFFER_CAPACITY 128U


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
     * 仅由 Producer 修改。
     */
    volatile uint32_t head;


    /*
     * 下一个读取位置。
     *
     * 仅由 Consumer 修改。
     */
    volatile uint32_t tail;

} ring_buffer_t;


/**
 * @brief 初始化 Ring Buffer。
 *
 * @param[in,out] ring_buffer
 * 待初始化的 Ring Buffer 对象。
 */
void ring_buffer_init(
    ring_buffer_t *ring_buffer
);


/**
 * @brief 向 Ring Buffer 写入一个字节。
 *
 * @param[in,out] ring_buffer
 * Ring Buffer 对象。
 *
 * @param[in] data
 * 待写入字节。
 *
 * @return
 * 1：
 * 写入成功。
 *
 * 0：
 * Ring Buffer 已满，
 * 当前字节未写入。
 */
uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
);


/**
 * @brief 从 Ring Buffer 读取一个字节。
 *
 * @param[in,out] ring_buffer
 * Ring Buffer 对象。
 *
 * @param[out] data
 * 输出读取到的字节。
 *
 * @return
 * 1：
 * 读取成功。
 *
 * 0：
 * Ring Buffer 为空。
 */
uint8_t ring_buffer_read(
    ring_buffer_t *ring_buffer,
    uint8_t *data
);


/**
 * @brief 判断 Ring Buffer 是否为空。
 *
 * @param[in] ring_buffer
 * Ring Buffer 对象。
 *
 * @return
 * 1：
 * Ring Buffer 为空。
 *
 * 0：
 * Ring Buffer 中存在数据。
 */
uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
);


/**
 * @brief 判断 Ring Buffer 是否已满。
 *
 * @param[in] ring_buffer
 * Ring Buffer 对象。
 *
 * @return
 * 1：
 * Ring Buffer 已满。
 *
 * 0：
 * Ring Buffer 仍有可写空间。
 */
uint8_t ring_buffer_is_full(
    const ring_buffer_t *ring_buffer
);


#endif