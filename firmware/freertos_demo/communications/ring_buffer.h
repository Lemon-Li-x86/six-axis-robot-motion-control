/*
 * 文件：ring_buffer.h
 *
 * 用途：
 * 定义通用环形缓冲区的数据结构和操作接口。
 *
 * 当前主要用于缓存 UART 接收到的字节，
 * 将 UART 数据接收与协议解析过程解耦。
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>


/* =========================================================
 * 环形缓冲区容量
 * ========================================================= */

#define RING_BUFFER_CAPACITY 128U


/* =========================================================
 * 环形缓冲区结构
 * ========================================================= */

typedef struct
{
    uint8_t buffer[RING_BUFFER_CAPACITY];

    /* 下一个写入位置 */
    uint32_t head;

    /* 下一个读取位置 */
    uint32_t tail;

    /* 当前已经保存的数据数量 */
    uint32_t count;

} ring_buffer_t;


/* =========================================================
 * 对外接口
 * ========================================================= */

/**
 * @brief 初始化环形缓冲区。
 */
void ring_buffer_init(
    ring_buffer_t *ring_buffer
);


/**
 * @brief 向环形缓冲区写入一个字节。
 *
 * @return 1：写入成功。
 *         0：缓冲区已满。
 */
uint8_t ring_buffer_write(
    ring_buffer_t *ring_buffer,
    uint8_t data
);


/**
 * @brief 从环形缓冲区读取一个字节。
 *
 * @return 1：读取成功。
 *         0：缓冲区为空。
 */
uint8_t ring_buffer_read(
    ring_buffer_t *ring_buffer,
    uint8_t *data
);


/**
 * @brief 判断环形缓冲区是否为空。
 *
 * @return 1：为空。
 *         0：存在数据。
 */
uint8_t ring_buffer_is_empty(
    const ring_buffer_t *ring_buffer
);


/**
 * @brief 判断环形缓冲区是否已满。
 *
 * @return 1：已满。
 *         0：仍有空间。
 */
uint8_t ring_buffer_is_full(
    const ring_buffer_t *ring_buffer
);


#endif