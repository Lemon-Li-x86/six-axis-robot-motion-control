/*
 * 文件：protocol.h
 *
 * 用途：
 * 定义 UART 通信协议的数据格式、协议帧结构、
 * 字节流解析状态机以及对外接口。
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>


/* =========================================================
 * 协议基本定义
 * ========================================================= */

/* 两个固定帧头字节 */
#define PROTOCOL_HEADER_0 0xAAU
#define PROTOCOL_HEADER_1 0x55U


/* =========================================================
 * Command 定义
 * ========================================================= */

/* Cortex-M4 -> Python：设置六轴目标角 */
#define CMD_SET_JOINT_TARGETS 0x01U

/* Python -> Cortex-M4：发送六轴实际状态 */
#define CMD_JOINT_STATE 0x81U

/* Cortex-M4 -> Python：确认已经收到状态 */
#define CMD_JOINT_STATE_ACK 0x82U


/* =========================================================
 * 数据长度定义
 * ========================================================= */

/* 六轴机器人共有 6 个关节 */
#define PROTOCOL_JOINT_COUNT 6U

/*
 * 每个关节角使用 int16_t：
 *
 * 6 × 2 Byte = 12 Byte
 */
#define PROTOCOL_JOINT_PAYLOAD_LEN 12U

/*
 * 当前六轴关节帧长度：
 *
 * 2 Byte Header
 * 1 Byte Command
 * 1 Byte Length
 * 12 Byte Payload
 * 1 Byte Checksum
 *
 * 总计 17 Byte
 */
#define PROTOCOL_JOINT_FRAME_LEN 17U

/*
 * 通用协议允许的最大 Payload 长度。
 *
 * 当前六轴数据只需要 12 Byte，
 * 这里预留 64 Byte，
 * 为以后增加其他 Command 留出空间。
 */
#define PROTOCOL_MAX_PAYLOAD_LEN 64U


/* =========================================================
 * 通用协议帧结构
 * ========================================================= */

/*
 * 一帧经过协议状态机完整解析以后，
 * 保存为该结构体。
 *
 * 帧头和 Checksum 不需要继续交给应用层，
 * 因此这里只保存：
 *
 * Command
 * Length
 * Payload
 */
typedef struct
{
    uint8_t command;

    uint8_t length;

    uint8_t payload[
        PROTOCOL_MAX_PAYLOAD_LEN
    ];

} protocol_frame_t;


/* =========================================================
 * 协议解析状态
 * ========================================================= */

/*
 * 状态机依次经历：
 *
 * 等 AA
 * -> 等 55
 * -> Command
 * -> Length
 * -> Payload
 * -> Checksum
 */
typedef enum
{
    PROTOCOL_STATE_WAIT_HEADER_0 = 0,

    PROTOCOL_STATE_WAIT_HEADER_1,

    PROTOCOL_STATE_READ_COMMAND,

    PROTOCOL_STATE_READ_LENGTH,

    PROTOCOL_STATE_READ_PAYLOAD,

    PROTOCOL_STATE_READ_CHECKSUM

} protocol_parser_state_t;


/* =========================================================
 * 协议解析器
 * ========================================================= */

/*
 * 保存当前字节流解析进行到哪里。
 *
 * 因为 UART 每次只收到一个 byte，
 * 所以解析器必须记住：
 *
 * 当前状态
 * 当前帧内容
 * Payload 已收到多少字节
 * 当前 Checksum
 */
typedef struct
{
    protocol_parser_state_t state;

    protocol_frame_t frame;

    uint8_t payload_index;

    uint8_t checksum;

} protocol_parser_t;


/* =========================================================
 * 字节流解析接口
 * ========================================================= */

/**
 * @brief 初始化协议解析器。
 *
 * @param parser 协议解析器。
 */
void protocol_parser_init(
    protocol_parser_t *parser
);


/**
 * @brief 向协议状态机输入一个字节。
 *
 * UART 每收到一个字节，就调用一次该函数。
 *
 * @param parser 当前协议解析器。
 * @param byte 当前收到的字节。
 * @param output_frame 用于保存解析完成的协议帧。
 *
 * @return 1：已经得到一帧完整且 Checksum 正确的数据。
 *         0：当前还没有得到完整协议帧。
 */
uint8_t protocol_parser_process_byte(
    protocol_parser_t *parser,
    uint8_t byte,
    protocol_frame_t *output_frame
);


/* =========================================================
 * 六轴关节协议接口
 * ========================================================= */

/**
 * @brief 构造六轴目标角协议帧。
 */
void protocol_build_joint_target_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 构造六轴状态 ACK 协议帧。
 */
void protocol_build_joint_state_ack_frame(
    const int16_t joints[PROTOCOL_JOINT_COUNT],
    uint8_t frame[PROTOCOL_JOINT_FRAME_LEN]
);


/**
 * @brief 从完整协议帧中解析六轴实际状态。
 *
 * @return 1：解析成功。
 *         0：Command 或 Payload 长度不正确。
 */
uint8_t protocol_parse_joint_state(
    const protocol_frame_t *frame,
    int16_t joints[PROTOCOL_JOINT_COUNT]
);


#endif