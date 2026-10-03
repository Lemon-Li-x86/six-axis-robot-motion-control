/*
 * 文件：error_code.h
 *
 * 用途：
 * 定义机器人固件各模块统一使用的返回状态码。
 *
 * 约定：
 *
 * 0：
 * 成功。
 *
 * 负数：
 * 错误。
 *
 * 后续 Driver、Communication、Algorithm、
 * Control 等模块可以统一使用 robot_status_t，
 * 避免每个模块自行定义不同的错误表示方式。
 */

#ifndef ERROR_CODE_H
#define ERROR_CODE_H


/* =========================================================
 * 通用状态码
 * ========================================================= */

typedef enum
{
    /*
     * 操作成功。
     */
    ROBOT_STATUS_OK = 0,


    /*
     * 传入 NULL Pointer。
     */
    ROBOT_STATUS_ERROR_NULL_POINTER = -1,


    /*
     * 普通参数非法。
     */
    ROBOT_STATUS_ERROR_INVALID_ARGUMENT = -2,


    /*
     * Command 不符合当前接口要求。
     */
    ROBOT_STATUS_ERROR_INVALID_COMMAND = -3,


    /*
     * 数据长度非法。
     */
    ROBOT_STATUS_ERROR_INVALID_LENGTH = -4,


    /*
     * Checksum 校验失败。
     */
    ROBOT_STATUS_ERROR_CHECKSUM = -5,


    /*
     * 软件缓冲区已满。
     */
    ROBOT_STATUS_ERROR_BUFFER_FULL = -6,


    /*
     * 当前模块或资源尚未准备完成。
     */
    ROBOT_STATUS_ERROR_NOT_READY = -7,


    /*
     * 未分类的内部错误。
     */
    ROBOT_STATUS_ERROR_INTERNAL = -8

} robot_status_t;


#endif