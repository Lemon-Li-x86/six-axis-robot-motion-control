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
 */

#ifndef ERROR_CODE_H
#define ERROR_CODE_H


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
     * 未分类内部错误。
     */
    ROBOT_STATUS_ERROR_INTERNAL = -8,


    /*
     * 输入数据超过允许范围。
     */
    ROBOT_STATUS_ERROR_OUT_OF_RANGE = -9,


    /*
     * 当前目标不存在可行解。
     *
     * 主要用于逆运动学。
     */
    ROBOT_STATUS_ERROR_NO_SOLUTION = -10,


    /*
     * 检测到运动学奇异状态。
     */
    ROBOT_STATUS_ERROR_SINGULAR = -11,


    /*
     * 接口已经定义，
     * 但算法实现尚未完成。
     */
    ROBOT_STATUS_ERROR_NOT_IMPLEMENTED = -12

} robot_status_t;


#endif