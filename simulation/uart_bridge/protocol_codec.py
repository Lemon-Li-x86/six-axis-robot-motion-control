"""
文件：protocol_codec.py

用途：
统一 Python 仿真与测试侧的 UART 二进制协议编解码。

本模块只负责协议格式，不负责：

1. Socket；
2. PyBullet；
3. 测试流程；
4. Application 业务逻辑。

协议格式与 firmware/freertos_demo/communications/protocol.h
保持一致。
"""

import struct
from typing import Optional

from angle_utils import (
    canonical_raw_to_degree,
    degree_to_canonical_raw,
)


# ==========================================================
# Protocol Header
# ==========================================================

HEADER = b"\xAA\x55"


# ==========================================================
# Command
# ==========================================================

CMD_SET_JOINT_TARGETS = 0x01

CMD_JOINT_STATE = 0x81

CMD_JOINT_STATE_ACK = 0x82

CMD_GET_DIAGNOSTICS = 0x83

CMD_DIAGNOSTICS_RESPONSE = 0x84


# ==========================================================
# Joint Protocol
# ==========================================================

JOINT_COUNT = 6

JOINT_PAYLOAD_LEN = JOINT_COUNT * 2

JOINT_FRAME_LEN = (
    2
    + 1
    + 1
    + JOINT_PAYLOAD_LEN
    + 1
)

_JOINT_STRUCT = struct.Struct("<6h")

_JOINT_COMMANDS = {
    CMD_SET_JOINT_TARGETS,
    CMD_JOINT_STATE,
    CMD_JOINT_STATE_ACK,
}


# ==========================================================
# Diagnostics Protocol
# ==========================================================

DIAGNOSTICS_METRIC_RX_DROP_COUNT = 0

DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ = 1

DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT = 2

DIAGNOSTICS_METRIC_PARSER_MIN_TICKS = 3

DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS = 4

DIAGNOSTICS_METRIC_PARSER_MAX_TICKS = 5

DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT = 6

DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS = 7

DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS = 8

DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS = 9

DIAGNOSTICS_PAYLOAD_LEN = 4

_DIAGNOSTICS_RESPONSE_STRUCT = struct.Struct("<I")


# ==========================================================
# Generic Protocol
# ==========================================================

PROTOCOL_MAX_PAYLOAD_LEN = 64

PROTOCOL_MIN_FRAME_LEN = 5


def build_frame(
    command: int,
    payload: bytes = b"",
) -> bytes:
    """
    构造通用协议帧。

    Frame：

        AA 55
        Command
        Length
        Payload
        Checksum

    Checksum：

        sum(Command + Length + Payload) & 0xFF
    """
    if not 0 <= command <= 0xFF:
        raise ValueError(
            "command 必须位于 [0, 255]"
        )

    payload = bytes(payload)

    if len(payload) > PROTOCOL_MAX_PAYLOAD_LEN:
        raise ValueError(
            "payload 超过 PROTOCOL_MAX_PAYLOAD_LEN"
        )

    body = (
        bytes([
            command,
            len(payload),
        ])
        + payload
    )

    checksum = (
        sum(body)
        & 0xFF
    )

    return (
        HEADER
        + body
        + bytes([checksum])
    )


def build_joint_payload(
    angles_deg: list[float],
) -> bytes:
    """
    六轴 degree 角度 -> 12 Byte Joint Payload。

    每个关节：

        int16
        1 unit = 0.01 degree
        Canonical Range = [-180°, 180°)
    """
    if len(angles_deg) != JOINT_COUNT:
        raise ValueError(
            f"angles_deg 必须包含 {JOINT_COUNT} 个关节角"
        )

    raw_angles = [
        degree_to_canonical_raw(
            angle_deg
        )
        for angle_deg in angles_deg
    ]

    return _JOINT_STRUCT.pack(
        *raw_angles
    )


def build_joint_frame(
    command: int,
    angles_deg: list[float],
) -> bytes:
    """
    构造六轴 Joint Protocol Frame。
    """
    if command not in _JOINT_COMMANDS:
        raise ValueError(
            "command 不是 Joint Protocol Command"
        )

    return build_frame(
        command,
        build_joint_payload(
            angles_deg
        ),
    )


def build_diagnostics_request(
    selector: Optional[int] = None,
) -> bytes:
    """
    构造 Diagnostics Request。

    selector = None：
        Length = 0
        按固件兼容规则查询 RX Drop Count。

    selector = 0..255：
        Length = 1
        Payload[0] = selector。
    """
    if selector is None:
        payload = b""
    else:
        if not 0 <= selector <= 0xFF:
            raise ValueError(
                "selector 必须位于 [0, 255]"
            )

        payload = bytes([
            selector
        ])

    return build_frame(
        CMD_GET_DIAGNOSTICS,
        payload
    )


def parse_frame(
    frame: bytes,
):
    """
    解析完整协议帧。

    成功：

        command, payload

    失败：

        None, None
    """
    if len(frame) < PROTOCOL_MIN_FRAME_LEN:
        return None, None

    if frame[0:2] != HEADER:
        return None, None

    command = frame[2]
    payload_length = frame[3]

    if payload_length > PROTOCOL_MAX_PAYLOAD_LEN:
        return None, None

    expected_length = (
        PROTOCOL_MIN_FRAME_LEN
        + payload_length
    )

    if len(frame) != expected_length:
        return None, None

    received_checksum = frame[-1]

    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )

    if received_checksum != calculated_checksum:
        return None, None

    payload = frame[4:-1]

    return command, payload


def parse_joint_payload(
    payload: bytes,
):
    """
    解析 12 Byte Joint Payload。

    成功：
        list[float]

    失败：
        None
    """
    if len(payload) != JOINT_PAYLOAD_LEN:
        return None

    raw_angles = _JOINT_STRUCT.unpack(
        payload
    )

    return [
        canonical_raw_to_degree(
            raw_angle
        )
        for raw_angle in raw_angles
    ]


def parse_joint_frame(
    frame: bytes,
):
    """
    解析 Joint Protocol Frame。

    成功：

        command, angles_deg

    非 Joint Command：

        command, None

    非法 Frame：

        None, None
    """
    command, payload = parse_frame(
        frame
    )

    if command is None:
        return None, None

    if command not in _JOINT_COMMANDS:
        return command, None

    angles_deg = parse_joint_payload(
        payload
    )

    return command, angles_deg


def parse_diagnostics_response_payload(
    payload: bytes,
):
    """
    解析 Diagnostics Response uint32 Payload。

    非法长度返回 None。
    """
    if len(payload) != DIAGNOSTICS_PAYLOAD_LEN:
        return None

    return _DIAGNOSTICS_RESPONSE_STRUCT.unpack(
        payload
    )[0]


def extract_frames(
    buffer: bytearray,
) -> list[bytes]:
    """
    从 TCP 字节流缓冲区中提取完整协议帧。

    本函数会：

    1. 搜索 AA 55 Header；
    2. 丢弃 Header 前噪声；
    3. 等待完整 Frame；
    4. 对明显非法 Length 做重新同步。

    Checksum 校验仍由 parse_frame() 完成。
    """
    frames = []

    while True:
        if len(buffer) < 2:
            break

        header_index = buffer.find(
            HEADER
        )

        if header_index < 0:
            # 如果最后一个 Byte 是 0xAA，
            # 保留它，避免 Header 被 TCP 分片拆开。
            if buffer[-1:] == HEADER[:1]:
                del buffer[:-1]
            else:
                buffer.clear()

            break

        if header_index > 0:
            del buffer[:header_index]

        if len(buffer) < 4:
            break

        payload_length = buffer[3]

        if payload_length > PROTOCOL_MAX_PAYLOAD_LEN:
            # 当前 Header 后的 Length 明显非法。
            # 丢弃第一个 Header Byte，重新搜索。
            del buffer[0]
            continue

        frame_length = (
            PROTOCOL_MIN_FRAME_LEN
            + payload_length
        )

        if len(buffer) < frame_length:
            break

        frame = bytes(
            buffer[:frame_length]
        )

        del buffer[:frame_length]

        frames.append(
            frame
        )

    return frames