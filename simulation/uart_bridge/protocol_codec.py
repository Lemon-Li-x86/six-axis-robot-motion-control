"""
文件：protocol_codec.py

用途：
统一 Python 仿真与测试侧的 UART 二进制协议编解码。

本模块只负责协议格式，不负责：
1. Socket；
2. PyBullet；
3. Application 业务逻辑。
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
# Commands
# ==========================================================

CMD_SET_JOINT_TARGETS = 0x01
CMD_SET_PARAMETER = 0x02

CMD_JOINT_STATE = 0x81
CMD_JOINT_STATE_ACK = 0x82
CMD_GET_DIAGNOSTICS = 0x83
CMD_DIAGNOSTICS_RESPONSE = 0x84
CMD_PARAMETER_ACK = 0x85


# ==========================================================
# Generic Protocol
# ==========================================================

PROTOCOL_MAX_PAYLOAD_LEN = 64
PROTOCOL_MIN_FRAME_LEN = 5


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
# Parameter Protocol
# ==========================================================

PARAM_PROTOCOL_TX_PERIOD_MS = 0x01

PARAMETER_REQUEST_PAYLOAD_LEN = 5
PARAMETER_ACK_PAYLOAD_LEN = 6

_PARAMETER_REQUEST_STRUCT = struct.Struct("<BI")
_PARAMETER_ACK_STRUCT = struct.Struct("<BbI")


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
# Generic Frame
# ==========================================================

def build_frame(
    command: int,
    payload: bytes = b"",
) -> bytes:
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

    checksum = sum(body) & 0xFF

    return (
        HEADER
        + body
        + bytes([checksum])
    )


def parse_frame(
    frame: bytes,
):
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

    return command, frame[4:-1]


# ==========================================================
# Joint Protocol
# ==========================================================

def build_joint_payload(
    angles_deg: list[float],
) -> bytes:
    if len(angles_deg) != JOINT_COUNT:
        raise ValueError(
            f"angles_deg 必须包含 {JOINT_COUNT} 个关节角"
        )

    raw_angles = [
        degree_to_canonical_raw(angle_deg)
        for angle_deg in angles_deg
    ]

    return _JOINT_STRUCT.pack(
        *raw_angles
    )


def build_joint_frame(
    command: int,
    angles_deg: list[float],
) -> bytes:
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


def parse_joint_payload(
    payload: bytes,
):
    if len(payload) != JOINT_PAYLOAD_LEN:
        return None

    raw_angles = _JOINT_STRUCT.unpack(
        payload
    )

    return [
        canonical_raw_to_degree(raw_angle)
        for raw_angle in raw_angles
    ]


def parse_joint_frame(
    frame: bytes,
):
    command, payload = parse_frame(
        frame
    )

    if command is None:
        return None, None

    if command not in _JOINT_COMMANDS:
        return command, None

    return (
        command,
        parse_joint_payload(payload),
    )


# ==========================================================
# Parameter Configuration
# ==========================================================

def build_set_parameter_frame(
    parameter_id: int,
    value: int,
) -> bytes:
    """
    构造：

        CMD_SET_PARAMETER

    Payload：

        Parameter ID : uint8
        Value        : uint32 little-endian
    """

    if not 0 <= parameter_id <= 0xFF:
        raise ValueError(
            "parameter_id 必须位于 [0, 255]"
        )

    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(
            "value 必须位于 uint32 范围"
        )

    payload = _PARAMETER_REQUEST_STRUCT.pack(
        parameter_id,
        value,
    )

    return build_frame(
        CMD_SET_PARAMETER,
        payload,
    )


def parse_parameter_ack_payload(
    payload: bytes,
):
    """
    成功返回：

        parameter_id,
        status,
        effective_value

    status 为 signed int8，
    与 MCU robot_status_t 对应。
    """

    if len(payload) != PARAMETER_ACK_PAYLOAD_LEN:
        return None

    return _PARAMETER_ACK_STRUCT.unpack(
        payload
    )


def parse_parameter_ack_frame(
    frame: bytes,
):
    command, payload = parse_frame(
        frame
    )

    if command != CMD_PARAMETER_ACK:
        return None

    return parse_parameter_ack_payload(
        payload
    )


# ==========================================================
# Diagnostics
# ==========================================================

def build_diagnostics_request(
    selector: Optional[int] = None,
) -> bytes:
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
        payload,
    )


def parse_diagnostics_response_payload(
    payload: bytes,
):
    if len(payload) != DIAGNOSTICS_PAYLOAD_LEN:
        return None

    return _DIAGNOSTICS_RESPONSE_STRUCT.unpack(
        payload
    )[0]


# ==========================================================
# Stream Frame Extraction
# ==========================================================

def extract_frames(
    buffer: bytearray,
) -> list[bytes]:
    """
    从 TCP 字节流中提取完整协议帧。

    Checksum 校验由 parse_frame() 完成。
    """

    frames = []

    while True:
        if len(buffer) < 2:
            break

        header_index = buffer.find(
            HEADER
        )

        if header_index < 0:
            # 保留末尾可能属于下一帧 Header 的 0xAA。
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
            del buffer[0]
            continue

        frame_length = (
            PROTOCOL_MIN_FRAME_LEN
            + payload_length
        )

        if len(buffer) < frame_length:
            break

        frames.append(
            bytes(buffer[:frame_length])
        )

        del buffer[:frame_length]

    return frames