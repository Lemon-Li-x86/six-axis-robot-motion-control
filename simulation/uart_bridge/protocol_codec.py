"""
文件：protocol_codec.py

用途：
统一 Python 仿真与测试侧的 UART 二进制协议编解码。

本模块负责：

1. Generic Frame 构造与解析；
2. 六轴 Joint Frame 编解码；
3. Runtime Parameter Frame；
4. Diagnostics Frame；
5. TCP Byte Stream Frame 提取。
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

# 最大 Payload 长度与固件侧 PROTOCOL_MAX_PAYLOAD_LEN 一致。
PROTOCOL_MAX_PAYLOAD_LEN = 64

# Header(2) + Command(1) + Length(1) + Checksum(1)。
PROTOCOL_MIN_FRAME_LEN = 5


# ==========================================================
# Joint Protocol
# ==========================================================

JOINT_COUNT = 6

# 每个 Joint 使用 signed int16，共 2 Byte。
JOINT_PAYLOAD_LEN = JOINT_COUNT * 2

JOINT_FRAME_LEN = (
    2
    + 1
    + 1
    + JOINT_PAYLOAD_LEN
    + 1
)

# "<"  = Little Endian
# "6h" = 6 × signed int16
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

# Parameter ID(uint8) + Value(uint32)。
PARAMETER_REQUEST_PAYLOAD_LEN = 5

# Parameter ID(uint8) + Status(int8) + Effective Value(uint32)。
PARAMETER_ACK_PAYLOAD_LEN = 6

# Little Endian：
# B = uint8
# b = int8
# I = uint32
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

# Diagnostics Response 当前固定返回一个 uint32。
DIAGNOSTICS_PAYLOAD_LEN = 4

_DIAGNOSTICS_RESPONSE_STRUCT = struct.Struct("<I")


# ==========================================================
# Generic Frame
# ==========================================================

def build_frame(
    command: int,
    payload: bytes = b"",
) -> bytes:
    """
    构造通用协议帧。

    Args:
        command: 8 bit Command，范围 [0, 255]。
        payload: Payload Byte Sequence。

    Returns:
        完整二进制协议帧。

    Raises:
        ValueError:
            command 超出 uint8 范围，
            或 Payload 超过最大长度。

    Note:
        Frame Format：

        AA 55 | Command | Length | Payload | Checksum

        Checksum 为：

        Command + Length + Payload

        累加结果的低 8 bit。
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

    checksum = sum(body) & 0xFF

    return (
        HEADER
        + body
        + bytes([checksum])
    )


def parse_frame(
    frame: bytes,
):
    """
    校验并解析一个完整协议帧。

    Args:
        frame: 完整二进制协议帧。

    Returns:
        合法时返回：

        (command, payload)

        非法时返回：

        (None, None)
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

    return command, frame[4:-1]


# ==========================================================
# Joint Protocol
# ==========================================================

def build_joint_payload(
    angles_deg: list[float],
) -> bytes:
    """
    将六轴 degree 角度编码为 Joint Payload。

    Args:
        angles_deg:
            六个关节角，单位 degree。

    Returns:
        12 Byte Joint Payload。

    Raises:
        ValueError:
            输入关节数量不是 6。

    Note:
        每个关节最终使用 signed int16，
        单位为 0.01 degree。
    """
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
    """
    构造六轴 Joint Protocol Frame。

    Args:
        command:
            Joint Protocol Command。

        angles_deg:
            六个关节角，单位 degree。

    Returns:
        完整 Joint Frame。

    Raises:
        ValueError:
            command 不是 Joint Protocol Command，
            或关节数量非法。
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


def parse_joint_payload(
    payload: bytes,
):
    """
    解析六轴 Joint Payload。

    Args:
        payload: 12 Byte Joint Payload。

    Returns:
        合法时返回六个 degree 角度。

        长度非法时返回 None。
    """
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
    """
    解析 Joint Protocol Frame。

    Args:
        frame: 完整协议帧。

    Returns:
        合法 Joint Frame：

        (command, angles_deg)

        合法但非 Joint Command：

        (command, None)

        Frame 本身非法：

        (None, None)
    """
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
    构造 SET_PARAMETER Frame。

    Args:
        parameter_id:
            Parameter ID，范围 [0, 255]。

        value:
            Parameter Value，uint32。

    Returns:
        完整 CMD_SET_PARAMETER Frame。

    Raises:
        ValueError:
            parameter_id 或 value 超出协议范围。

    Note:
        Payload Layout：

        Parameter ID : uint8
        Value        : uint32 Little Endian
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
    解析 PARAMETER_ACK Payload。

    Args:
        payload: Parameter ACK Payload。

    Returns:
        合法时返回：

        (parameter_id, status, effective_value)

        长度非法时返回 None。

    Note:
        status 为 signed int8，
        与 MCU robot_status_t Wire Format 对应。
    """
    if len(payload) != PARAMETER_ACK_PAYLOAD_LEN:
        return None

    return _PARAMETER_ACK_STRUCT.unpack(
        payload
    )


def parse_parameter_ack_frame(
    frame: bytes,
):
    """
    解析完整 PARAMETER_ACK Frame。

    Args:
        frame: 完整协议帧。

    Returns:
        合法时返回：

        (parameter_id, status, effective_value)

        Command 或 Frame 非法时返回 None。
    """
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
    """
    构造 Diagnostics Request。

    Args:
        selector:
            Diagnostics Metric Selector。

            为 None 时发送空 Payload，
            由固件使用默认 Selector。

    Returns:
        完整 CMD_GET_DIAGNOSTICS Frame。

    Raises:
        ValueError:
            selector 超出 uint8 范围。
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
        payload,
    )


def parse_diagnostics_response_payload(
    payload: bytes,
):
    """
    解析 Diagnostics Response Payload。

    Args:
        payload:
            固定 4 Byte uint32 Payload。

    Returns:
        合法时返回 uint32 Value。

        长度非法时返回 None。
    """
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
    从 TCP Byte Stream Buffer 中提取完整协议帧。

    Args:
        buffer:
            可变 Byte Buffer。

            已成功提取的 Byte 会从该 Buffer 删除。

    Returns:
        当前可以完整提取出的 Frame List。

    Note:
        本函数负责：

        1. Header 同步；
        2. Payload Length 边界；
        3. TCP 拆包 / 粘包处理。

        Checksum 校验继续由 parse_frame() 完成。
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

        # 丢弃有效 Header 之前的噪声 Byte。
        if header_index > 0:
            del buffer[:header_index]

        # 至少需要 Header + Command + Length。
        if len(buffer) < 4:
            break

        payload_length = buffer[3]

        if payload_length > PROTOCOL_MAX_PAYLOAD_LEN:
            # 当前 Header 不可能形成合法 Frame。
            # 删除一个 Byte 后重新寻找 Header。
            del buffer[0]
            continue

        frame_length = (
            PROTOCOL_MIN_FRAME_LEN
            + payload_length
        )

        # TCP Buffer 中尚未收到完整 Frame。
        if len(buffer) < frame_length:
            break

        frames.append(
            bytes(buffer[:frame_length])
        )

        del buffer[:frame_length]

    return frames