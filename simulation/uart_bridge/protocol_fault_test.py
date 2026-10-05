"""
文件：protocol_fault_test.py

用途：
验证 Cortex-M4 UART Protocol Byte Stream FSM
面对异常输入时的恢复能力。

测试内容：

1. 正常 JOINT_STATE；
2. Garbage Byte + 正常 Frame；
3. 错误 Checksum；
4. 非法 JOINT_STATE Payload Length + 正常 Frame；
5. 错误 Checksum Frame + 正常 Frame；
6. Unknown Command + 正常 Frame。
"""

import select
import socket
import struct
import time


# ==========================================================
# TCP Configuration
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# Protocol Configuration
# ==========================================================

HEADER = b"\xAA\x55"

# Cortex-M4 -> Python。
CMD_SET_JOINT_TARGETS = 0x01

# Python -> Cortex-M4。
CMD_JOINT_STATE = 0x81

# Cortex-M4 -> Python。
CMD_JOINT_STATE_ACK = 0x82

JOINT_COUNT = 6

# 6 × signed int16 = 12 Byte。
JOINT_PAYLOAD_LEN = 12


# ==========================================================
# Protocol Helpers
# ==========================================================

def build_frame(
    command: int,
    payload: bytes,
) -> bytes:
    """
    构造通用 Protocol Frame。

    Args:
        command:
            8 bit Command。

        payload:
            Payload Byte Sequence。

    Returns:
        完整协议帧。

    Note:
        Frame：

        AA 55 | Command | Length | Payload | Checksum

        Checksum：

        Command + Length + Payload

        累加结果取低 8 bit。
    """
    frame_without_checksum = (
        HEADER
        + bytes([
            command,
            len(payload),
        ])
        + payload
    )

    checksum = (
        sum(frame_without_checksum[2:])
        & 0xFF
    )

    return (
        frame_without_checksum
        + bytes([checksum])
    )


def build_joint_state_frame(
    joints: list[int],
) -> tuple[bytes, bytes]:
    """
    根据六轴 Raw Joint Value 构造 JOINT_STATE。

    Args:
        joints:
            六个 signed int16 Raw Angle。

            单位：
            0.01 degree。

    Returns:
        Tuple：

        (
            complete_frame,
            payload,
        )

    Raises:
        ValueError:
            Joint Count 不是 6。
    """
    if len(joints) != JOINT_COUNT:
        raise ValueError(
            "JOINT_STATE 必须包含 6 个关节值"
        )

    # "<6h"：
    #
    # Little Endian
    # 6 × signed int16。
    payload = struct.pack(
        "<6h",
        *joints,
    )

    frame = build_frame(
        CMD_JOINT_STATE,
        payload,
    )

    return frame, payload


def extract_frames(
    buffer: bytearray,
) -> list[bytes]:
    """
    从 TCP Byte Stream 提取完整协议帧。

    Args:
        buffer:
            持续接收 Buffer。

    Returns:
        当前能够完整提取的 Frame List。

    Note:
        使用 Length Field 决定 Frame Size。

        Header 不匹配时每次删除一个 Byte，
        用于重新同步到后续 AA 55。
    """
    frames = []

    while True:
        # 至少需要：
        #
        # AA 55 CMD LEN
        if len(buffer) < 4:
            break

        if buffer[0:2] != HEADER:
            del buffer[0]
            continue

        payload_length = buffer[3]

        frame_length = (
            2
            + 1
            + 1
            + payload_length
            + 1
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


def parse_frame(
    frame: bytes,
):
    """
    校验并解析完整 Protocol Frame。

    Args:
        frame:
            完整协议帧。

    Returns:
        合法时：

        (
            command,
            payload,
        )

        Header、Length 或 Checksum 非法时：

        (
            None,
            None,
        )
    """
    if len(frame) < 5:
        return None, None

    if frame[0:2] != HEADER:
        return None, None

    command = frame[2]
    payload_length = frame[3]

    expected_length = (
        2
        + 1
        + 1
        + payload_length
        + 1
    )

    if len(frame) != expected_length:
        return None, None

    payload = frame[4:-1]

    received_checksum = frame[-1]

    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )

    if received_checksum != calculated_checksum:
        return None, None

    return command, payload


# ==========================================================
# TCP Receiver
# ==========================================================

class UARTReceiver:
    """
    管理跨 Test Case 持续存在的 TCP RX Buffer。

    不在不同测试之间直接清空 Buffer，
    避免人为丢弃半个 Protocol Frame，
    反而由测试程序自己制造解析错误。
    """

    def __init__(
        self,
        sock: socket.socket,
    ):
        """
        初始化 UART Receiver。

        Args:
            sock:
                已连接 QEMU UART TCP Socket。
        """
        self.sock = sock
        self.buffer = bytearray()

    def receive(
        self,
        timeout: float,
    ) -> list[bytes]:
        """
        在指定时间窗口内接收完整协议帧。

        Args:
            timeout:
                最大接收时间，单位 second。

        Returns:
            当前时间窗口收到的完整 Frame List。
        """
        frames = []

        end_time = (
            time.monotonic()
            + timeout
        )

        while time.monotonic() < end_time:
            remaining = (
                end_time
                - time.monotonic()
            )

            readable, _, _ = select.select(
                [self.sock],
                [],
                [],
                min(
                    remaining,
                    0.05,
                ),
            )

            if not readable:
                continue

            data = self.sock.recv(
                1024
            )

            if not data:
                break

            self.buffer.extend(
                data
            )

            new_frames = extract_frames(
                self.buffer
            )

            frames.extend(
                new_frames
            )

        return frames

    def settle(
        self,
        timeout: float = 0.2,
    ) -> None:
        """
        消费测试之间仍然在链路中的旧数据。

        Args:
            timeout:
                Settle 时间窗口，单位 second。

        Note:
            不直接 clear() RX Buffer，
            防止人为截断半个 Protocol Frame。
        """
        self.receive(
            timeout
        )


# ==========================================================
# ACK Matching
# ==========================================================

def has_matching_ack(
    frames: list[bytes],
    expected_payload: bytes,
) -> bool:
    """
    判断 Frame List 中是否存在当前测试对应的 ACK。

    Args:
        frames:
            收到的完整 Frame List。

        expected_payload:
            当前测试 JOINT_STATE Payload。

    Returns:
        True：
        存在 Command = 0x82，
        且 Payload 完全一致的 Frame。

        False：
        没有匹配 ACK。

    Note:
        Payload 也必须一致，
        防止上一 Test Case 的旧 ACK
        被错误认为当前测试成功。
    """
    for frame in frames:
        command, payload = parse_frame(
            frame
        )

        if (
            command == CMD_JOINT_STATE_ACK
            and payload == expected_payload
        ):
            return True

    return False


# ==========================================================
# Debug Output
# ==========================================================

def print_received_frames(
    frames: list[bytes],
) -> None:
    """
    打印收到的合法 MCU Protocol Frame。

    Args:
        frames:
            Frame List。
    """
    for frame in frames:
        command, _ = parse_frame(
            frame
        )

        if command is None:
            continue

        print(
            "    MCU RX:",
            frame.hex(" "),
            f" Command=0x{command:02X}"
        )


def print_result(
    name: str,
    passed: bool,
) -> None:
    """
    统一输出一个 Fault Test Case 的结果。

    Args:
        name:
            Test Case 描述。

        passed:
            Test Result。
    """
    result = (
        "PASS"
        if passed
        else "FAIL"
    )

    print(
        f"[{result}] {name}"
    )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
    """
    顺序执行六个 Protocol Fault Recovery Test。
    """
    print(
        "正在连接 QEMU UART："
        f"{HOST}:{PORT}"
    )

    sock = socket.create_connection(
        (
            HOST,
            PORT,
        )
    )

    receiver = UARTReceiver(
        sock
    )

    print("连接成功。")
    print()

    passed_count = 0
    total_count = 0

    try:
        # 清理 QEMU 在测试启动前
        # 已经发送到 TCP 链路的数据。
        receiver.settle(
            0.2
        )

        # ==================================================
        # Test 1
        # Valid JOINT_STATE
        # ==================================================

        total_count += 1

        valid_frame_1, payload_1 = (
            build_joint_state_frame(
                [
                    101,
                    102,
                    103,
                    104,
                    105,
                    106,
                ]
            )
        )

        sock.sendall(
            valid_frame_1
        )

        frames = receiver.receive(
            0.5
        )

        passed = has_matching_ack(
            frames,
            payload_1
        )

        print_result(
            "正常 JOINT_STATE -> 应收到对应 0x82 ACK",
            passed
        )

        if not passed:
            print_received_frames(
                frames
            )

        if passed:
            passed_count += 1

        receiver.settle()

        # ==================================================
        # Test 2
        # Garbage + Valid Frame
        # ==================================================

        total_count += 1

        valid_frame_2, payload_2 = (
            build_joint_state_frame(
                [
                    201,
                    202,
                    203,
                    204,
                    205,
                    206,
                ]
            )
        )

        # 人为插入不包含合法 AA 55 Header 的垃圾数据。
        garbage = bytes([
            0x12,
            0x34,
            0x99,
            0xAB,
            0xCD,
        ])

        sock.sendall(
            garbage
            + valid_frame_2
        )

        frames = receiver.receive(
            0.5
        )

        passed = has_matching_ack(
            frames,
            payload_2
        )

        print_result(
            "垃圾字节后接正常帧 -> 应恢复并收到 ACK",
            passed
        )

        if not passed:
            print_received_frames(
                frames
            )

        if passed:
            passed_count += 1

        receiver.settle()

        # ==================================================
        # Test 3
        # Bad Checksum
        # ==================================================

        total_count += 1

        bad_frame_3, payload_3 = (
            build_joint_state_frame(
                [
                    301,
                    302,
                    303,
                    304,
                    305,
                    306,
                ]
            )
        )

        bad_frame_3 = bytearray(
            bad_frame_3
        )

        # 翻转 Checksum Byte，
        # 确保 Frame 校验失败。
        bad_frame_3[-1] ^= 0xFF

        sock.sendall(
            bad_frame_3
        )

        frames = receiver.receive(
            0.5
        )

        passed = (
            not has_matching_ack(
                frames,
                payload_3
            )
        )

        print_result(
            "错误 Checksum -> 不应收到对应 ACK",
            passed
        )

        if not passed:
            print_received_frames(
                frames
            )

        if passed:
            passed_count += 1

        receiver.settle()

        # ==================================================
        # Test 4
        # Invalid Application Payload Length
        # ==================================================

        total_count += 1

        # Protocol FSM 可以根据 Length = 4
        # 正常读取完整 Frame。
        #
        # 但 JOINT_STATE Application Contract
        # 要求 Payload 为 12 Byte，
        # 因此该 Frame 应被业务层忽略。
        short_payload = bytes([
            0x11,
            0x22,
            0x33,
            0x44,
        ])

        variable_length_frame = build_frame(
            CMD_JOINT_STATE,
            short_payload
        )

        valid_frame_4, payload_4 = (
            build_joint_state_frame(
                [
                    401,
                    402,
                    403,
                    404,
                    405,
                    406,
                ]
            )
        )

        sock.sendall(
            variable_length_frame
        )

        # 给 MCU 少量时间处理前一个非法业务 Frame。
        time.sleep(
            0.1
        )

        sock.sendall(
            valid_frame_4
        )

        frames = receiver.receive(
            0.8
        )

        passed = has_matching_ack(
            frames,
            payload_4
        )

        print_result(
            "4 Byte Payload 后接正常帧 -> 应保持同步",
            passed
        )

        if not passed:
            print(
                "    发送的变长帧：",
                variable_length_frame.hex(" ")
            )

            print(
                "    后续正常帧：",
                valid_frame_4.hex(" ")
            )

            print_received_frames(
                frames
            )

        if passed:
            passed_count += 1

        receiver.settle()

        # ==================================================
        # Test 5
        # Bad Checksum + Valid Frame
        # ==================================================

        total_count += 1

        bad_frame_5, _ = (
            build_joint_state_frame(
                [
                    501,
                    502,
                    503,
                    504,
                    505,
                    506,
                ]
            )
        )

        bad_frame_5 = bytearray(
            bad_frame_5
        )

        # 使用不同 Pattern 破坏 Checksum，
        # 验证 Parser 不是只对某一个错误值恢复。
        bad_frame_5[-1] ^= 0x55

        valid_frame_5, payload_5 = (
            build_joint_state_frame(
                [
                    511,
                    512,
                    513,
                    514,
                    515,
                    516,
                ]
            )
        )

        sock.sendall(
            bad_frame_5
            + valid_frame_5
        )

        frames = receiver.receive(
            0.8
        )

        passed = has_matching_ack(
            frames,
            payload_5
        )

        print_result(
            "错误帧后接正常帧 -> 应恢复并收到 ACK",
            passed
        )

        if not passed:
            print_received_frames(
                frames
            )

        if passed:
            passed_count += 1

        receiver.settle()

        # ==================================================
        # Test 6
        # Unknown Command + Valid Frame
        # ==================================================

        total_count += 1

        # FSM 本身应能够完整解析 Unknown Command。
        #
        # Application 不认识 0x90，
        # 因此应忽略该 Frame，
        # 但 Parser 必须继续处理后续合法 Frame。
        unknown_frame = build_frame(
            0x90,
            bytes([
                0x61,
                0x62,
                0x63,
                0x64,
            ])
        )

        valid_frame_6, payload_6 = (
            build_joint_state_frame(
                [
                    601,
                    602,
                    603,
                    604,
                    605,
                    606,
                ]
            )
        )

        sock.sendall(
            unknown_frame
        )

        time.sleep(
            0.1
        )

        sock.sendall(
            valid_frame_6
        )

        frames = receiver.receive(
            0.8
        )

        passed = has_matching_ack(
            frames,
            payload_6
        )

        print_result(
            "未知 Command 后接正常帧 -> 应保持同步",
            passed
        )

        if not passed:
            print_received_frames(
                frames
            )

        if passed:
            passed_count += 1

        # ==================================================
        # Final Result
        # ==================================================

        print()
        print(
            "======================================"
        )

        print(
            f"测试结果："
            f"{passed_count}/{total_count} 通过"
        )

        print(
            "======================================"
        )

        if passed_count == total_count:
            print(
                "协议字节流状态机基础异常测试通过。"
            )

        else:
            print(
                "存在失败测试，需要继续检查协议状态机。"
            )

    finally:
        sock.close()


if __name__ == "__main__":
    main()