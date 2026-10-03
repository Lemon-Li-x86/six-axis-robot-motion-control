"""
文件：protocol_fault_test.py

用途：
测试 Cortex-M4 端 UART 协议字节流状态机的异常处理能力。

测试内容：
1. 正常 JOINT_STATE 帧；
2. 垃圾字节之后恢复正常解析；
3. Checksum 错误帧应被丢弃；
4. 非 12 Byte Payload 后能够继续解析正常帧；
5. Checksum 错误帧之后能够恢复；
6. 未知 Command 后能够继续解析正常帧。

说明：
本程序直接连接 QEMU UART TCP：

    127.0.0.1:5555

运行本程序时，不要同时运行：

    ur5_uart_bridge.py

因为 QEMU 当前 UART TCP 后端只使用一个连接。
"""

import select
import socket
import struct
import time


# ==========================================================
# 1. TCP 参数
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# 2. 协议参数
# ==========================================================

HEADER = b"\xAA\x55"

# Cortex-M4 -> Python
CMD_SET_JOINT_TARGETS = 0x01

# Python -> Cortex-M4
CMD_JOINT_STATE = 0x81

# Cortex-M4 -> Python
CMD_JOINT_STATE_ACK = 0x82

JOINT_COUNT = 6

JOINT_PAYLOAD_LEN = 12


# ==========================================================
# 3. 构造通用协议帧
# ==========================================================

def build_frame(
    command: int,
    payload: bytes
) -> bytes:
    """
    构造：

        AA 55
        Command
        Length
        Payload
        Checksum

    Checksum =
        Command + Length + Payload
        取最低 8 bit。
    """

    frame_without_checksum = (
        HEADER
        + bytes([
            command,
            len(payload)
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


# ==========================================================
# 4. 构造 JOINT_STATE 帧
# ==========================================================

def build_joint_state_frame(
    joints: list[int]
) -> tuple[bytes, bytes]:
    """
    根据六个 int16_t 原始关节值构造 JOINT_STATE。

    当前协议中：

        1 = 0.01°

    返回：

        完整协议帧
        Payload
    """

    if len(joints) != JOINT_COUNT:
        raise ValueError(
            "JOINT_STATE 必须包含 6 个关节值"
        )

    payload = struct.pack(
        "<6h",
        *joints
    )

    frame = build_frame(
        CMD_JOINT_STATE,
        payload
    )

    return frame, payload


# ==========================================================
# 5. 从连续字节流中提取协议帧
# ==========================================================

def extract_frames(
    buffer: bytearray
) -> list[bytes]:
    """
    从 TCP 连续字节流中提取完整协议帧。

    使用 Length 字段确定帧长度。
    """

    frames = []


    while True:

        # 至少需要：
        #
        # AA 55 CMD LEN
        #
        if len(buffer) < 4:
            break


        # --------------------------------------------------
        # 搜索 AA 55
        # --------------------------------------------------

        if buffer[0:2] != HEADER:

            del buffer[0]

            continue


        payload_length = buffer[3]


        frame_length = (
            2                  # Header
            + 1                # Command
            + 1                # Length
            + payload_length   # Payload
            + 1                # Checksum
        )


        # 当前数据还不是完整一帧。
        if len(buffer) < frame_length:
            break


        frame = bytes(
            buffer[:frame_length]
        )

        del buffer[:frame_length]

        frames.append(frame)


    return frames


# ==========================================================
# 6. 解析并验证协议帧
# ==========================================================

def parse_frame(
    frame: bytes
):
    """
    验证：

        Header
        Length
        Checksum

    成功：

        返回 command, payload

    失败：

        返回 None, None
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


    if (
        received_checksum
        != calculated_checksum
    ):
        return None, None


    return command, payload


# ==========================================================
# 7. TCP 接收器
# ==========================================================

class UARTReceiver:
    """
    保存持续的 TCP 接收缓冲区。

    不在每个测试之间直接清空半截协议帧，
    避免测试程序自己破坏串口字节流。
    """

    def __init__(
        self,
        sock: socket.socket
    ):
        self.sock = sock

        self.buffer = bytearray()


    def receive(
        self,
        timeout: float
    ) -> list[bytes]:
        """
        在 timeout 时间内接收所有完整协议帧。
        """

        frames = []

        end_time = (
            time.monotonic()
            + timeout
        )


        while (
            time.monotonic()
            < end_time
        ):

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
                    0.05
                )
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
        timeout: float = 0.2
    ):
        """
        等待并消费当前已经在路上的完整数据。

        主要用于测试之间清理上一项测试的返回结果。

        注意：
        不直接清空 buffer，
        因此不会人为切断半个协议帧。
        """

        self.receive(
            timeout
        )


# ==========================================================
# 8. 判断是否收到对应的 ACK
# ==========================================================

def has_matching_ack(
    frames: list[bytes],
    expected_payload: bytes
) -> bool:
    """
    判断收到的数据中是否存在：

        Command = 0x82

    并且 ACK Payload 与当前测试的
    JOINT_STATE Payload 完全一致。

    这样可以避免上一项测试的旧 ACK
    对当前测试造成误判。
    """

    for frame in frames:

        command, payload = parse_frame(
            frame
        )


        if (
            command
            == CMD_JOINT_STATE_ACK
            and payload
            == expected_payload
        ):
            return True


    return False


# ==========================================================
# 9. 打印 MCU 返回帧
# ==========================================================

def print_received_frames(
    frames: list[bytes]
):
    """
    调试时打印 MCU 实际返回的协议帧。
    """

    for frame in frames:

        command, payload = parse_frame(
            frame
        )


        if command is None:
            continue


        print(
            "    MCU RX:",
            frame.hex(" "),
            f" Command=0x{command:02X}"
        )


# ==========================================================
# 10. 打印测试结果
# ==========================================================

def print_result(
    name: str,
    passed: bool
):
    """
    统一打印 PASS / FAIL。
    """

    if passed:
        result = "PASS"
    else:
        result = "FAIL"


    print(
        f"[{result}] {name}"
    )


# ==========================================================
# 11. 主测试程序
# ==========================================================

def main():

    print(
        "正在连接 QEMU UART："
        f"{HOST}:{PORT}"
    )


    sock = socket.create_connection(
        (HOST, PORT)
    )


    receiver = UARTReceiver(
        sock
    )


    print("连接成功。")
    print()


    passed_count = 0

    total_count = 0


    try:

        # --------------------------------------------------
        # 等待 QEMU 当前可能已经发出的数据处理完。
        # --------------------------------------------------

        receiver.settle(
            0.2
        )


        # ==================================================
        # Test 1
        #
        # 正常 JOINT_STATE
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
                    106
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
        #
        # 垃圾字节 + 正常帧
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
                    206
                ]
            )
        )


        garbage = bytes([
            0x12,
            0x34,
            0x99,
            0xAB,
            0xCD
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
        #
        # 错误 Checksum
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
                    306
                ]
            )
        )


        bad_frame_3 = bytearray(
            bad_frame_3
        )


        # 故意破坏 Checksum。
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
        #
        # JOINT_STATE 使用 4 Byte Payload
        #
        # FSM 应该能够根据 Length 正常读完整帧。
        #
        # 但 application 层发现长度不是 12 Byte，
        # 因此不会处理它。
        #
        # 随后发送正常帧，
        # 应该仍然能够正常得到 ACK。
        # ==================================================

        total_count += 1


        short_payload = bytes([
            0x11,
            0x22,
            0x33,
            0x44
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
                    406
                ]
            )
        )


        sock.sendall(
            variable_length_frame
        )

        time.sleep(0.1)

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
        #
        # 错误 Checksum 帧 + 正常帧
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
                    506
                ]
            )
        )


        bad_frame_5 = bytearray(
            bad_frame_5
        )


        bad_frame_5[-1] ^= 0x55


        valid_frame_5, payload_5 = (
            build_joint_state_frame(
                [
                    511,
                    512,
                    513,
                    514,
                    515,
                    516
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
        #
        # 未知 Command + 正常帧
        #
        # FSM 本身应该能够解析未知 Command。
        #
        # 应用层不认识 0x90，因此忽略它。
        #
        # 后面的正常帧仍应正常处理。
        # ==================================================

        total_count += 1


        unknown_frame = build_frame(
            0x90,
            bytes([
                0x61,
                0x62,
                0x63,
                0x64
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
                    606
                ]
            )
        )


        sock.sendall(
            unknown_frame
        )

        time.sleep(0.1)

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
        # 最终测试结果
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


        if (
            passed_count
            == total_count
        ):

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