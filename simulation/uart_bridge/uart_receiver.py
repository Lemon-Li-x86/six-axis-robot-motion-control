"""
文件：uart_receiver.py

用途：
通过 TCP 连接 QEMU UART0，
接收并解析 Cortex-M4 发出的六轴目标角协议帧。

本脚本主要用于早期 UART 通信链路人工验证。
"""

import socket
import struct
import time


# ==========================================================
# TCP Configuration
# ==========================================================

# QEMU UART0 TCP Endpoint。
HOST = "127.0.0.1"
PORT = 5555

# QEMU 尚未启动时的重连间隔，单位 second。
RECONNECT_DELAY_S = 0.5

# 单次 socket.recv() 最大读取长度，单位 Byte。
SOCKET_RECV_SIZE = 1024


# ==========================================================
# Protocol Constants
# ==========================================================

HEADER = b"\xAA\x55"

CMD_SET_JOINT_TARGETS = 0x01

JOINT_COUNT = 6

# 6 × signed int16 = 12 Byte。
JOINT_PAYLOAD_LEN = JOINT_COUNT * 2

# Header(2) + Command(1) + Length(1) + Checksum(1)。
PROTOCOL_MIN_FRAME_LEN = 5

# Joint Raw Angle：
# 1 unit = 0.01 degree。
JOINT_ANGLE_SCALE = 100.0


# ==========================================================
# Frame Parser
# ==========================================================

def parse_frame(frame: bytes) -> None:
    """
    校验并打印一个完整 UART 协议帧。

    Args:
        frame: 完整协议帧。

    Note:
        当前脚本只处理 CMD_SET_JOINT_TARGETS。
    """
    if len(frame) < PROTOCOL_MIN_FRAME_LEN:
        return

    if frame[0:2] != HEADER:
        return

    command = frame[2]
    payload_length = frame[3]

    payload = frame[4:-1]
    received_checksum = frame[-1]

    # Checksum：
    # Command + Length + Payload，
    # 最终保留低 8 bit。
    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )

    if received_checksum != calculated_checksum:
        print(
            "校验失败:",
            frame.hex(" ")
        )

        return

    if (
        command == CMD_SET_JOINT_TARGETS
        and payload_length == JOINT_PAYLOAD_LEN
    ):
        # "<"  = Little Endian
        # "6h" = 6 × signed int16
        raw_angles = struct.unpack(
            "<6h",
            payload
        )

        angles_deg = [
            value / JOINT_ANGLE_SCALE
            for value in raw_angles
        ]

        print(
            "收到目标角:",
            angles_deg
        )


# ==========================================================
# TCP Receiver
# ==========================================================

def main() -> None:
    """
    连接 QEMU UART TCP Server 并持续接收协议帧。

    QEMU 尚未启动时持续重试连接。

    TCP 是 Byte Stream，
    因此使用本地 Buffer 处理拆包和粘包。
    """
    print(
        f"正在连接 QEMU UART: "
        f"{HOST}:{PORT}"
    )

    while True:
        try:
            sock = socket.create_connection(
                (HOST, PORT)
            )

            break

        except ConnectionRefusedError:
            print(
                "QEMU 尚未监听，"
                f"{RECONNECT_DELAY_S} 秒后重试..."
            )

            time.sleep(
                RECONNECT_DELAY_S
            )

    print("已连接到 QEMU UART。")

    # TCP recv() 不保证一次正好返回一个 Frame。
    buffer = bytearray()

    try:
        while True:
            data = sock.recv(
                SOCKET_RECV_SIZE
            )

            if not data:
                print("QEMU 已断开连接。")
                break

            buffer.extend(data)

            while True:
                # 至少需要：
                # Header + Command + Length。
                if len(buffer) < 4:
                    break

                # 当前 Byte 不属于合法 Header，
                # 删除一个 Byte 后重新同步。
                if buffer[0:2] != HEADER:
                    del buffer[0]
                    continue

                payload_length = buffer[3]

                # Frame：
                #
                # Header   2 Byte
                # Command  1 Byte
                # Length   1 Byte
                # Payload  N Byte
                # Checksum 1 Byte
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

                print(
                    "RX:",
                    frame.hex(" ")
                )

                parse_frame(
                    frame
                )

    except KeyboardInterrupt:
        print()
        print("接收程序停止。")

    finally:
        sock.close()


if __name__ == "__main__":
    main()