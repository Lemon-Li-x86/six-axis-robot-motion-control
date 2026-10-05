"""
文件：qemu_uart_receiver.py

用途：
通过 TCP 连接 QEMU UART0，
接收并解析 Cortex-M4 发送的六轴目标角 Frame。

本脚本位于 PyBullet 仿真目录，
主要用于验证 QEMU UART 输出和
Python 仿真侧协议解析链路。
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

# QEMU 尚未启动时的重连周期，单位 second。
RECONNECT_DELAY_S = 0.5

# 单次 TCP recv() 最大读取长度，单位 Byte。
SOCKET_RECV_SIZE = 1024


# ==========================================================
# Protocol Definition
# ==========================================================

FRAME_HEADER = b"\xAA\x55"

CMD_SET_JOINT_TARGETS = 0x01

JOINT_COUNT = 6

# 每个关节使用 signed int16。
JOINT_PAYLOAD_LEN = JOINT_COUNT * 2

# UART Raw Joint Angle：
# 1 unit = 0.01 degree。
JOINT_ANGLE_SCALE = 100.0


# ==========================================================
# Frame Parser
# ==========================================================

def parse_frame(frame: bytes) -> None:
    """
    校验并解析一个 SET_JOINT_TARGETS Frame。

    Args:
        frame: 完整 UART 协议帧。

    Note:
        Checksum 不包含 AA 55 Header，
        只计算 Command + Length + Payload。
    """
    command = frame[2]
    payload_length = frame[3]

    payload = frame[
        4:4 + payload_length
    ]

    received_checksum = frame[-1]

    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )

    if received_checksum != calculated_checksum:
        print(
            "校验失败:",
            f"received=0x{received_checksum:02X}",
            f"calculated=0x{calculated_checksum:02X}",
        )

        return

    if command == CMD_SET_JOINT_TARGETS:
        if payload_length != JOINT_PAYLOAD_LEN:
            print(
                "Payload 长度错误:",
                payload_length
            )

            return

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
            "收到六轴目标角:",
            angles_deg
        )

    else:
        print(
            f"未知命令: 0x{command:02X}"
        )


# ==========================================================
# TCP Receiver
# ==========================================================

def main() -> None:
    """
    持续连接 QEMU UART TCP Server 并解析数据流。

    TCP 可能出现拆包和粘包，
    因此所有数据先进入 bytearray Buffer，
    再按协议 Frame Length 提取。
    """
    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    print("等待连接 QEMU UART...")

    while True:
        try:
            sock.connect(
                (HOST, PORT)
            )

            break

        except ConnectionRefusedError:
            time.sleep(
                RECONNECT_DELAY_S
            )

    print(
        f"已连接到 QEMU UART："
        f"{HOST}:{PORT}"
    )

    buffer = bytearray()

    try:
        while True:
            data = sock.recv(
                SOCKET_RECV_SIZE
            )

            if not data:
                print(
                    "QEMU 已断开连接。"
                )

                break

            buffer.extend(data)

            # 一个 recv() 可能包含：
            #
            # 1. 半个 Frame；
            # 2. 一个完整 Frame；
            # 3. 多个连续 Frame。
            while True:
                start = buffer.find(
                    FRAME_HEADER
                )

                if start < 0:
                    # 当前 Buffer 中没有有效 Header。
                    buffer.clear()
                    break

                if start > 0:
                    # 丢弃 Header 之前的无效数据。
                    del buffer[:start]

                # 至少需要：
                # Header(2) + Command(1) + Length(1)。
                if len(buffer) < 4:
                    break

                payload_length = buffer[3]

                total_length = (
                    2
                    + 1
                    + 1
                    + payload_length
                    + 1
                )

                if len(buffer) < total_length:
                    break

                frame = bytes(
                    buffer[:total_length]
                )

                del buffer[:total_length]

                print(
                    "RX:",
                    frame.hex(" ").upper()
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