import socket
import struct
import time


# ==========================================================
# TCP 配置
#
# QEMU 会把 Cortex-M4 的 UART0 映射到这个 TCP 端口。
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# 协议定义
# ==========================================================

FRAME_HEADER = b"\xAA\x55"

CMD_SET_JOINT_TARGETS = 0x01


# ==========================================================
# 解析一帧数据
# ==========================================================

def parse_frame(frame: bytes) -> None:

    # ----------------------------------------------
    # 1. 基本字段
    # ----------------------------------------------

    command = frame[2]
    payload_length = frame[3]

    payload = frame[
        4 : 4 + payload_length
    ]

    received_checksum = frame[-1]


    # ----------------------------------------------
    # 2. 计算 checksum
    #
    # 不包含 AA 55，
    # 对 Command + Length + Payload 求和并取低 8 位。
    # ----------------------------------------------

    calculated_checksum = (
        sum(frame[2:-1]) & 0xFF
    )


    if received_checksum != calculated_checksum:

        print(
            "校验失败:",
            f"received=0x{received_checksum:02X}",
            f"calculated=0x{calculated_checksum:02X}",
        )

        return


    # ----------------------------------------------
    # 3. 解析 SET_JOINT_TARGETS
    # ----------------------------------------------

    if command == CMD_SET_JOINT_TARGETS:

        if payload_length != 12:

            print(
                "Payload 长度错误:",
                payload_length
            )

            return


        # <  表示 little-endian
        # h  表示 signed int16
        # 6h 表示六个 int16
        raw_angles = struct.unpack(
            "<6h",
            payload
        )


        # 协议单位为 0.01°
        angles_deg = [
            value / 100.0
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
# 主程序
# ==========================================================

def main():

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )


    print(
        f"等待连接 QEMU UART..."
    )


    # QEMU 可能还没启动，因此循环尝试连接
    while True:

        try:

            sock.connect(
                (HOST, PORT)
            )

            break

        except ConnectionRefusedError:

            time.sleep(0.5)


    print(
        f"已连接到 QEMU UART：{HOST}:{PORT}"
    )


    # 用于处理 TCP 拆包 / 粘包
    buffer = bytearray()


    try:

        while True:

            data = sock.recv(1024)

            if not data:

                print(
                    "QEMU 已断开连接。"
                )

                break


            buffer.extend(data)


            # ------------------------------------------
            # 一个 recv() 可能收到：
            #
            # 半帧
            # 一帧
            # 多帧
            #
            # 所以必须使用缓冲区持续解析。
            # ------------------------------------------

            while True:

                # 找帧头 AA 55
                start = buffer.find(
                    FRAME_HEADER
                )


                if start < 0:

                    # 没找到有效帧头，
                    # 清空当前无效数据。
                    buffer.clear()
                    break


                # 丢弃帧头前面的无效数据
                if start > 0:

                    del buffer[:start]


                # 至少需要：
                # AA 55 CMD LEN
                if len(buffer) < 4:
                    break


                payload_length = buffer[3]


                # 总长度：
                #
                # Header 2
                # Command 1
                # Length 1
                # Payload N
                # Checksum 1
                total_length = (
                    2
                    + 1
                    + 1
                    + payload_length
                    + 1
                )


                if len(buffer) < total_length:
                    break


                # 取出完整一帧
                frame = bytes(
                    buffer[:total_length]
                )

                del buffer[:total_length]


                print(
                    "RX:",
                    frame.hex(" ").upper()
                )


                parse_frame(frame)


    except KeyboardInterrupt:

        print()
        print("接收程序停止。")


    finally:

        sock.close()


if __name__ == "__main__":
    main()