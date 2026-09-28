import socket
import struct
import time


# ==========================================================
# 协议常量
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555

HEADER = b"\xAA\x55"

CMD_SET_JOINT_TARGETS = 0x01

JOINT_COUNT = 6
JOINT_PAYLOAD_LEN = 12


# ==========================================================
# 解析一个完整数据帧
# ==========================================================

def parse_frame(frame: bytes):

    # ------------------------------------------------------
    # 1. 基本长度检查
    # ------------------------------------------------------

    if len(frame) < 5:
        return


    # ------------------------------------------------------
    # 2. 帧头检查
    # ------------------------------------------------------

    if frame[0:2] != HEADER:
        return


    command = frame[2]
    payload_length = frame[3]

    payload = frame[4:-1]
    received_checksum = frame[-1]


    # ------------------------------------------------------
    # 3. Checksum
    # ------------------------------------------------------

    calculated_checksum = (
        sum(frame[2:-1]) & 0xFF
    )

    if received_checksum != calculated_checksum:

        print(
            "校验失败:",
            frame.hex(" ")
        )

        return


    # ------------------------------------------------------
    # 4. 解析六轴目标角
    # ------------------------------------------------------

    if (
        command == CMD_SET_JOINT_TARGETS
        and payload_length == JOINT_PAYLOAD_LEN
    ):

        # <  : little-endian
        # 6h : 六个 int16
        raw_angles = struct.unpack(
            "<6h",
            payload
        )

        # int16 单位是 0.01°
        angles_deg = [
            value / 100.0
            for value in raw_angles
        ]

        print(
            "收到目标角:",
            angles_deg
        )


# ==========================================================
# TCP 接收主程序
# ==========================================================

def main():

    print(
        f"正在连接 QEMU UART: "
        f"{HOST}:{PORT}"
    )


    # ------------------------------------------------------
    # 如果 QEMU 还没启动，就持续尝试连接
    # ------------------------------------------------------

    while True:

        try:

            sock = socket.create_connection(
                (HOST, PORT)
            )

            break

        except ConnectionRefusedError:

            print(
                "QEMU 尚未监听，0.5 秒后重试..."
            )

            time.sleep(0.5)


    print("已连接到 QEMU UART。")


    # TCP 是字节流，
    # 一次 recv() 不保证正好收到一整个 frame。
    buffer = bytearray()


    try:

        while True:

            data = sock.recv(1024)

            if not data:
                print("QEMU 已断开连接。")
                break


            buffer.extend(data)


            # ------------------------------------------------
            # 从字节流中不断寻找 AA 55 帧头
            # ------------------------------------------------

            while True:

                # 至少需要 Header + Command + Length
                if len(buffer) < 4:
                    break


                # 如果开头不是 AA 55，
                # 丢掉一个字节并重新同步。
                if buffer[0:2] != HEADER:

                    del buffer[0]

                    continue


                payload_length = buffer[3]

                # 2 Header
                # 1 Command
                # 1 Length
                # N Payload
                # 1 Checksum
                frame_length = (
                    2
                    + 1
                    + 1
                    + payload_length
                    + 1
                )


                # 当前还没有收到完整帧
                if len(buffer) < frame_length:
                    break


                # 提取一个完整帧
                frame = bytes(
                    buffer[:frame_length]
                )

                del buffer[:frame_length]


                print(
                    "RX:",
                    frame.hex(" ")
                )


                parse_frame(frame)


    except KeyboardInterrupt:

        print()
        print("接收程序停止。")


    finally:

        sock.close()


if __name__ == "__main__":
    main()