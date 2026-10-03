"""
文件：burst_stress_test.py

用途：
对当前 QEMU Cortex-M4 UART 接收链路进行 Burst 压力测试。

测试方式：

1. 构造大量完整 JOINT_STATE 帧；
2. 将所有帧拼接成一个大 bytes；
3. 使用一次 sendall() 尽可能快地写入 QEMU TCP UART；
4. 统计最终收到多少匹配的 JOINT_STATE_ACK；
5. 测试前后查询 MCU Diagnostics；
6. 计算 UART Driver Ring Buffer 的新增丢字节数量。

重点观察：

ACK Lost
RX Drop Bytes

其中：

RX Drop Bytes

表示 UART ISR 因 Ring Buffer 已满而无法写入，
导致丢弃的字节数量。

它不是“丢帧数”。

本测试属于：

QEMU + TCP + FreeRTOS

仿真环境压力测试，
不等同于真实 Cortex-M4 硬件吞吐能力。
"""

import select
import socket
import struct
import time


# ==========================================================
# TCP
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# Protocol
# ==========================================================

HEADER = b"\xAA\x55"

CMD_JOINT_STATE = 0x81
CMD_JOINT_STATE_ACK = 0x82

CMD_GET_DIAGNOSTICS = 0x83
CMD_DIAGNOSTICS_RESPONSE = 0x84


# ==========================================================
# Burst 配置
# ==========================================================

BURST_FRAME_COUNTS = [
    10,
    100,
    500,
    1000,
    5000,
]


ACK_TIMEOUT = 10.0

TEST_GAP = 0.5


# ==========================================================
# Protocol Frame Builder
# ==========================================================

def build_frame(
    command: int,
    payload: bytes
) -> bytes:

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
# JOINT_STATE Builder
# ==========================================================

def build_joint_state_frame(
    sequence: int
) -> tuple[bytes, bytes]:

    value = (
        sequence % 30000
    )


    joints = [
        value,
        value + 1,
        value + 2,
        value + 3,
        value + 4,
        value + 5,
    ]


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
# Diagnostics Request
# ==========================================================

def build_diagnostics_request() -> bytes:

    return build_frame(
        CMD_GET_DIAGNOSTICS,
        b""
    )


# ==========================================================
# Extract Frames
# ==========================================================

def extract_frames(
    buffer: bytearray
) -> list[bytes]:

    frames = []


    while True:

        if len(buffer) < 4:
            break


        if buffer[0:2] != HEADER:

            del buffer[0]

            continue


        payload_length = (
            buffer[3]
        )


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


# ==========================================================
# Parse Frame
# ==========================================================

def parse_frame(
    frame: bytes
):

    if len(frame) < 5:
        return None, None


    if frame[0:2] != HEADER:
        return None, None


    command = (
        frame[2]
    )


    payload_length = (
        frame[3]
    )


    expected_length = (
        2
        + 1
        + 1
        + payload_length
        + 1
    )


    if len(frame) != expected_length:
        return None, None


    received_checksum = (
        frame[-1]
    )


    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )


    if (
        received_checksum
        != calculated_checksum
    ):
        return None, None


    return (
        command,
        frame[4:-1]
    )


# ==========================================================
# 清理当前已经在 TCP 链路中的数据
# ==========================================================

def settle_socket(
    sock: socket.socket,
    rx_buffer: bytearray,
    timeout: float = 0.2
):

    deadline = (
        time.perf_counter()
        + timeout
    )


    while (
        time.perf_counter()
        < deadline
    ):

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            0.01
        )


        if not readable:
            continue


        data = sock.recv(
            4096
        )


        if not data:
            return


        rx_buffer.extend(
            data
        )


        extract_frames(
            rx_buffer
        )


# ==========================================================
# 查询 MCU Diagnostics
# ==========================================================

def query_rx_drop_count(
    sock: socket.socket,
    rx_buffer: bytearray
) -> int:

    request = (
        build_diagnostics_request()
    )


    # 如果链路刚结束高负载，
    # 允许最多尝试 3 次查询。
    for _ in range(3):

        sock.sendall(
            request
        )


        deadline = (
            time.perf_counter()
            + 1.0
        )


        while (
            time.perf_counter()
            < deadline
        ):

            remaining = (
                deadline
                - time.perf_counter()
            )


            readable, _, _ = select.select(
                [sock],
                [],
                [],
                min(
                    remaining,
                    0.05
                )
            )


            if not readable:
                continue


            data = sock.recv(
                4096
            )


            if not data:
                break


            rx_buffer.extend(
                data
            )


            frames = extract_frames(
                rx_buffer
            )


            for frame in frames:

                command, payload = (
                    parse_frame(
                        frame
                    )
                )


                if (
                    command
                    == CMD_DIAGNOSTICS_RESPONSE
                    and payload is not None
                    and len(payload) == 4
                ):
                    return struct.unpack(
                        "<I",
                        payload
                    )[0]


    raise RuntimeError(
        "无法获得 MCU Diagnostics Response"
    )


# ==========================================================
# 收集 Burst 对应 ACK
# ==========================================================

def collect_burst_acks(
    sock: socket.socket,
    rx_buffer: bytearray,
    expected_payloads: set[bytes]
) -> set[bytes]:

    received_payloads = set()


    deadline = (
        time.perf_counter()
        + ACK_TIMEOUT
    )


    while (
        time.perf_counter()
        < deadline
    ):

        if (
            expected_payloads
            <= received_payloads
        ):
            break


        remaining = (
            deadline
            - time.perf_counter()
        )


        readable, _, _ = select.select(
            [sock],
            [],
            [],
            min(
                remaining,
                0.01
            )
        )


        if not readable:
            continue


        data = sock.recv(
            16384
        )


        if not data:
            break


        rx_buffer.extend(
            data
        )


        frames = extract_frames(
            rx_buffer
        )


        for frame in frames:

            command, payload = (
                parse_frame(
                    frame
                )
            )


            if (
                command
                == CMD_JOINT_STATE_ACK
                and payload is not None
            ):
                received_payloads.add(
                    payload
                )


    return received_payloads


# ==========================================================
# 单个 Burst Test
# ==========================================================

def run_burst_test(
    sock: socket.socket,
    rx_buffer: bytearray,
    frame_count: int,
    sequence_start: int
):

    settle_socket(
        sock,
        rx_buffer
    )


    drop_before = (
        query_rx_drop_count(
            sock,
            rx_buffer
        )
    )


    frames = []

    expected_payloads = set()


    for i in range(
        frame_count
    ):

        frame, payload = (
            build_joint_state_frame(
                sequence_start + i
            )
        )


        frames.append(
            frame
        )


        expected_payloads.add(
            payload
        )


    # ------------------------------------------------------
    # 拼成单个连续 Burst。
    # ------------------------------------------------------

    burst_data = b"".join(
        frames
    )


    burst_bytes = len(
        burst_data
    )


    # ------------------------------------------------------
    # 一次 sendall()
    # ------------------------------------------------------

    send_start = (
        time.perf_counter()
    )


    sock.sendall(
        burst_data
    )


    send_end = (
        time.perf_counter()
    )


    send_elapsed = (
        send_end
        - send_start
    )


    if send_elapsed > 0:

        host_push_rate = (
            frame_count
            / send_elapsed
        )

    else:

        host_push_rate = 0.0


    # ------------------------------------------------------
    # 收集 ACK。
    # ------------------------------------------------------

    received_payloads = (
        collect_burst_acks(
            sock,
            rx_buffer,
            expected_payloads
        )
    )


    matched = (
        expected_payloads
        &
        received_payloads
    )


    success_count = len(
        matched
    )


    lost_count = (
        frame_count
        - success_count
    )


    success_rate = (
        success_count
        / frame_count
        * 100.0
    )


    # ------------------------------------------------------
    # 查询 Burst 后驱动丢字节计数。
    # ------------------------------------------------------

    drop_after = (
        query_rx_drop_count(
            sock,
            rx_buffer
        )
    )

        # uint32_t 计数器理论上可能溢出，
    # 因此按 32 bit 计算差值。
    drop_delta = (
        drop_after
        - drop_before
    ) & 0xFFFFFFFF


    return {
        "frame_count": frame_count,
        "burst_bytes": burst_bytes,
        "send_elapsed": send_elapsed,
        "host_push_rate": host_push_rate,
        "success_count": success_count,
        "lost_count": lost_count,
        "success_rate": success_rate,
        "drop_before": drop_before,
        "drop_after": drop_after,
        "drop_delta": drop_delta,
    }


# ==========================================================
# 主程序
# ==========================================================

def main():

    print(
        "正在连接 QEMU UART："
        f"{HOST}:{PORT}"
    )


    sock = socket.create_connection(
        (HOST, PORT)
    )


    sock.setsockopt(
        socket.IPPROTO_TCP,
        socket.TCP_NODELAY,
        1
    )


    rx_buffer = bytearray()


    print("连接成功。")
    print()


    try:

        print(
            "=============================================="
        )

        print(
            "UART Burst Stress Test"
        )

        print(
            "=============================================="
        )

        print()


        sequence_start = 10000

        results = []


        for frame_count in BURST_FRAME_COUNTS:

            print(
                f"测试 Burst："
                f"{frame_count} frames"
            )


            result = run_burst_test(
                sock,
                rx_buffer,
                frame_count,
                sequence_start
            )


            results.append(
                result
            )


            print(
                f"  Burst 大小："
                f"{result['burst_bytes']} Byte"
            )


            print(
                f"  sendall 耗时："
                f"{result['send_elapsed']:.6f} s"
            )


            print(
                f"  Host push rate："
                f"{result['host_push_rate']:.1f} frame/s"
            )


            print(
                f"  ACK："
                f"{result['success_count']}/"
                f"{frame_count}"
            )


            print(
                f"  ACK Lost："
                f"{result['lost_count']}"
            )


            print(
                f"  ACK Success："
                f"{result['success_rate']:.2f}%"
            )


            print(
                f"  RX Drop Before："
                f"{result['drop_before']} Byte"
            )


            print(
                f"  RX Drop After："
                f"{result['drop_after']} Byte"
            )


            print(
                f"  RX Drop Delta："
                f"{result['drop_delta']} Byte"
            )


            print()


            sequence_start += (
                frame_count
                + 100
            )


            time.sleep(
                TEST_GAP
            )


        # ==================================================
        # 汇总
        # ==================================================

        print()

        print(
            "===================================================================="
        )

        print(
            "Burst Test Summary"
        )

        print(
            "===================================================================="
        )


        print(
            f"{'Frames':>8}"
            f"{'Bytes':>10}"
            f"{'ACK':>10}"
            f"{'Lost':>10}"
            f"{'ACK %':>12}"
            f"{'Drop Bytes':>14}"
        )


        print(
            "-" * 64
        )


        for result in results:

            print(
                f"{result['frame_count']:>8}"
                f"{result['burst_bytes']:>10}"
                f"{result['success_count']:>10}"
                f"{result['lost_count']:>10}"
                f"{result['success_rate']:>11.2f}%"
                f"{result['drop_delta']:>14}"
            )


        print()

        print(
            "Drop Bytes 表示 UART Driver Ring Buffer 满时"
        )

        print(
            "由 ISR 丢弃的字节数，不是协议帧数量。"
        )


    finally:

        sock.close()


if __name__ == "__main__":
    main()