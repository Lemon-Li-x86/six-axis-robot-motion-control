"""
文件：burst_stress_test.py

用途：
对当前 QEMU Cortex-M4 UART 接收链路
执行 Burst 压力测试。
"""

import select
import socket
import struct
import time

from protocol_codec import (
    CMD_DIAGNOSTICS_RESPONSE,
    CMD_JOINT_STATE,
    CMD_JOINT_STATE_ACK,
    build_diagnostics_request,
    build_frame,
    extract_frames,
    parse_diagnostics_response_payload,
    parse_frame,
)


# ==========================================================
# TCP Configuration
# ==========================================================

# QEMU UART0 TCP Endpoint。
HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# Burst Configuration
# ==========================================================

# 每轮一次性发送的 Frame 数量。
#
# 从较小 Burst 开始逐级提高，
# 用于观察 ACK Loss 和 RX Drop
# 在不同突发规模下的变化。
BURST_FRAME_COUNTS = [
    10,
    100,
    500,
    1000,
    5000,
]

# 一次 Burst 发送完成后，
# 等待剩余 ACK 的最长时间，单位 second。
ACK_TIMEOUT = 10.0

# 不同 Burst Test 之间的间隔，单位 second。
TEST_GAP = 0.5


# ==========================================================
# Test Payload Builder
# ==========================================================

def build_joint_state_frame(
    sequence: int,
) -> tuple[bytes, bytes]:
    """
    使用 sequence 构造可区分的 JOINT_STATE Frame。

    Args:
        sequence:
            测试序号。

    Returns:
        Tuple：

        (
            complete_frame,
            payload,
        )

    Note:
        Payload 使用：

        6 × signed int16

        Wire Format 为 Little Endian。

        不同 sequence 产生不同 Payload，
        便于将 ACK 与发送 Frame 匹配。
    """
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
        *joints,
    )

    frame = build_frame(
        CMD_JOINT_STATE,
        payload,
    )

    return frame, payload


# ==========================================================
# Socket Settle
# ==========================================================

def settle_socket(
    sock: socket.socket,
    rx_buffer: bytearray,
    timeout: float = 0.2,
) -> None:
    """
    消费测试开始前已经存在于 TCP 链路中的数据。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        timeout:
            清理时间窗口，单位 second。

    Note:
        本函数只消费旧 Frame，
        防止上一阶段残余响应影响当前 Burst 统计。
    """
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
            0.01,
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
# Diagnostics
# ==========================================================

def query_rx_drop_count(
    sock: socket.socket,
    rx_buffer: bytearray,
) -> int:
    """
    查询 MCU UART Driver RX Drop Byte Count。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

    Returns:
        MCU 当前累计 RX Drop Byte Count。

    Raises:
        RuntimeError:
            连续三次查询都没有获得合法
            Diagnostics Response。

    Note:
        每次查询最多等待 1 second。

        高负载后允许最多尝试三次，
        避免旧 ACK 或链路积压影响 Diagnostics 查询。
    """
    request = (
        build_diagnostics_request()
    )

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
                    0.05,
                ),
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
                command, payload = parse_frame(
                    frame
                )

                if (
                    command == CMD_DIAGNOSTICS_RESPONSE
                    and payload is not None
                ):
                    value = (
                        parse_diagnostics_response_payload(
                            payload
                        )
                    )

                    if value is not None:
                        return value

    raise RuntimeError(
        "无法获得 MCU Diagnostics Response"
    )


# ==========================================================
# ACK Collection
# ==========================================================

def collect_burst_acks(
    sock: socket.socket,
    rx_buffer: bytearray,
    expected_payloads: set[bytes],
) -> set[bytes]:
    """
    收集当前 Burst 对应的 JOINT_STATE_ACK。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        expected_payloads:
            当前 Burst 所有预期 ACK Payload。

    Returns:
        实际收到的 ACK Payload Set。

    Note:
        ACK 收集最长持续 ACK_TIMEOUT second。

        如果全部 Expected Payload
        已经收到，则提前结束。
    """
    received_payloads = set()

    deadline = (
        time.perf_counter()
        + ACK_TIMEOUT
    )

    while (
        time.perf_counter()
        < deadline
    ):
        if expected_payloads <= received_payloads:
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
                0.01,
            ),
        )

        if not readable:
            continue

        # Burst 后返回数据可能较多，
        # 因此这里使用较大的单次读取 Buffer。
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
            command, payload = parse_frame(
                frame
            )

            if (
                command == CMD_JOINT_STATE_ACK
                and payload is not None
            ):
                received_payloads.add(
                    payload
                )

    return received_payloads


# ==========================================================
# Single Burst Test
# ==========================================================

def run_burst_test(
    sock: socket.socket,
    rx_buffer: bytearray,
    frame_count: int,
    sequence_start: int,
) -> dict:
    """
    执行一次指定规模的 Burst Test。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        frame_count:
            当前 Burst 包含的 Frame 数量。

        sequence_start:
            当前测试的起始 Sequence。

    Returns:
        包含 Burst Size、发送耗时、
        ACK 成功数、丢失数和 RX Drop
        等指标的 Dictionary。
    """
    settle_socket(
        sock,
        rx_buffer,
    )

    drop_before = query_rx_drop_count(
        sock,
        rx_buffer,
    )

    frames = []
    expected_payloads = set()

    for index in range(
        frame_count
    ):
        frame, payload = (
            build_joint_state_frame(
                sequence_start
                + index
            )
        )

        frames.append(
            frame
        )

        expected_payloads.add(
            payload
        )

    # 将所有协议帧合并成一次 TCP sendall()。
    #
    # 目的是尽可能制造突发输入，
    # 而不是按照固定周期逐帧发送。
    burst_data = b"".join(
        frames
    )

    burst_bytes = len(
        burst_data
    )

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

    if send_elapsed > 0.0:
        host_push_rate = (
            frame_count
            / send_elapsed
        )
    else:
        host_push_rate = 0.0

    received_payloads = (
        collect_burst_acks(
            sock,
            rx_buffer,
            expected_payloads,
        )
    )

    matched = (
        expected_payloads
        & received_payloads
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

    drop_after = query_rx_drop_count(
        sock,
        rx_buffer,
    )

    # MCU Counter 为 uint32_t。
    #
    # 即使理论上发生 32 bit Wrap，
    # 这里仍可以得到正确无符号差值。
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
# Entry
# ==========================================================

def main() -> None:
    """
    连接 QEMU UART 并依次执行所有 Burst Test。
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

    # 禁止 Nagle Algorithm，
    # 避免额外 TCP 聚合影响测试行为。
    sock.setsockopt(
        socket.IPPROTO_TCP,
        socket.TCP_NODELAY,
        1,
    )

    rx_buffer = bytearray()

    print(
        "连接成功。"
    )
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

        # 各测试使用不同 Sequence 区域，
        # 减少不同 Burst 之间 Payload 重复。
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
                sequence_start,
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