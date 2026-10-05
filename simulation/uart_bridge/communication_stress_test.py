"""
文件：communication_stress_test.py

用途：
对当前 QEMU Cortex-M4 UART 通信链路
执行连续高频压力测试。

测试内容：

1. 按目标发送频率连续发送 JOINT_STATE；
2. 同时消费 MCU 返回的 JOINT_STATE_ACK；
3. 统计匹配 ACK 数量；
4. 统计最终丢失 Frame 数量；
5. 记录实际发送耗时；
6. 根据实际耗时计算真实平均发送频率。
"""

import select
import socket
import struct
import time

from protocol_codec import (
    CMD_JOINT_STATE,
    CMD_JOINT_STATE_ACK,
    build_frame,
    extract_frames,
    parse_frame,
)


# ==========================================================
# TCP Configuration
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# Stress Test Configuration
# ==========================================================

# 每一轮要求 Python 尝试达到的目标发送频率。
#
# 单位：
# Frame / second。
TEST_RATES_HZ = [
    1000,
    2000,
    5000,
    10000,
]

# 每一个目标频率下发送的 Frame 数量。
FRAMES_PER_RATE = 1000

# 全部 Frame 发送完成后，
# 继续等待剩余 ACK 的时间，单位 second。
ACK_DRAIN_TIMEOUT = 2.0

# 不同 Rate Test 之间的间隔，单位 second。
RATE_TEST_GAP = 0.5


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
            当前测试 Frame Sequence。

    Returns:
        Tuple：

        (
            complete_frame,
            payload,
        )

    Note:
        ACK Payload 与原 JOINT_STATE Payload 一致，
        因此可以直接通过 Payload 判断
        某一个发送 Frame 是否获得 ACK。
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


# ==========================================================
# Receive Current Data
# ==========================================================

def receive_current_data(
    sock: socket.socket,
    rx_buffer: bytearray,
    received_payloads: set[bytes],
) -> None:
    """
    非阻塞消费当前已经到达 Socket 的 ACK。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        received_payloads:
            已收到的 ACK Payload Set。

    Note:
        压力测试发送过程中同步消费 ACK，
        避免 Host TCP RX Buffer 持续积压。
    """
    while True:
        readable, _, _ = select.select(
            [sock],
            [],
            [],
            0,
        )

        if not readable:
            break

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
                command == CMD_JOINT_STATE_ACK
                and payload is not None
            ):
                received_payloads.add(
                    payload
                )


# ==========================================================
# ACK Drain
# ==========================================================

def drain_remaining_acks(
    sock: socket.socket,
    rx_buffer: bytearray,
    received_payloads: set[bytes],
    expected_payloads: set[bytes],
    timeout: float,
) -> None:
    """
    在发送结束后继续等待仍在链路中的 ACK。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        received_payloads:
            当前已经收到的 ACK Payload。

        expected_payloads:
            当前 Rate Test 所有预期 Payload。

        timeout:
            最大 Drain 时间，单位 second。

    Note:
        如果全部预期 ACK 已收到，
        则提前返回。
    """
    deadline = (
        time.perf_counter()
        + timeout
    )

    while (
        time.perf_counter()
        < deadline
    ):
        if expected_payloads <= received_payloads:
            return

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

        data = sock.recv(
            4096
        )

        if not data:
            return

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


# ==========================================================
# Single Rate Test
# ==========================================================

def run_rate_test(
    sock: socket.socket,
    rx_buffer: bytearray,
    rate_hz: int,
    sequence_start: int,
) -> tuple[int, int, float, float, float]:
    """
    在指定目标频率下发送 FRAMES_PER_RATE 个 Frame。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        rate_hz:
            目标发送频率，单位 Frame / second。

        sequence_start:
            当前测试使用的起始 Sequence。

    Returns:
        Tuple：

        (
            success_count,
            lost_count,
            success_rate,
            send_elapsed,
            actual_rate_hz,
        )
    """
    interval = (
        1.0
        / rate_hz
    )

    expected_payloads = set()
    received_payloads = set()

    send_start_time = (
        time.perf_counter()
    )

    next_send_time = (
        send_start_time
    )

    for index in range(
        FRAMES_PER_RATE
    ):
        sequence = (
            sequence_start
            + index
        )

        frame, payload = (
            build_joint_state_frame(
                sequence
            )
        )

        expected_payloads.add(
            payload
        )

        # 等待下一个目标发送时刻。
        #
        # 剩余时间大于 1 ms 时，
        # 先 sleep 到距离目标约 0.5 ms，
        # 最后一小段使用循环等待，
        # 减少 Windows sleep 粗粒度带来的误差。
        while True:
            now = (
                time.perf_counter()
            )

            remaining = (
                next_send_time
                - now
            )

            if remaining <= 0:
                break

            if remaining > 0.001:
                time.sleep(
                    remaining
                    - 0.0005
                )

        sock.sendall(
            frame
        )

        next_send_time += (
            interval
        )

        # 发送期间同步消费已经返回的 ACK，
        # 避免 TCP RX Queue 堆积。
        receive_current_data(
            sock,
            rx_buffer,
            received_payloads,
        )

    send_end_time = (
        time.perf_counter()
    )

    send_elapsed = (
        send_end_time
        - send_start_time
    )

    if send_elapsed > 0.0:
        actual_rate_hz = (
            FRAMES_PER_RATE
            / send_elapsed
        )
    else:
        actual_rate_hz = 0.0

    drain_remaining_acks(
        sock,
        rx_buffer,
        received_payloads,
        expected_payloads,
        ACK_DRAIN_TIMEOUT,
    )

    matched_payloads = (
        expected_payloads
        & received_payloads
    )

    success_count = len(
        matched_payloads
    )

    lost_count = (
        FRAMES_PER_RATE
        - success_count
    )

    success_rate = (
        success_count
        / FRAMES_PER_RATE
        * 100.0
    )

    return (
        success_count,
        lost_count,
        success_rate,
        send_elapsed,
        actual_rate_hz,
    )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
    """
    连接 QEMU UART 并依次执行全部 Rate Test。
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

    # 关闭 Nagle Algorithm，
    # 尽量降低 TCP 自动聚合带来的额外变量。
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

    print(
        "======================================"
    )
    print(
        "UART 连续通信压力测试"
    )
    print(
        "======================================"
    )
    print()

    try:
        results = []

        # 不同 Rate Test 使用不同 Sequence 区域，
        # 避免旧 ACK 与当前 Payload 重复。
        sequence_start = 5000

        for rate_hz in TEST_RATES_HZ:
            print(
                "--------------------------------------"
            )

            print(
                f"目标测试速率："
                f"{rate_hz} Hz"
            )

            print(
                f"发送帧数："
                f"{FRAMES_PER_RATE}"
            )

            theoretical_duration = (
                FRAMES_PER_RATE
                / rate_hz
            )

            print(
                f"理论发送耗时："
                f"{theoretical_duration:.4f} s"
            )

            (
                success_count,
                lost_count,
                success_rate,
                send_elapsed,
                actual_rate_hz,
            ) = run_rate_test(
                sock,
                rx_buffer,
                rate_hz,
                sequence_start,
            )

            results.append(
                (
                    rate_hz,
                    actual_rate_hz,
                    send_elapsed,
                    success_count,
                    lost_count,
                    success_rate,
                )
            )

            print(
                f"实际发送耗时："
                f"{send_elapsed:.4f} s"
            )

            print(
                f"实际平均发送速率："
                f"{actual_rate_hz:.1f} frame/s"
            )

            print(
                f"ACK 成功："
                f"{success_count}/{FRAMES_PER_RATE}"
            )

            print(
                f"丢失："
                f"{lost_count}"
            )

            print(
                f"成功率："
                f"{success_rate:.2f}%"
            )

            print()

            sequence_start += (
                FRAMES_PER_RATE
                + 100
            )

            time.sleep(
                RATE_TEST_GAP
            )

        print()

        print(
            "=============================================================="
        )
        print(
            "压力测试结果汇总"
        )
        print(
            "=============================================================="
        )

        print(
            f"{'Target':>10}"
            f"{'Actual':>14}"
            f"{'Time':>12}"
            f"{'Success':>12}"
            f"{'Lost':>8}"
            f"{'ACK %':>10}"
        )

        print(
            "-" * 66
        )

        for (
            target_rate_hz,
            actual_rate_hz,
            send_elapsed,
            success_count,
            lost_count,
            success_rate,
        ) in results:
            print(
                f"{target_rate_hz:>8} Hz"
                f"{actual_rate_hz:>11.1f} Hz"
                f"{send_elapsed:>10.4f} s"
                f"{success_count:>12}"
                f"{lost_count:>8}"
                f"{success_rate:>9.2f}%"
            )

        print()

        print(
            "说明："
        )

        print(
            "Target 表示脚本请求的目标发送频率。"
        )

        print(
            "Actual 表示根据实际发送总耗时计算得到的平均发送频率。"
        )

        print(
            "只有 Actual 才能说明 Python + Windows "
            "实际产生了多大的发送负载。"
        )

        print(
            "ACK % 表示在等待剩余 ACK 后，"
            "最终能够与发送 Payload 匹配的 ACK 比例。"
        )

        print()

        print(
            "本结果属于 QEMU 仿真性能基线，"
            "不能直接视为真实 Cortex-M4 硬件吞吐指标。"
        )

    finally:
        sock.close()


if __name__ == "__main__":
    main()