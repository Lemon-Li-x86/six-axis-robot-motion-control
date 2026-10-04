"""
文件：communication_stress_test.py

用途：
对当前 QEMU Cortex-M4 UART 通信链路
进行连续高频压力测试。

测试内容：

1. 按指定目标速率连续发送 JOINT_STATE（0x81）；
2. 统计 MCU 返回的 JOINT_STATE_ACK（0x82）；
3. 统计成功帧数和丢失帧数；
4. 测量实际完成发送所需时间；
5. 根据实际发送时间计算真实平均发送速率。

协议编解码统一由 protocol_codec.py 提供。

注意：

TEST_RATES_HZ 表示目标发送速率。

Windows + Python 不能保证严格按照该频率调度，
因此最终必须同时观察 actual_rate_hz。

本测试反映：

QEMU + TCP + Windows + Python + FreeRTOS

组成的完整仿真系统压力表现。

不能直接作为真实 Cortex-M4 硬件吞吐能力。
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
# TCP
# ==========================================================

HOST = "127.0.0.1"

PORT = 5555


# ==========================================================
# Stress Test Configuration
# ==========================================================

TEST_RATES_HZ = [
    1000,
    2000,
    5000,
    10000,
]

FRAMES_PER_RATE = 1000

ACK_DRAIN_TIMEOUT = 2.0

RATE_TEST_GAP = 0.5


# ==========================================================
# Test Payload Builder
# ==========================================================

def build_joint_state_frame(
    sequence: int,
) -> tuple[bytes, bytes]:
    """
    使用 sequence 构造唯一 Payload。

    收到 ACK 后通过原始 Payload
    判断具体对应哪一帧。
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
# Receive Current Data
# ==========================================================

def receive_current_data(
    sock: socket.socket,
    rx_buffer: bytearray,
    received_payloads: set[bytes],
) -> None:
    """
    非阻塞读取当前已经到达 Socket 的数据。

    压力测试发送过程中同时消费 ACK，
    避免主机 TCP RX Buffer 持续积压。
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
                command
                == CMD_JOINT_STATE_ACK
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
    全部测试帧发送完成以后，
    等待仍然存在于链路中的 ACK。

    如果全部预期 ACK 已经收到，
    提前结束。
    """

    deadline = (
        time.perf_counter()
        + timeout
    )

    while (
        time.perf_counter()
        < deadline
    ):
        if (
            expected_payloads
            <= received_payloads
        ):
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
                command
                == CMD_JOINT_STATE_ACK
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
):
    """
    在指定目标频率下发送 FRAMES_PER_RATE 帧。

    返回：

        success_count
        lost_count
        success_rate
        send_elapsed
        actual_rate_hz
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

        # --------------------------------------------------
        # 等待目标发送时刻
        # --------------------------------------------------

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

        # --------------------------------------------------
        # Send
        # --------------------------------------------------

        sock.sendall(
            frame
        )

        next_send_time += (
            interval
        )

        # --------------------------------------------------
        # 同时消费已经返回的 ACK
        # --------------------------------------------------

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
        &
        received_payloads
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