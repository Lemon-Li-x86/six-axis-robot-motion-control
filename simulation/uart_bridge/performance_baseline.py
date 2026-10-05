"""
文件：performance_baseline.py

用途：
测量当前 QEMU Cortex-M4 UART 通信链路
端到端往返延迟 RTT。
"""

import select
import socket
import statistics
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

CMD_JOINT_STATE = 0x81
CMD_JOINT_STATE_ACK = 0x82

JOINT_COUNT = 6


# ==========================================================
# Benchmark Configuration
# ==========================================================

# 正式测试前的预热次数。
#
# 预热 Sample 不进入最终 RTT Statistics。
WARMUP_COUNT = 20

# 正式 RTT 样本数量。
TEST_COUNT = 200

# 单个 JOINT_STATE 最大 ACK 等待时间，单位 second。
ACK_TIMEOUT = 1.0


# ==========================================================
# Protocol Helpers
# ==========================================================

def build_frame(
    command: int,
    payload: bytes,
) -> bytes:
    """
    构造通用协议帧。

    Args:
        command:
            8 bit Protocol Command。

        payload:
            Payload Byte Sequence。

    Returns:
        完整 Frame。

    Note:
        Frame：

        AA 55 | Command | Length | Payload | Checksum

        Checksum 为 Command + Length + Payload
        累加结果的低 8 bit。
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
    sequence: int,
) -> tuple[bytes, bytes]:
    """
    根据 Sequence 构造唯一 JOINT_STATE Frame。

    Args:
        sequence:
            当前测试 Sequence。

    Returns:
        Tuple：

        (
            complete_frame,
            payload,
        )

    Note:
        Joint Payload 为：

        6 × signed int16
        Little Endian。

        Payload 唯一性用于匹配对应 ACK。
    """
    joints = [
        sequence,
        sequence + 1,
        sequence + 2,
        sequence + 3,
        sequence + 4,
        sequence + 5,
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


def extract_frames(
    buffer: bytearray,
) -> list[bytes]:
    """
    从 TCP Byte Stream 提取完整协议帧。

    Args:
        buffer:
            持续使用的可变 Byte Buffer。

    Returns:
        当前可以完整提取的 Frame List。
    """
    frames = []

    while True:
        # 至少需要 Header + Command + Length。
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
    校验并解析一个完整协议帧。

    Args:
        frame:
            完整 Protocol Frame。

    Returns:
        合法时：

        (
            command,
            payload,
        )

        非法时：

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

    received_checksum = frame[-1]

    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )

    if received_checksum != calculated_checksum:
        return None, None

    payload = frame[4:-1]

    return command, payload


# ==========================================================
# ACK Wait
# ==========================================================

def wait_for_matching_ack(
    sock: socket.socket,
    rx_buffer: bytearray,
    expected_payload: bytes,
    timeout: float,
) -> bool:
    """
    等待与当前 JOINT_STATE Payload 完全一致的 ACK。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议 Buffer。

        expected_payload:
            当前发送 Frame 的 Payload。

        timeout:
            最大等待时间，单位 second。

    Returns:
        True：
        收到匹配 JOINT_STATE_ACK。

        False：
        Timeout 或 Connection Closed。

    Note:
        MCU 还会周期发送 CMD_SET_JOINT_TARGETS，
        因此不能把“收到任意 Frame”
        视为当前 RTT Sample 完成。
    """
    deadline = (
        time.perf_counter()
        + timeout
    )

    while time.perf_counter() < deadline:
        remaining = (
            deadline
            - time.perf_counter()
        )

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            remaining,
        )

        if not readable:
            return False

        data = sock.recv(
            4096
        )

        if not data:
            return False

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
                and payload == expected_payload
            ):
                return True

    return False


# ==========================================================
# Statistics
# ==========================================================

def percentile(
    values: list[float],
    percentage: float,
) -> float:
    """
    使用线性插值计算 Percentile。

    Args:
        values:
            输入数值 List。

        percentage:
            Percentile Fraction。

            例如：
            0.95 = P95。

    Returns:
        Percentile Value。

        输入为空时返回 0。
    """
    ordered = sorted(
        values
    )

    if not ordered:
        return 0.0

    position = (
        (len(ordered) - 1)
        * percentage
    )

    lower_index = int(
        position
    )

    upper_index = min(
        lower_index + 1,
        len(ordered) - 1,
    )

    fraction = (
        position
        - lower_index
    )

    return (
        ordered[lower_index]
        * (1.0 - fraction)
        + ordered[upper_index]
        * fraction
    )


# ==========================================================
# RTT Measurement
# ==========================================================

def measure_once(
    sock: socket.socket,
    rx_buffer: bytearray,
    sequence: int,
):
    """
    执行一次完整 RTT 测量。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议 Buffer。

        sequence:
            当前测试 Frame Sequence。

    Returns:
        成功：
        RTT，单位 ms。

        Timeout：
        None。

    Note:
        RTT 起点位于 Python sendall() 前，
        终点位于收到匹配 ACK 后。
    """
    frame, payload = (
        build_joint_state_frame(
            sequence
        )
    )

    start_ns = (
        time.perf_counter_ns()
    )

    sock.sendall(
        frame
    )

    success = wait_for_matching_ack(
        sock,
        rx_buffer,
        payload,
        ACK_TIMEOUT,
    )

    end_ns = (
        time.perf_counter_ns()
    )

    if not success:
        return None

    elapsed_ns = (
        end_ns
        - start_ns
    )

    # nanosecond -> millisecond。
    return (
        elapsed_ns
        / 1_000_000.0
    )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
    """
    执行 UART RTT Performance Baseline。
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

    # 禁用 Nagle Algorithm，
    # 减少 localhost 小数据包额外等待。
    sock.setsockopt(
        socket.IPPROTO_TCP,
        socket.TCP_NODELAY,
        1,
    )

    rx_buffer = bytearray()

    print("连接成功。")
    print()

    try:
        # ==================================================
        # Warmup
        # ==================================================

        print(
            f"预热：{WARMUP_COUNT} 次"
        )

        for index in range(
            WARMUP_COUNT
        ):
            # 预热使用 Sequence 1000~1019。
            #
            # 正式测试从 2000 开始，
            # 避免 Payload 与 Warmup 重复。
            result = measure_once(
                sock,
                rx_buffer,
                1000 + index,
            )

            if result is None:
                print(
                    f"[WARN] "
                    f"预热 {index + 1} 未收到 ACK"
                )

        print("预热完成。")
        print()

        # ==================================================
        # RTT Samples
        # ==================================================

        print(
            f"开始 RTT 测试："
            f"{TEST_COUNT} 次"
        )

        samples_ms = []
        timeout_count = 0

        for index in range(
            TEST_COUNT
        ):
            result = measure_once(
                sock,
                rx_buffer,
                2000 + index,
            )

            if result is None:
                timeout_count += 1

                print(
                    f"[TIMEOUT] "
                    f"{index + 1}/{TEST_COUNT}"
                )

                continue

            samples_ms.append(
                result
            )

            # 每完成 20 个计划 Sample 打印一次进度。
            if (
                (index + 1) % 20
                == 0
            ):
                print(
                    f"进度："
                    f"{index + 1}/{TEST_COUNT}"
                )

        # ==================================================
        # Statistics
        # ==================================================

        print()

        print(
            "======================================"
        )

        print(
            "UART 通信 RTT 性能基线"
        )

        print(
            "======================================"
        )

        print(
            f"计划样本数：{TEST_COUNT}"
        )

        print(
            f"成功样本数：{len(samples_ms)}"
        )

        print(
            f"超时次数：{timeout_count}"
        )

        if not samples_ms:
            print()

            print(
                "没有获得有效 RTT 数据。"
            )

            return

        minimum_ms = min(
            samples_ms
        )

        mean_ms = statistics.mean(
            samples_ms
        )

        median_ms = statistics.median(
            samples_ms
        )

        p95_ms = percentile(
            samples_ms,
            0.95,
        )

        p99_ms = percentile(
            samples_ms,
            0.99,
        )

        maximum_ms = max(
            samples_ms
        )

        print()

        print(
            f"Minimum : "
            f"{minimum_ms:.3f} ms"
        )

        print(
            f"Mean    : "
            f"{mean_ms:.3f} ms"
        )

        print(
            f"Median  : "
            f"{median_ms:.3f} ms"
        )

        print(
            f"P95     : "
            f"{p95_ms:.3f} ms"
        )

        print(
            f"P99     : "
            f"{p99_ms:.3f} ms"
        )

        print(
            f"Maximum : "
            f"{maximum_ms:.3f} ms"
        )

        print()

        print(
            "说明："
        )

        print(
            "该数据表示 QEMU + TCP + "
            "FreeRTOS 固件整条通信链路的 RTT。"
        )

        print(
            "不能直接作为实际 Cortex-M4 "
            "硬件上的执行延迟。"
        )

        print(
            "======================================"
        )

    finally:
        sock.close()


if __name__ == "__main__":
    main()