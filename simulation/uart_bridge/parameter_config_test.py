"""
文件：parameter_config_test.py

用途：
验证 Runtime Parameter Configuration 通信链路。

当前测试参数：

ProtocolTX Period
"""

import select
import socket
import statistics
import time

from protocol_codec import (
    CMD_PARAMETER_ACK,
    CMD_SET_JOINT_TARGETS,
    PARAM_PROTOCOL_TX_PERIOD_MS,
    build_set_parameter_frame,
    extract_frames,
    parse_frame,
    parse_parameter_ack_payload,
)


# ==========================================================
# TCP Configuration
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555

# Parameter ACK 最大等待时间，单位 second。
ACK_TIMEOUT_S = 2.0


# ==========================================================
# Test Configuration
# ==========================================================

# 默认 ProtocolTX Period。
DEFAULT_PROTOCOL_TX_PERIOD_MS = 1000

# 测试期间使用的快速发送周期。
FAST_PROTOCOL_TX_PERIOD_MS = 200

# 周期比例允许 ±15% 相对误差。
RATIO_TOLERANCE = 0.15


# ==========================================================
# Receive
# ==========================================================

def receive_frames(
    sock: socket.socket,
    rx_buffer: bytearray,
    timeout_s: float,
) -> list[bytes]:
    """
    在指定时间窗口内接收所有完整 Protocol Frame。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议 Buffer。

        timeout_s:
            最大接收时间，单位 second。

    Returns:
        当前窗口内提取出的完整 Frame List。
    """
    frames = []

    deadline = (
        time.monotonic()
        + timeout_s
    )

    while time.monotonic() < deadline:
        remaining = (
            deadline
            - time.monotonic()
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

        frames.extend(
            extract_frames(
                rx_buffer
            )
        )

    return frames


# ==========================================================
# Parameter ACK
# ==========================================================

def wait_for_parameter_ack(
    sock: socket.socket,
    rx_buffer: bytearray,
    expected_parameter_id: int,
    timeout_s: float,
):
    """
    等待指定 Parameter ID 的 PARAMETER_ACK。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议 Buffer。

        expected_parameter_id:
            预期 Parameter ID。

        timeout_s:
            最大等待时间，单位 second。

    Returns:
        成功时：

        (
            status,
            effective_value,
        )

        Timeout 或无合法 ACK 时返回 None。
    """
    deadline = (
        time.monotonic()
        + timeout_s
    )

    while time.monotonic() < deadline:
        remaining = (
            deadline
            - time.monotonic()
        )

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            remaining,
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

            if command != CMD_PARAMETER_ACK:
                continue

            result = parse_parameter_ack_payload(
                payload
            )

            if result is None:
                continue

            (
                parameter_id,
                status,
                effective_value,
            ) = result

            if parameter_id == expected_parameter_id:
                return (
                    status,
                    effective_value,
                )

    return None


def set_protocol_tx_period(
    sock: socket.socket,
    rx_buffer: bytearray,
    period_ms: int,
) -> bool:
    """
    设置 MCU ProtocolTX Period 并验证 ACK。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议 Buffer。

        period_ms:
            目标 ProtocolTX Period，单位 ms。

    Returns:
        True：
        MCU 接受配置且 Effective Value 正确。

        False：
        ACK Timeout、配置被拒绝或
        Effective Value 不匹配。
    """
    frame = build_set_parameter_frame(
        PARAM_PROTOCOL_TX_PERIOD_MS,
        period_ms,
    )

    print(
        f"发送配置：ProtocolTX Period = "
        f"{period_ms} ms"
    )

    sock.sendall(
        frame
    )

    result = wait_for_parameter_ack(
        sock,
        rx_buffer,
        PARAM_PROTOCOL_TX_PERIOD_MS,
        ACK_TIMEOUT_S,
    )

    if result is None:
        print(
            "[FAIL] 未收到 0x85 PARAMETER_ACK"
        )

        return False

    status, effective_value = result

    print(
        f"收到 ACK：status={status}, "
        f"effective_value={effective_value} ms"
    )

    if status != 0:
        print(
            "[FAIL] MCU 拒绝参数配置"
        )

        return False

    if effective_value != period_ms:
        print(
            "[FAIL] ACK Effective Value 不匹配"
        )

        return False

    print(
        "[PASS] 参数配置 ACK 正确"
    )

    return True


# ==========================================================
# Period Measurement
# ==========================================================

def measure_protocol_tx_period(
    sock: socket.socket,
    rx_buffer: bytearray,
    sample_count: int,
    timeout_s: float,
):
    """
    测量 MCU 周期发送 CMD_SET_JOINT_TARGETS 的 Host 间隔。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议 Buffer。

        sample_count:
            需要记录的 Frame Timestamp 数量。

        timeout_s:
            最大采样时间，单位 second。

    Returns:
        相邻 Frame 间隔 List，单位 ms。

        少于三个 Timestamp 时返回 None。

    Note:
        返回的是 Host Wall Clock 观测间隔，
        主要用于比较配置前后的相对变化。
    """
    timestamps = []

    deadline = (
        time.monotonic()
        + timeout_s
    )

    while (
        len(timestamps) < sample_count
        and time.monotonic() < deadline
    ):
        remaining = (
            deadline
            - time.monotonic()
        )

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            remaining,
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
            command, _ = parse_frame(
                frame
            )

            if command == CMD_SET_JOINT_TARGETS:
                timestamps.append(
                    time.monotonic()
                )

                if len(timestamps) >= sample_count:
                    break

    if len(timestamps) < 3:
        return None

    return [
        (
            timestamps[index]
            - timestamps[index - 1]
        )
        * 1000.0
        for index in range(
            1,
            len(timestamps)
        )
    ]


def get_stable_median(
    intervals: list[float],
) -> float:
    """
    获取较稳定的 Period Median。

    Args:
        intervals:
            相邻 Frame 间隔，单位 ms。

    Returns:
        Median Period，单位 ms。

    Note:
        参数刚切换时，
        MCU 可能仍处于旧 Period 的 Delay 中。

        因此优先忽略前两个 Interval。
    """
    stable_intervals = intervals[2:]

    if not stable_intervals:
        stable_intervals = intervals

    return statistics.median(
        stable_intervals
    )


def print_intervals(
    intervals: list[float],
) -> None:
    """
    打印测得的 ProtocolTX Period List。

    Args:
        intervals:
            Period List，单位 ms。
    """
    print(
        "测得周期：",
        [
            round(
                value,
                1
            )
            for value in intervals
        ],
        "ms"
    )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
    """
    执行完整 Parameter Configuration Test。
    """
    print(
        f"正在连接 QEMU UART："
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

    print("连接成功。")
    print()

    passed_count = 0
    total_count = 0

    try:
        # 清理连接建立前后已经在路上的旧数据。
        receive_frames(
            sock,
            rx_buffer,
            0.3,
        )

        # ==================================================
        # Test 1
        # Default Baseline
        # ==================================================

        total_count += 1

        print(
            "=== Test 1: 测量默认 ProtocolTX 基线 ==="
        )

        baseline_intervals = measure_protocol_tx_period(
            sock,
            rx_buffer,
            sample_count=6,
            timeout_s=10.0,
        )

        if baseline_intervals is None:
            print(
                "[FAIL] 无法测得默认 ProtocolTX 周期"
            )

            return

        print_intervals(
            baseline_intervals
        )

        baseline_ms = get_stable_median(
            baseline_intervals
        )

        print(
            f"Baseline Median = "
            f"{baseline_ms:.1f} ms"
        )

        print(
            "[PASS] 默认基线测量完成"
        )

        passed_count += 1

        print()

        # ==================================================
        # Test 2
        # Set 200 ms
        # ==================================================

        total_count += 1

        print(
            "=== Test 2: 设置 ProtocolTX = 200 ms ==="
        )

        if set_protocol_tx_period(
            sock,
            rx_buffer,
            FAST_PROTOCOL_TX_PERIOD_MS,
        ):
            passed_count += 1

        print()

        # ==================================================
        # Test 3
        # Relative Period
        # ==================================================

        total_count += 1

        print(
            "=== Test 3: 验证 200 ms 相对周期 ==="
        )

        fast_intervals = measure_protocol_tx_period(
            sock,
            rx_buffer,
            sample_count=8,
            timeout_s=5.0,
        )

        if fast_intervals is None:
            print(
                "[FAIL] 无法测得配置后的周期"
            )

        else:
            print_intervals(
                fast_intervals
            )

            fast_ms = get_stable_median(
                fast_intervals
            )

            measured_ratio = (
                fast_ms
                / baseline_ms
            )

            expected_ratio = (
                FAST_PROTOCOL_TX_PERIOD_MS
                / DEFAULT_PROTOCOL_TX_PERIOD_MS
            )

            print(
                f"Fast Median = "
                f"{fast_ms:.1f} ms"
            )

            print(
                f"Measured Ratio = "
                f"{measured_ratio:.3f}"
            )

            print(
                f"Expected Ratio = "
                f"{expected_ratio:.3f}"
            )

            lower_bound = (
                expected_ratio
                * (1.0 - RATIO_TOLERANCE)
            )

            upper_bound = (
                expected_ratio
                * (1.0 + RATIO_TOLERANCE)
            )

            if (
                lower_bound
                <= measured_ratio
                <= upper_bound
            ):
                print(
                    "[PASS] ProtocolTX 周期按配置比例缩短"
                )

                passed_count += 1

            else:
                print(
                    "[FAIL] 周期变化比例不符合配置"
                )

        print()

        # ==================================================
        # Test 4
        # Restore 1000 ms
        # ==================================================

        total_count += 1

        print(
            "=== Test 4: 恢复 ProtocolTX = 1000 ms ==="
        )

        if set_protocol_tx_period(
            sock,
            rx_buffer,
            DEFAULT_PROTOCOL_TX_PERIOD_MS,
        ):
            passed_count += 1

        print()

        # ==================================================
        # Test 5
        # Verify Restore
        # ==================================================

        total_count += 1

        print(
            "=== Test 5: 验证周期恢复 ==="
        )

        restored_intervals = measure_protocol_tx_period(
            sock,
            rx_buffer,
            sample_count=6,
            timeout_s=10.0,
        )

        if restored_intervals is None:
            print(
                "[FAIL] 无法测得恢复后的周期"
            )

        else:
            print_intervals(
                restored_intervals
            )

            restored_ms = get_stable_median(
                restored_intervals
            )

            restored_ratio = (
                restored_ms
                / baseline_ms
            )

            print(
                f"Restored Median = "
                f"{restored_ms:.1f} ms"
            )

            print(
                f"Restored / Baseline = "
                f"{restored_ratio:.3f}"
            )

            lower_bound = (
                1.0
                - RATIO_TOLERANCE
            )

            upper_bound = (
                1.0
                + RATIO_TOLERANCE
            )

            if (
                lower_bound
                <= restored_ratio
                <= upper_bound
            ):
                print(
                    "[PASS] ProtocolTX 周期恢复到原基线"
                )

                passed_count += 1

            else:
                print(
                    "[FAIL] 恢复后的周期偏离原基线"
                )

        print()
        print(
            "======================================"
        )

        print(
            f"Parameter Configuration Test: "
            f"{passed_count}/{total_count} PASS"
        )

        print(
            "======================================"
        )

    finally:
        sock.close()


if __name__ == "__main__":
    main()