"""
文件：parameter_config_test.py

用途：
验证 Parameter Configuration 通信链路。

测试内容：
1. 测量默认 ProtocolTX 基线周期；
2. 设置 ProtocolTX Period = 200 ms；
3. 验证 MCU 返回 PARAMETER_ACK；
4. 验证新周期约为基线的 20%；
5. 恢复 ProtocolTX Period = 1000 ms；
6. 验证周期恢复到原基线。

注意：
QEMU Guest 时间与 Host Wall Clock
不要求严格 1:1。

因此本测试以相对周期变化为主要判据。
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


HOST = "127.0.0.1"
PORT = 5555

ACK_TIMEOUT_S = 2.0


# ==========================================================
# Receive
# ==========================================================

def receive_frames(
    sock: socket.socket,
    rx_buffer: bytearray,
    timeout_s: float,
) -> list[bytes]:
    frames = []

    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            min(remaining, 0.05),
        )

        if not readable:
            continue

        data = sock.recv(4096)

        if not data:
            break

        rx_buffer.extend(data)

        frames.extend(
            extract_frames(rx_buffer)
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
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            remaining,
        )

        if not readable:
            break

        data = sock.recv(4096)

        if not data:
            break

        rx_buffer.extend(data)

        frames = extract_frames(
            rx_buffer
        )

        for frame in frames:
            command, payload = parse_frame(frame)

            if command != CMD_PARAMETER_ACK:
                continue

            result = parse_parameter_ack_payload(
                payload
            )

            if result is None:
                continue

            parameter_id, status, effective_value = result

            if parameter_id == expected_parameter_id:
                return status, effective_value

    return None


def set_protocol_tx_period(
    sock: socket.socket,
    rx_buffer: bytearray,
    period_ms: int,
) -> bool:
    frame = build_set_parameter_frame(
        PARAM_PROTOCOL_TX_PERIOD_MS,
        period_ms,
    )

    print(
        f"发送配置：ProtocolTX Period = "
        f"{period_ms} ms"
    )

    sock.sendall(frame)

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
    timestamps = []

    deadline = time.monotonic() + timeout_s

    while (
        len(timestamps) < sample_count
        and time.monotonic() < deadline
    ):
        remaining = deadline - time.monotonic()

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            remaining,
        )

        if not readable:
            break

        data = sock.recv(4096)

        if not data:
            break

        rx_buffer.extend(data)

        frames = extract_frames(
            rx_buffer
        )

        for frame in frames:
            command, _ = parse_frame(frame)

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
    # 参数切换时可能正处于旧的一次 Delay 中，
    # 因此忽略前两个区间。
    stable_intervals = intervals[2:]

    if not stable_intervals:
        stable_intervals = intervals

    return statistics.median(
        stable_intervals
    )


def print_intervals(
    intervals: list[float],
) -> None:
    print(
        "测得周期：",
        [
            round(value, 1)
            for value in intervals
        ],
        "ms"
    )


# ==========================================================
# Main
# ==========================================================

def main():
    print(
        f"正在连接 QEMU UART："
        f"{HOST}:{PORT}"
    )

    sock = socket.create_connection(
        (HOST, PORT)
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
        # 清理连接刚建立时已经在路上的数据。
        receive_frames(
            sock,
            rx_buffer,
            0.3,
        )

        # ==================================================
        # Test 1
        # 测量默认基线
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
        # 设置 200 ms
        # ==================================================

        total_count += 1

        print(
            "=== Test 2: 设置 ProtocolTX = 200 ms ==="
        )

        if set_protocol_tx_period(
            sock,
            rx_buffer,
            200,
        ):
            passed_count += 1

        print()

        # ==================================================
        # Test 3
        # 验证相对比例
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
                200.0
                / 1000.0
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

            # 允许 ±15% 相对误差。
            lower_bound = (
                expected_ratio * 0.85
            )

            upper_bound = (
                expected_ratio * 1.15
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
        # 恢复 1000 ms
        # ==================================================

        total_count += 1

        print(
            "=== Test 4: 恢复 ProtocolTX = 1000 ms ==="
        )

        if set_protocol_tx_period(
            sock,
            rx_buffer,
            1000,
        ):
            passed_count += 1

        print()

        # ==================================================
        # Test 5
        # 验证恢复
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

            if 0.85 <= restored_ratio <= 1.15:
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