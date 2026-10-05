"""
文件：communication_soak_test.py

用途：
对当前 QEMU Cortex-M4 + FreeRTOS UART 通信链路
执行长时间稳定性 Soak Test。

默认测试条件：

持续时间：
30 min

目标发送频率：
100 Hz

理论发送 Frame 数：
约 180,000

测试内容：

1. 周期发送合法 JOINT_STATE；
2. 等待对应 JOINT_STATE_ACK；
3. 统计成功 ACK；
4. 统计 ACK Timeout；
5. 查询测试前后 UART RX Drop Count；
6. 统计实际运行时长和平均发送频率；
7. 将最终结果保存为 JSON。
"""

import argparse
import json
import select
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

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
# Default Configuration
# ==========================================================

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5555

# 默认测试持续时间：
#
# 30 min
# =
# 1800 second。
DEFAULT_DURATION_S = 30.0 * 60.0

# 默认目标发送频率：
#
# 100 Frame / second。
DEFAULT_RATE_HZ = 100.0

# 单帧 ACK 最大等待时间：
#
# 0.05 second
# =
# 50 ms。
DEFAULT_ACK_TIMEOUT_S = 0.05

# 长时间测试状态打印周期：
#
# 60 second。
DEFAULT_PROGRESS_INTERVAL_S = 60.0


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
        Payload 使用原始 signed int16，
        单位为 0.01 degree。

        每个 Frame 使用不同 Payload，
        便于与 JOINT_STATE_ACK 逐帧匹配。
    """
    # 将 Base Value 控制在约：
    #
    # [-15000, 14999]
    #
    # 即约：
    #
    # [-150°, +149.99°]
    #
    # 避免 int16 范围溢出。
    base_value = (
        sequence % 30000
    ) - 15000

    joints = [
        base_value,
        base_value + 1,
        base_value + 2,
        base_value + 3,
        base_value + 4,
        base_value + 5,
    ]

    # Little Endian：
    #
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
# Socket Settle
# ==========================================================

def settle_socket(
    sock: socket.socket,
    rx_buffer: bytearray,
    timeout: float = 0.2,
) -> None:
    """
    消费当前 TCP 链路中已经存在的数据。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

        timeout:
            清理时间窗口，单位 second。

    Note:
        用于测试阶段切换前，
        防止旧 Frame 干扰后续统计。
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
    查询 UART Driver RX Drop Byte Count。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        rx_buffer:
            持续使用的协议接收 Buffer。

    Returns:
        MCU 当前累计 RX Drop Byte Count。

    Raises:
        RuntimeError:
            连续三次查询都没有获得
            Diagnostics Response。
    """
    request = (
        build_diagnostics_request()
    )

    # 链路刚经历较高负载时，
    # 允许最多尝试三次 Diagnostics 查询。
    for _ in range(3):
        sock.sendall(
            request
        )

        # 每次查询最多等待 1 second。
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
            持续使用的协议接收 Buffer。

        expected_payload:
            当前发送 Frame 的原始 Payload。

        timeout:
            ACK 最大等待时间，单位 second。

    Returns:
        True：
        在 Timeout 前收到匹配 ACK。

        False：
        超时仍未收到匹配 ACK。

    Raises:
        ConnectionError:
            QEMU UART TCP 连接断开。
    """
    deadline = (
        time.perf_counter()
        + timeout
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
                0.005,
            ),
        )

        if not readable:
            continue

        data = sock.recv(
            4096
        )

        if not data:
            raise ConnectionError(
                "QEMU UART TCP 连接已断开"
            )

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
# Result Output
# ==========================================================

def save_result(
    result: dict,
) -> Path:
    """
    将 Soak Test 结果保存为 JSON。

    Args:
        result:
            测试结果 Dictionary。

    Returns:
        最终 JSON 文件路径。

    Note:
        输出目录固定为：

        docs/testing/results/
    """
    current_file = (
        Path(__file__).resolve()
    )

    repo_root = (
        current_file.parent.parent.parent
    )

    result_dir = (
        repo_root
        / "docs"
        / "testing"
        / "results"
    )

    result_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    timestamp = (
        datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )
    )

    output_path = (
        result_dir
        / (
            "communication_soak_"
            f"{timestamp}.json"
        )
    )

    with output_path.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            result,
            file,
            ensure_ascii=False,
            indent=4,
        )

    return output_path


# ==========================================================
# Soak Test
# ==========================================================

def run_soak_test(
    host: str,
    port: int,
    duration_s: float,
    rate_hz: float,
    ack_timeout_s: float,
    progress_interval_s: float,
) -> None:
    """
    执行完整 UART Communication Soak Test。

    Args:
        host:
            QEMU UART TCP Host。

        port:
            QEMU UART TCP Port。

        duration_s:
            目标测试持续时间，单位 second。

        rate_hz:
            目标发送频率，单位 Frame / second。

        ack_timeout_s:
            单帧 ACK 最大等待时间，单位 second。

        progress_interval_s:
            状态输出间隔，单位 second。

    Raises:
        ValueError:
            duration_s 或 rate_hz 不大于 0。
    """
    if duration_s <= 0.0:
        raise ValueError(
            "duration 必须大于 0"
        )

    if rate_hz <= 0.0:
        raise ValueError(
            "rate 必须大于 0"
        )

    period_s = (
        1.0
        / rate_hz
    )

    print(
        "正在连接 QEMU UART："
        f"{host}:{port}"
    )

    sock = socket.create_connection(
        (
            host,
            port,
        )
    )

    # 禁用 Nagle Algorithm，
    # 减少 TCP 自动聚合对周期发送的影响。
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
        settle_socket(
            sock,
            rx_buffer,
        )

        drop_before = query_rx_drop_count(
            sock,
            rx_buffer,
        )

        sent_count = 0
        ack_count = 0
        timeout_count = 0

        start_time = (
            time.perf_counter()
        )

        next_send_time = (
            start_time
        )

        next_progress_time = (
            start_time
            + progress_interval_s
        )

        end_time = (
            start_time
            + duration_s
        )

        print(
            "=============================================="
        )
        print(
            "UART Communication Soak Test"
        )
        print(
            "=============================================="
        )
        print(
            f"目标持续时间：{duration_s:.1f} s"
        )
        print(
            f"目标发送频率：{rate_hz:.1f} Hz"
        )
        print(
            f"ACK Timeout："
            f"{ack_timeout_s * 1000.0:.1f} ms"
        )
        print(
            f"RX Drop Before："
            f"{drop_before} Byte"
        )
        print(
            "=============================================="
        )
        print()

        while True:
            now = (
                time.perf_counter()
            )

            if now >= end_time:
                break

            if now < next_send_time:
                time.sleep(
                    next_send_time
                    - now
                )

            frame, expected_payload = (
                build_joint_state_frame(
                    sent_count
                )
            )

            sock.sendall(
                frame
            )

            sent_count += 1

            ack_received = (
                wait_for_matching_ack(
                    sock,
                    rx_buffer,
                    expected_payload,
                    ack_timeout_s,
                )
            )

            if ack_received:
                ack_count += 1
            else:
                timeout_count += 1

            next_send_time += (
                period_s
            )

            current_time = (
                time.perf_counter()
            )

            # 如果程序已经比目标 Schedule
            # 落后超过一个完整 Period，
            # 则重新从当前时间建立下一次发送点。
            #
            # 这样避免为了追赶旧 Schedule
            # 瞬间连续发送大量补偿 Frame。
            if (
                current_time
                > next_send_time + period_s
            ):
                next_send_time = (
                    current_time
                    + period_s
                )

            if (
                current_time
                >= next_progress_time
            ):
                elapsed = (
                    current_time
                    - start_time
                )

                success_rate = (
                    ack_count
                    / sent_count
                    * 100.0
                    if sent_count > 0
                    else 0.0
                )

                actual_rate = (
                    sent_count
                    / elapsed
                    if elapsed > 0.0
                    else 0.0
                )

                print(
                    f"[{elapsed:8.1f} s] "
                    f"Sent={sent_count} "
                    f"ACK={ack_count} "
                    f"Timeout={timeout_count} "
                    f"Success={success_rate:.6f}% "
                    f"Rate={actual_rate:.2f} Hz"
                )

                next_progress_time += (
                    progress_interval_s
                )

        finish_time = (
            time.perf_counter()
        )

        actual_duration_s = (
            finish_time
            - start_time
        )

        # 主循环结束后再消费少量残留数据，
        # 避免随后 Diagnostics Request
        # 被旧 Frame 干扰。
        settle_socket(
            sock,
            rx_buffer,
            timeout=0.2,
        )

        drop_after = query_rx_drop_count(
            sock,
            rx_buffer,
        )

        # MCU Counter 为 uint32_t，
        # 使用无符号 32 bit 差值处理潜在 Wrap。
        drop_delta = (
            drop_after
            - drop_before
        ) & 0xFFFFFFFF

        lost_count = (
            sent_count
            - ack_count
        )

        success_rate = (
            ack_count
            / sent_count
            * 100.0
            if sent_count > 0
            else 0.0
        )

        actual_rate_hz = (
            sent_count
            / actual_duration_s
            if actual_duration_s > 0.0
            else 0.0
        )

        result = {
            "test": (
                "QEMU + TCP + FreeRTOS "
                "UART Communication Soak Test"
            ),
            "timestamp": (
                datetime.now().isoformat(
                    timespec="seconds"
                )
            ),
            "target_duration_s": duration_s,
            "actual_duration_s": actual_duration_s,
            "target_rate_hz": rate_hz,
            "actual_rate_hz": actual_rate_hz,
            "ack_timeout_s": ack_timeout_s,
            "sent": sent_count,
            "ack": ack_count,
            "lost": lost_count,
            "timeout": timeout_count,
            "ack_success_rate_percent": success_rate,
            "rx_drop_before_bytes": drop_before,
            "rx_drop_after_bytes": drop_after,
            "rx_drop_delta_bytes": drop_delta,
        }

        output_path = save_result(
            result
        )

        print()
        print(
            "=============================================="
        )
        print(
            "UART Communication Soak Test Result"
        )
        print(
            "=============================================="
        )
        print(
            f"Actual Duration : "
            f"{actual_duration_s:.3f} s"
        )
        print(
            f"Target Rate     : "
            f"{rate_hz:.3f} Hz"
        )
        print(
            f"Actual Rate     : "
            f"{actual_rate_hz:.3f} Hz"
        )
        print()
        print(
            f"Sent            : "
            f"{sent_count}"
        )
        print(
            f"ACK             : "
            f"{ack_count}"
        )
        print(
            f"Lost            : "
            f"{lost_count}"
        )
        print(
            f"Timeout         : "
            f"{timeout_count}"
        )
        print(
            f"ACK Success     : "
            f"{success_rate:.6f}%"
        )
        print()
        print(
            f"RX Drop Before  : "
            f"{drop_before} Byte"
        )
        print(
            f"RX Drop After   : "
            f"{drop_after} Byte"
        )
        print(
            f"RX Drop Delta   : "
            f"{drop_delta} Byte"
        )
        print()
        print(
            "结果文件："
        )
        print(
            output_path
        )
        print()
        print(
            "说明：该结果属于 "
            "QEMU + TCP + FreeRTOS "
            "仿真稳定性基线。"
        )
        print(
            "不能直接等同于真实 "
            "Cortex-M4 硬件 UART 性能。"
        )
        print(
            "=============================================="
        )

    finally:
        sock.close()


# ==========================================================
# Arguments
# ==========================================================

def parse_arguments():
    """
    解析 Soak Test 命令行参数。

    Returns:
        argparse.Namespace：
        包含 Host、Port、Duration、Rate、
        ACK Timeout 和 Progress Interval。
    """
    parser = argparse.ArgumentParser(
        description=(
            "QEMU Cortex-M4 UART "
            "communication soak test"
        )
    )

    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
    )

    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=(
            "测试持续时间，单位 second。"
            "默认 1800。"
        ),
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ,
        help=(
            "目标发送频率，单位 Hz。"
            "默认 100。"
        ),
    )

    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=DEFAULT_ACK_TIMEOUT_S,
        help=(
            "单帧 ACK Timeout，单位 second。"
            "默认 0.05。"
        ),
    )

    parser.add_argument(
        "--progress-interval",
        type=float,
        default=DEFAULT_PROGRESS_INTERVAL_S,
        help=(
            "进度打印周期，单位 second。"
            "默认 60。"
        ),
    )

    return parser.parse_args()


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
    """
    解析参数并启动 Communication Soak Test。
    """
    arguments = (
        parse_arguments()
    )

    run_soak_test(
        host=arguments.host,
        port=arguments.port,
        duration_s=arguments.duration,
        rate_hz=arguments.rate,
        ack_timeout_s=arguments.ack_timeout,
        progress_interval_s=(
            arguments.progress_interval
        ),
    )


if __name__ == "__main__":
    main()