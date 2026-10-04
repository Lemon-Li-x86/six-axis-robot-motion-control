"""
文件：internal_performance_metrics.py

用途：
读取 Cortex-M4 固件内部性能基线。

建议运行顺序：

1. 重启 QEMU；
2. 运行 performance_baseline.py；
3. 再运行本程序。

这样 Task Wakeup 已经积累约 200 个以上样本。

协议编解码和 Diagnostics Selector
统一由 protocol_codec.py 提供。

结果属于 QEMU 仿真基线，
不能直接代表真实 Cortex-M4 硬件性能。
"""

import select
import socket
import time

from protocol_codec import (
    CMD_DIAGNOSTICS_RESPONSE,
    DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS,
    DIAGNOSTICS_METRIC_PARSER_MAX_TICKS,
    DIAGNOSTICS_METRIC_PARSER_MIN_TICKS,
    DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT,
    DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS,
    DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS,
    DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS,
    DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT,
    DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ,
    build_diagnostics_request,
    extract_frames,
    parse_diagnostics_response_payload,
    parse_frame,
)


HOST = "127.0.0.1"

PORT = 5555


# ==========================================================
# Diagnostics Query
# ==========================================================

def query_metric(
    sock: socket.socket,
    rx_buffer: bytearray,
    selector: int,
) -> int:
    request = (
        build_diagnostics_request(
            selector
        )
    )

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
                command
                == CMD_DIAGNOSTICS_RESPONSE
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
        f"无法读取 Diagnostics Metric {selector}"
    )


# ==========================================================
# Time Conversion
# ==========================================================

def ticks_to_us(
    ticks: int,
    frequency_hz: int,
) -> float:
    if frequency_hz <= 0:
        return 0.0

    return (
        ticks
        * 1_000_000.0
        / frequency_hz
    )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
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

    try:
        timer_hz = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_TIMER_FREQUENCY_HZ,
        )

        parser_samples = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_PARSER_SAMPLE_COUNT,
        )

        parser_min = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_PARSER_MIN_TICKS,
        )

        parser_avg = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_PARSER_AVERAGE_TICKS,
        )

        parser_max = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_PARSER_MAX_TICKS,
        )

        wake_samples = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_TASK_WAKEUP_SAMPLE_COUNT,
        )

        wake_min = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_TASK_WAKEUP_MIN_TICKS,
        )

        wake_avg = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_TASK_WAKEUP_AVERAGE_TICKS,
        )

        wake_max = query_metric(
            sock,
            rx_buffer,
            DIAGNOSTICS_METRIC_TASK_WAKEUP_MAX_TICKS,
        )

        print()
        print(
            "=============================================="
        )
        print(
            "Cortex-M4 Internal Performance Baseline"
        )
        print(
            "=============================================="
        )

        print(
            f"Timer Frequency : "
            f"{timer_hz} Hz"
        )

        print()

        print(
            "Protocol Parser / Full Frame"
        )

        print(
            f"Samples : {parser_samples}"
        )

        print(
            f"Minimum : "
            f"{parser_min} ticks / "
            f"{ticks_to_us(parser_min, timer_hz):.3f} us"
        )

        print(
            f"Average : "
            f"{parser_avg} ticks / "
            f"{ticks_to_us(parser_avg, timer_hz):.3f} us"
        )

        print(
            f"Maximum : "
            f"{parser_max} ticks / "
            f"{ticks_to_us(parser_max, timer_hz):.3f} us"
        )

        print()

        print(
            "UART ISR -> ProtocolRX Task Wakeup"
        )

        print(
            f"Samples : {wake_samples}"
        )

        print(
            f"Minimum : "
            f"{wake_min} ticks / "
            f"{ticks_to_us(wake_min, timer_hz):.3f} us"
        )

        print(
            f"Average : "
            f"{wake_avg} ticks / "
            f"{ticks_to_us(wake_avg, timer_hz):.3f} us"
        )

        print(
            f"Maximum : "
            f"{wake_max} ticks / "
            f"{ticks_to_us(wake_max, timer_hz):.3f} us"
        )

        print()

        print(
            "说明：以上属于 QEMU MPS2-AN386 仿真基线，"
        )

        print(
            "不能直接视为真实 Cortex-M4 硬件执行时间。"
        )

        print(
            "=============================================="
        )

    finally:
        sock.close()


if __name__ == "__main__":
    main()