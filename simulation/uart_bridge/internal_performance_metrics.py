"""
文件：internal_performance_metrics.py

用途：
读取 Cortex-M4 固件内部性能基线。

建议运行顺序：

1. 重启 QEMU；
2. 运行 performance_baseline.py；
3. 再运行本程序。

这样 Task Wakeup 已经积累约 200 个以上样本。

结果属于 QEMU 仿真基线，
不能直接代表真实 Cortex-M4 硬件性能。
"""

import select
import socket
import struct
import time


HOST = "127.0.0.1"

PORT = 5555


HEADER = b"\xAA\x55"

CMD_GET_DIAGNOSTICS = 0x83

CMD_DIAGNOSTICS_RESPONSE = 0x84


METRIC_TIMER_HZ = 1

METRIC_PARSER_SAMPLES = 2

METRIC_PARSER_MIN = 3

METRIC_PARSER_AVG = 4

METRIC_PARSER_MAX = 5

METRIC_WAKE_SAMPLES = 6

METRIC_WAKE_MIN = 7

METRIC_WAKE_AVG = 8

METRIC_WAKE_MAX = 9


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


def parse_frame(
    frame: bytes
):

    if len(frame) < 5:
        return None, None


    if frame[0:2] != HEADER:
        return None, None


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
        frame[2],
        frame[4:-1]
    )


def query_metric(
    sock: socket.socket,
    rx_buffer: bytearray,
    selector: int
) -> int:

    request = build_frame(
        CMD_GET_DIAGNOSTICS,
        bytes([selector])
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

            command, payload = parse_frame(
                frame
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
        f"无法读取 Diagnostics Metric {selector}"
    )


def ticks_to_us(
    ticks: int,
    frequency_hz: int
) -> float:

    if frequency_hz <= 0:
        return 0.0


    return (
        ticks
        * 1_000_000.0
        / frequency_hz
    )


def main():

    sock = socket.create_connection(
        (
            HOST,
            PORT
        )
    )


    sock.setsockopt(
        socket.IPPROTO_TCP,
        socket.TCP_NODELAY,
        1
    )


    rx_buffer = bytearray()


    try:

        timer_hz = query_metric(
            sock,
            rx_buffer,
            METRIC_TIMER_HZ
        )


        parser_samples = query_metric(
            sock,
            rx_buffer,
            METRIC_PARSER_SAMPLES
        )


        parser_min = query_metric(
            sock,
            rx_buffer,
            METRIC_PARSER_MIN
        )


        parser_avg = query_metric(
            sock,
            rx_buffer,
            METRIC_PARSER_AVG
        )


        parser_max = query_metric(
            sock,
            rx_buffer,
            METRIC_PARSER_MAX
        )


        wake_samples = query_metric(
            sock,
            rx_buffer,
            METRIC_WAKE_SAMPLES
        )


        wake_min = query_metric(
            sock,
            rx_buffer,
            METRIC_WAKE_MIN
        )


        wake_avg = query_metric(
            sock,
            rx_buffer,
            METRIC_WAKE_AVG
        )


        wake_max = query_metric(
            sock,
            rx_buffer,
            METRIC_WAKE_MAX
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