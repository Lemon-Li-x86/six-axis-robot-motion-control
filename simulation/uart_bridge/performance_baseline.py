"""
文件：performance_baseline.py

用途：
测量当前 QEMU Cortex-M4 UART 通信链路的端到端往返延迟 RTT。

测量路径：

Python
    ↓
TCP / QEMU UART
    ↓
UART RX Interrupt
    ↓
Ring Buffer
    ↓
FreeRTOS Task Notification
    ↓
Protocol FSM
    ↓
ACK 构造
    ↓
UART TX
    ↓
QEMU / TCP
    ↓
Python

说明：

本测试测量的是当前仿真系统的端到端通信性能，
不是实际 Cortex-M4 硬件上的纯 CPU 执行时间。

运行本程序时不要同时运行：

    ur5_uart_bridge.py
    protocol_fault_test.py
"""

import select
import socket
import statistics
import struct
import time


# ==========================================================
# TCP 配置
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555


# ==========================================================
# 协议配置
# ==========================================================

HEADER = b"\xAA\x55"

CMD_JOINT_STATE = 0x81
CMD_JOINT_STATE_ACK = 0x82

JOINT_COUNT = 6


# ==========================================================
# 测试配置
# ==========================================================

WARMUP_COUNT = 20

TEST_COUNT = 200

ACK_TIMEOUT = 1.0


# ==========================================================
# 构造通用协议帧
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
# 构造 JOINT_STATE 帧
# ==========================================================

def build_joint_state_frame(
    sequence: int
) -> tuple[bytes, bytes]:

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
        *joints
    )

    frame = build_frame(
        CMD_JOINT_STATE,
        payload
    )

    return frame, payload


# ==========================================================
# 从连续字节流提取协议帧
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


# ==========================================================
# 解析协议帧
# ==========================================================

def parse_frame(
    frame: bytes
):

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


    if (
        received_checksum
        != calculated_checksum
    ):
        return None, None


    payload = frame[4:-1]


    return command, payload


# ==========================================================
# 等待当前测试对应的 ACK
# ==========================================================

def wait_for_matching_ack(
    sock: socket.socket,
    rx_buffer: bytearray,
    expected_payload: bytes,
    timeout: float
) -> bool:

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
            remaining
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


            # Cortex-M4 自己还会周期发送 0x01。
            #
            # 因此这里只认：
            #
            # Command = 0x82
            # 并且 Payload 与当前测试完全一致。
            if (
                command
                == CMD_JOINT_STATE_ACK
                and payload
                == expected_payload
            ):
                return True


    return False


# ==========================================================
# 百分位数
# ==========================================================

def percentile(
    values: list[float],
    percentage: float
) -> float:

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
        len(ordered) - 1
    )


    fraction = (
        position
        - lower_index
    )


    return (
        ordered[lower_index]
        * (1.0 - fraction)
        +
        ordered[upper_index]
        * fraction
    )


# ==========================================================
# 单次 RTT 测量
# ==========================================================

def measure_once(
    sock: socket.socket,
    rx_buffer: bytearray,
    sequence: int
):

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
        ACK_TIMEOUT
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


    return (
        elapsed_ns
        / 1_000_000.0
    )


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


    # 关闭 Nagle Algorithm，
    # 减少 localhost 小数据包额外等待。
    sock.setsockopt(
        socket.IPPROTO_TCP,
        socket.TCP_NODELAY,
        1
    )


    rx_buffer = bytearray()


    print("连接成功。")
    print()


    try:

        # ==================================================
        # 预热
        # ==================================================

        print(
            f"预热：{WARMUP_COUNT} 次"
        )


        for i in range(
            WARMUP_COUNT
        ):

            # 预热使用 1000~1019。
            #
            # 正式测试从 2000 开始，
            # 避免 Payload 重复。
            result = measure_once(
                sock,
                rx_buffer,
                1000 + i
            )


            if result is None:

                print(
                    f"[WARN] "
                    f"预热 {i + 1} 未收到 ACK"
                )


        print("预热完成。")
        print()


        # ==================================================
        # 正式 RTT 测试
        # ==================================================

        print(
            f"开始 RTT 测试："
            f"{TEST_COUNT} 次"
        )


        samples_ms = []

        timeout_count = 0


        for i in range(
            TEST_COUNT
        ):

            result = measure_once(
                sock,
                rx_buffer,
                2000 + i
            )


            if result is None:

                timeout_count += 1


                print(
                    f"[TIMEOUT] "
                    f"{i + 1}/{TEST_COUNT}"
                )


                continue


            samples_ms.append(
                result
            )


            if (
                (i + 1) % 20
                == 0
            ):

                print(
                    f"进度："
                    f"{i + 1}/{TEST_COUNT}"
                )


        # ==================================================
        # 结果统计
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
            0.95
        )

        p99_ms = percentile(
            samples_ms,
            0.99
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