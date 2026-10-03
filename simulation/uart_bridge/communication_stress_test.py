"""
文件：communication_stress_test.py

用途：
对当前 QEMU Cortex-M4 UART 通信链路进行连续高频压力测试。

测试内容：

1. 按指定目标速率连续发送 JOINT_STATE（0x81）帧；
2. 统计 MCU 返回的 JOINT_STATE_ACK（0x82）；
3. 统计成功帧数和丢失帧数；
4. 测量实际完成发送所需时间；
5. 根据实际发送时间计算真实平均发送速率。

注意：

TEST_RATES_HZ 表示“目标发送速率”。

Windows + Python 并不能保证严格按照该频率调度，
因此最终需要同时观察 actual_rate_hz。

本测试反映的是：

QEMU + TCP + Windows + Python + FreeRTOS 固件

组成的完整仿真系统压力表现。

不能直接作为真实 Cortex-M4 硬件吞吐能力。

运行时不要同时运行：

    ur5_uart_bridge.py
    protocol_fault_test.py
    performance_baseline.py
"""

import select
import socket
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

# Python -> Cortex-M4
CMD_JOINT_STATE = 0x81

# Cortex-M4 -> Python
CMD_JOINT_STATE_ACK = 0x82


# ==========================================================
# 压力测试配置
# ==========================================================

# 目标发送速率。
#
# 后续输出中会同时显示实际达到的平均速率。
TEST_RATES_HZ = [
    1000,
    2000,
    5000,
    10000,
]


# 每个速率档位发送多少帧。
FRAMES_PER_RATE = 1000


# 全部帧发送完成以后，
# 最多再等待多少秒，让剩余 ACK 返回。
ACK_DRAIN_TIMEOUT = 2.0


# 每个档位完成以后稍作等待，
# 避免上一档测试影响下一档。
RATE_TEST_GAP = 0.5


# ==========================================================
# 构造通用协议帧
# ==========================================================

def build_frame(
    command: int,
    payload: bytes
) -> bytes:
    """
    构造协议帧：

        AA 55
        Command
        Length
        Payload
        Checksum

    Checksum：

        Command + Length + Payload

    最后取低 8 bit。
    """

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
# 构造带序号的 JOINT_STATE
# ==========================================================

def build_joint_state_frame(
    sequence: int
) -> tuple[bytes, bytes]:
    """
    使用 sequence 构造唯一 Payload。

    这样收到 ACK 时，
    可以判断具体是哪一帧的 ACK，
    避免把旧 ACK 误认为当前帧。
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
        *joints
    )

    frame = build_frame(
        CMD_JOINT_STATE,
        payload
    )

    return frame, payload


# ==========================================================
# 从 TCP 连续字节流中提取完整协议帧
# ==========================================================

def extract_frames(
    buffer: bytearray
) -> list[bytes]:

    frames = []

    while True:

        # 至少需要：
        #
        # AA 55 CMD LEN
        #
        if len(buffer) < 4:
            break


        # --------------------------------------------------
        # 搜索帧头
        # --------------------------------------------------

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


        # 当前还没有收到完整一帧。
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
# 验证并解析协议帧
# ==========================================================

def parse_frame(
    frame: bytes
):
    """
    成功：

        返回 command, payload

    失败：

        返回 None, None
    """

    if len(frame) < 5:
        return None, None


    if frame[0:2] != HEADER:
        return None, None


    command = (
        frame[2]
    )


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


    payload = (
        frame[4:-1]
    )


    return command, payload


# ==========================================================
# 处理当前 socket 已经收到的数据
# ==========================================================

def receive_current_data(
    sock: socket.socket,
    rx_buffer: bytearray,
    received_payloads: set[bytes]
):
    """
    非阻塞读取当前已经到达 socket 的数据。

    主要用于发送压力测试过程中，
    一边发送一边及时读取 MCU 返回的 ACK，
    避免主机 TCP 接收缓冲区堆积。
    """

    while True:

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            0
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

            command, payload = (
                parse_frame(
                    frame
                )
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
# 等待剩余 ACK
# ==========================================================

def drain_remaining_acks(
    sock: socket.socket,
    rx_buffer: bytearray,
    received_payloads: set[bytes],
    expected_payloads: set[bytes],
    timeout: float
):
    """
    所有测试帧发送完成以后，
    等待仍在链路上的 ACK。

    如果所有预期 ACK 都已经收到，
    会提前结束，不必等待完整 timeout。
    """

    deadline = (
        time.perf_counter()
        + timeout
    )


    while (
        time.perf_counter()
        < deadline
    ):

        # 已经全部收到。
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
                0.01
            )
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

            command, payload = (
                parse_frame(
                    frame
                )
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
# 测试单个目标发送速率
# ==========================================================

def run_rate_test(
    sock: socket.socket,
    rx_buffer: bytearray,
    rate_hz: int,
    sequence_start: int
):
    """
    在指定目标频率下发送 FRAMES_PER_RATE 个协议帧。

    返回：

        success_count
        lost_count
        success_rate
        send_elapsed
        actual_rate_hz
    """

    interval = (
        1.0 / rate_hz
    )


    expected_payloads = set()

    received_payloads = set()


    # ------------------------------------------------------
    # 记录整个发送阶段的实际开始时间
    # ------------------------------------------------------

    send_start_time = (
        time.perf_counter()
    )


    next_send_time = (
        send_start_time
    )


    # ======================================================
    # 连续发送
    # ======================================================

    for i in range(
        FRAMES_PER_RATE
    ):

        sequence = (
            sequence_start + i
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


            # 时间还比较充裕时，
            # 先交给操作系统 sleep。
            #
            # 接近发送时刻以后，
            # 再使用短时间忙等待，
            # 尽量减少 Windows sleep 精度带来的误差。
            if remaining > 0.001:

                time.sleep(
                    remaining - 0.0005
                )


        # --------------------------------------------------
        # 发送当前协议帧
        # --------------------------------------------------

        sock.sendall(
            frame
        )


        # 下一帧理论发送时间。
        next_send_time += (
            interval
        )


        # --------------------------------------------------
        # 顺便消费已经返回的 ACK
        # --------------------------------------------------

        receive_current_data(
            sock,
            rx_buffer,
            received_payloads
        )


    # ------------------------------------------------------
    # 记录所有测试帧完成 sendall 的时间
    # ------------------------------------------------------

    send_end_time = (
        time.perf_counter()
    )


    send_elapsed = (
        send_end_time
        - send_start_time
    )


    # ------------------------------------------------------
    # 实际平均发送速率
    # ------------------------------------------------------

    if send_elapsed > 0.0:

        actual_rate_hz = (
            FRAMES_PER_RATE
            / send_elapsed
        )

    else:

        actual_rate_hz = 0.0


    # ======================================================
    # 等待最后仍在链路中的 ACK
    # ======================================================

    drain_remaining_acks(
        sock,
        rx_buffer,
        received_payloads,
        expected_payloads,
        ACK_DRAIN_TIMEOUT
    )


    # ======================================================
    # 统计结果
    # ======================================================

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
        actual_rate_hz
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


    # 关闭 Nagle Algorithm。
    #
    # 当前运行在 localhost，
    # 仍显式关闭 TCP 小包聚合，
    # 避免它影响延迟和压力测试。
    sock.setsockopt(
        socket.IPPROTO_TCP,
        socket.TCP_NODELAY,
        1
    )


    rx_buffer = bytearray()


    print("连接成功。")

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


        # 每个档位使用不同的数据范围，
        # 避免不同测试之间出现相同 Payload。
        sequence_start = 5000


        # ==================================================
        # 逐档测试
        # ==================================================

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
                actual_rate_hz
            ) = run_rate_test(
                sock,
                rx_buffer,
                rate_hz,
                sequence_start
            )


            results.append(
                (
                    rate_hz,
                    actual_rate_hz,
                    send_elapsed,
                    success_count,
                    lost_count,
                    success_rate
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


            # 下一个测试档位使用新的 Payload 区间。
            sequence_start += (
                FRAMES_PER_RATE
                + 100
            )


            # 避免上一档测试残余数据影响下一档。
            time.sleep(
                RATE_TEST_GAP
            )


        # ==================================================
        # 汇总
        # ==================================================

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
            success_rate
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