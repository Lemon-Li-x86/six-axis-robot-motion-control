import math
import select
import socket
import struct
import time
from pathlib import Path

import pybullet as p
import pybullet_data


# ==========================================================
# 1. TCP / 协议参数
# ==========================================================

HOST = "127.0.0.1"
PORT = 5555

HEADER = b"\xAA\x55"

# Cortex-M4 -> Python
CMD_SET_JOINT_TARGETS = 0x01

# Python -> Cortex-M4
CMD_JOINT_STATE = 0x81

# Cortex-M4 -> Python
# 表示 Cortex-M4 已成功接收并解析 JOINT_STATE
CMD_JOINT_STATE_ACK = 0x82

JOINT_COUNT = 6
JOINT_PAYLOAD_LEN = 12

FRAME_LEN = 17


# ==========================================================
# 2. UR5 六个运动关节名称
# ==========================================================

UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ==========================================================
# 3. 找到 UR5 模型
# ==========================================================

current_file = Path(__file__).resolve()

# 当前文件：
# simulation/uart_bridge/ur5_uart_bridge.py
#
# parent.parent = simulation/
simulation_dir = current_file.parent.parent

ur5_path = (
    simulation_dir
    / "pybullet_ur5"
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)

print("UR5 path:", ur5_path)
print("UR5 exists:", ur5_path.exists())


# ==========================================================
# 4. 构造一个六轴关节数据帧
# ==========================================================

def build_joint_frame(command: int, angles_deg: list[float]) -> bytes:
    """
    根据 6 个关节角构造协议帧。

    angles_deg：
        单位为 degree。

    协议表示：
        int16
        1 = 0.01°
    """

    raw_angles = []

    for angle in angles_deg:

        # degree -> 0.01 degree
        raw_value = int(round(angle * 100.0))

        # 当前协议使用 int16
        # 防止超出表示范围
        raw_value = max(
            -32768,
            min(32767, raw_value)
        )

        raw_angles.append(raw_value)


    # <  = little-endian
    # 6h = 6 个 signed int16
    payload = struct.pack(
        "<6h",
        *raw_angles
    )


    frame_without_checksum = (
        HEADER
        + bytes([
            command,
            len(payload)
        ])
        + payload
    )


    # checksum 不包含 AA 55
    checksum = (
        sum(frame_without_checksum[2:])
        & 0xFF
    )


    return (
        frame_without_checksum
        + bytes([checksum])
    )


# ==========================================================
# 5. 解析一帧
# ==========================================================

def parse_frame(frame: bytes):
    """
    返回：

        command,
        angles_deg

    失败时返回：

        None,
        None
    """

    if len(frame) < 5:
        return None, None


    # 帧头
    if frame[0:2] != HEADER:
        return None, None


    command = frame[2]
    payload_length = frame[3]


    # 根据 Length 检查实际帧长度
    expected_length = (
        2
        + 1
        + 1
        + payload_length
        + 1
    )

    if len(frame) != expected_length:
        return None, None


    payload = frame[4:-1]

    received_checksum = frame[-1]


    # Command + Length + Payload
    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )


    if received_checksum != calculated_checksum:

        print(
            "Checksum 校验失败:",
            frame.hex(" ")
        )

        return None, None


    # 当前三个命令都使用 6 × int16 payload
    if (
        payload_length == JOINT_PAYLOAD_LEN
        and command in (
            CMD_SET_JOINT_TARGETS,
            CMD_JOINT_STATE,
            CMD_JOINT_STATE_ACK,
        )
    ):

        raw_angles = struct.unpack(
            "<6h",
            payload
        )

        angles_deg = [
            value / 100.0
            for value in raw_angles
        ]

        return command, angles_deg


    return command, None


# ==========================================================
# 6. 从 TCP 字节流中提取完整帧
# ==========================================================

def extract_frames(buffer: bytearray):

    frames = []


    while True:

        # AA 55 CMD LEN
        if len(buffer) < 4:
            break


        # --------------------------------------------------
        # 找帧头
        # --------------------------------------------------

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


        # 数据还没收完整
        if len(buffer) < frame_length:
            break


        frame = bytes(
            buffer[:frame_length]
        )

        del buffer[:frame_length]

        frames.append(frame)


    return frames


# ==========================================================
# 7. 连接 QEMU
# ==========================================================

print()
print(
    f"正在连接 QEMU UART: "
    f"{HOST}:{PORT}"
)


while True:

    try:

        sock = socket.create_connection(
            (HOST, PORT)
        )

        break

    except ConnectionRefusedError:

        print(
            "QEMU 尚未监听，0.5 秒后重试..."
        )

        time.sleep(0.5)


print("已连接到 QEMU UART。")


# ==========================================================
# 8. 启动 PyBullet
# ==========================================================

physics_client = p.connect(
    p.GUI
)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(
    0,
    0,
    -9.81
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.4]
)

p.loadURDF(
    "plane.urdf"
)


# ==========================================================
# 9. 加载 UR5
# ==========================================================

robot_id = p.loadURDF(
    str(ur5_path),
    basePosition=[0, 0, 0],
    useFixedBase=True
)

print("Robot ID:", robot_id)


# ==========================================================
# 10. 建立 joint name -> index 映射
# ==========================================================

joint_map = {}

joint_count = p.getNumJoints(
    robot_id
)


for joint_index in range(joint_count):

    joint_info = p.getJointInfo(
        robot_id,
        joint_index
    )

    joint_name = joint_info[1].decode(
        "utf-8"
    )

    joint_map[joint_name] = joint_index


controlled_joint_indices = [
    joint_map[name]
    for name in UR5_JOINT_NAMES
]


print(
    "Controlled joint indices:",
    controlled_joint_indices
)


# ==========================================================
# 11. 初始目标角
# ==========================================================

target_positions = [
    0.0
] * JOINT_COUNT


# TCP 接收缓冲区
rx_buffer = bytearray()


# ==========================================================
# 12. 状态反馈周期
# ==========================================================

# 每 0.2 秒发送一次实际关节状态
# 即 5 Hz
STATE_TX_INTERVAL = 0.2

last_state_tx_time = time.monotonic()

# 控制终端状态打印频率
last_state_print_time = time.monotonic()


print()
print("======================================")
print("UART <-> PyBullet 双向控制桥已启动")
print("======================================")
print()


# ==========================================================
# 13. 主循环
# ==========================================================

try:

    while True:

        # --------------------------------------------------
        # A. 检查 QEMU 是否发送了数据
        #
        # select timeout = 0
        # 不阻塞 PyBullet 仿真
        # --------------------------------------------------

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            0
        )


        if readable:

            data = sock.recv(
                1024
            )


            if not data:

                print(
                    "QEMU 已断开连接。"
                )

                break


            rx_buffer.extend(
                data
            )


        # --------------------------------------------------
        # B. 提取完整协议帧
        # --------------------------------------------------

        frames = extract_frames(
            rx_buffer
        )


        for frame in frames:

            command, angles_deg = parse_frame(
                frame
            )


            # ==============================================
            # Cortex-M4 -> Python
            # 目标角命令
            # ==============================================

            if (
                command == CMD_SET_JOINT_TARGETS
                and angles_deg is not None
            ):

                print(
                    "RX 0x01 目标角:",
                    angles_deg
                )


                # degree -> rad
                target_positions = [
                    math.radians(angle)
                    for angle in angles_deg
                ]


            # ==============================================
            # Cortex-M4 -> Python
            # MCU 对状态帧的 ACK
            # ==============================================

            elif (
                command == CMD_JOINT_STATE_ACK
                and angles_deg is not None
            ):

                print(
                    "RX 0x82 MCU 已确认状态:",
                    angles_deg
                )


        # --------------------------------------------------
        # C. 控制 UR5 六个关节
        # --------------------------------------------------

        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,

            jointIndices=controlled_joint_indices,

            controlMode=p.POSITION_CONTROL,

            targetPositions=target_positions,

            forces=[
                150,
                150,
                150,
                28,
                28,
                28,
            ]
        )


        # --------------------------------------------------
        # D. 推进物理仿真
        # --------------------------------------------------

        p.stepSimulation()


        # --------------------------------------------------
        # E. 周期读取 UR5 实际关节状态
        # --------------------------------------------------

        now = time.monotonic()


        if (
            now - last_state_tx_time
            >= STATE_TX_INTERVAL
        ):

            actual_angles_deg = []


            for joint_index in controlled_joint_indices:

                joint_state = p.getJointState(
                    robot_id,
                    joint_index
                )

                actual_rad = joint_state[0]

                actual_deg = math.degrees(
                    actual_rad
                )

                actual_angles_deg.append(
                    actual_deg
                )


            # ----------------------------------------------
            # 构造 Python -> MCU 的 0x81 状态帧
            # ----------------------------------------------

            state_frame = build_joint_frame(
                CMD_JOINT_STATE,
                actual_angles_deg
            )


            # ----------------------------------------------
            # 发回 QEMU UART
            # ----------------------------------------------

            sock.sendall(
                state_frame
            )


            last_state_tx_time = now


            # 每约 1 秒打印一次，
            # 不然终端会变成瀑布。
            if (
                now - last_state_print_time
                >= 1.0
            ):

                rounded_angles = [
                    round(angle, 2)
                    for angle in actual_angles_deg
                ]

                print(
                    "TX 0x81 实际角度:",
                    rounded_angles
                )

                last_state_print_time = now


        # --------------------------------------------------
        # F. 约 240 Hz 仿真
        # --------------------------------------------------

        time.sleep(
            1.0 / 240.0
        )


# ==========================================================
# 14. 退出
# ==========================================================

except KeyboardInterrupt:

    print()
    print("控制桥停止。")


finally:

    sock.close()

    p.disconnect()