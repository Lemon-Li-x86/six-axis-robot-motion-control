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
# 2. Joint Angle Convention
# ==========================================================

# UART Protocol：
#
# int16
# 1 unit = 0.01 degree
#
# Canonical Angle：
#
# [-180°, 180°)

ANGLE_FULL_TURN_DEG = 360.0
ANGLE_HALF_TURN_DEG = 180.0

ANGLE_FULL_TURN_RAW = 36000
ANGLE_HALF_TURN_RAW = 18000


def normalize_angle_deg(angle_deg: float) -> float:
    """
    将任意角度规范化到：

        [-180°, 180°)

    例如：

        190°  -> -170°
        350°  ->  -10°
        180°  -> -180°
    """

    normalized = (
        (angle_deg + ANGLE_HALF_TURN_DEG)
        % ANGLE_FULL_TURN_DEG
    ) - ANGLE_HALF_TURN_DEG

    return normalized


def unwrap_angle_deg(
    canonical_angle_deg: float,
    reference_angle_deg: float,
) -> float:
    """
    根据上一连续角度 reference，
    将 Canonical Angle 恢复成距离 reference
    最近的等价连续角。

    例如：

        reference = 179°
        canonical = -179°

    返回：

        181°

    而不是：

        -179°
    """

    delta = normalize_angle_deg(
        canonical_angle_deg
        -
        reference_angle_deg
    )

    return (
        reference_angle_deg
        +
        delta
    )


# ==========================================================
# 3. UR5 六个运动关节名称
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
# 4. 找到 UR5 模型
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
# 5. 构造一个六轴关节数据帧
# ==========================================================

def build_joint_frame(
    command: int,
    angles_deg: list[float],
) -> bytes:
    """
    根据 6 个关节角构造协议帧。

    angles_deg：
        单位为 degree。

    协议表示：
        int16
        1 = 0.01°

    发送前统一规范化到：

        [-180°, 180°)

    不使用数值 Clamp。

    例如：

        350° -> -10°
    """

    raw_angles = []


    for angle in angles_deg:

        # degree -> 0.01 degree
        raw_value = int(
            round(
                angle * 100.0
            )
        )


        # --------------------------------------------------
        # Canonical Normalize
        #
        # 任意整数角规范化到：
        #
        # [-18000, 18000)
        #
        # 即：
        #
        # [-180°, 180°)
        # --------------------------------------------------

        raw_value = (
            (
                raw_value
                +
                ANGLE_HALF_TURN_RAW
            )
            %
            ANGLE_FULL_TURN_RAW
        ) - ANGLE_HALF_TURN_RAW


        raw_angles.append(
            raw_value
        )


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
# 6. 解析一帧
# ==========================================================

def parse_frame(frame: bytes):
    """
    返回：

        command,
        angles_deg

    Joint Angle 返回值统一为：

        [-180°, 180°)

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


    calculated_checksum = (
        sum(frame[2:-1])
        & 0xFF
    )


    if (
        received_checksum
        != calculated_checksum
    ):

        print(
            "Checksum 校验失败:",
            frame.hex(" ")
        )

        return None, None


    if (
        payload_length
        == JOINT_PAYLOAD_LEN
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
            normalize_angle_deg(
                value / 100.0
            )
            for value in raw_angles
        ]


        return (
            command,
            angles_deg
        )


    return command, None


# ==========================================================
# 7. 从 TCP 字节流中提取完整帧
# ==========================================================

def extract_frames(
    buffer: bytearray
):

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


        if (
            len(buffer)
            <
            frame_length
        ):
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
# 8. 连接 QEMU
# ==========================================================

print()

print(
    f"正在连接 QEMU UART: "
    f"{HOST}:{PORT}"
)


while True:

    try:

        sock = socket.create_connection(
            (
                HOST,
                PORT
            )
        )

        break

    except ConnectionRefusedError:

        print(
            "QEMU 尚未监听，0.5 秒后重试..."
        )

        time.sleep(
            0.5
        )


print(
    "已连接到 QEMU UART。"
)


# ==========================================================
# 9. 启动 PyBullet
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
    cameraTargetPosition=[
        0,
        0,
        0.4
    ]
)


p.loadURDF(
    "plane.urdf"
)


# ==========================================================
# 10. 加载 UR5
# ==========================================================

robot_id = p.loadURDF(
    str(ur5_path),
    basePosition=[
        0,
        0,
        0
    ],
    useFixedBase=True
)


print(
    "Robot ID:",
    robot_id
)


# ==========================================================
# 11. 建立 joint name -> index 映射
# ==========================================================

joint_map = {}


joint_count = p.getNumJoints(
    robot_id
)


for joint_index in range(
    joint_count
):

    joint_info = p.getJointInfo(
        robot_id,
        joint_index
    )


    joint_name = (
        joint_info[1]
        .decode(
            "utf-8"
        )
    )


    joint_map[
        joint_name
    ] = joint_index


controlled_joint_indices = [
    joint_map[name]
    for name
    in UR5_JOINT_NAMES
]


print(
    "Controlled joint indices:",
    controlled_joint_indices
)


# ==========================================================
# 12. 初始目标角
# ==========================================================

# 连续目标角，单位 degree。
#
# 不强制限制在 ±180°。
#
# 当 UART 收到 Canonical Angle 时，
# 会根据前一个目标值恢复最近的连续等价角。
target_positions_deg = [
    0.0
] * JOINT_COUNT


target_positions = [
    0.0
] * JOINT_COUNT


# TCP 接收缓冲区
rx_buffer = bytearray()


# ==========================================================
# 13. 状态反馈周期
# ==========================================================

# 每 0.2 秒发送一次实际关节状态
# 即 5 Hz
STATE_TX_INTERVAL = 0.2


last_state_tx_time = (
    time.monotonic()
)


# 控制终端状态打印频率
last_state_print_time = (
    time.monotonic()
)


print()
print(
    "======================================"
)
print(
    "UART <-> PyBullet 双向控制桥已启动"
)
print(
    "======================================"
)
print()


# ==========================================================
# 14. 主循环
# ==========================================================

try:

    while True:

        # --------------------------------------------------
        # A. 检查 QEMU 是否发送了数据
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

            command, angles_deg = (
                parse_frame(
                    frame
                )
            )


            # ==============================================
            # Cortex-M4 -> Python
            # 目标角命令
            # ==============================================

            if (
                command
                ==
                CMD_SET_JOINT_TARGETS
                and
                angles_deg is not None
            ):

                print(
                    "RX 0x01 Canonical 目标角:",
                    angles_deg
                )


                new_target_positions_deg = []


                for i in range(
                    JOINT_COUNT
                ):

                    continuous_target = (
                        unwrap_angle_deg(
                            angles_deg[i],
                            target_positions_deg[i]
                        )
                    )


                    new_target_positions_deg.append(
                        continuous_target
                    )


                target_positions_deg = (
                    new_target_positions_deg
                )


                target_positions = [
                    math.radians(
                        angle
                    )
                    for angle
                    in target_positions_deg
                ]


                print(
                    "PyBullet Continuous 目标角:",
                    [
                        round(
                            value,
                            2
                        )
                        for value
                        in target_positions_deg
                    ]
                )


            # ==============================================
            # Cortex-M4 -> Python
            # MCU 对状态帧的 ACK
            # ==============================================

            elif (
                command
                ==
                CMD_JOINT_STATE_ACK
                and
                angles_deg is not None
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

            jointIndices=
                controlled_joint_indices,

            controlMode=
                p.POSITION_CONTROL,

            targetPositions=
                target_positions,

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
            now
            -
            last_state_tx_time
            >=
            STATE_TX_INTERVAL
        ):

            actual_angles_deg = []


            for joint_index in (
                controlled_joint_indices
            ):

                joint_state = (
                    p.getJointState(
                        robot_id,
                        joint_index
                    )
                )


                actual_rad = (
                    joint_state[0]
                )


                actual_deg = (
                    math.degrees(
                        actual_rad
                    )
                )


                actual_angles_deg.append(
                    actual_deg
                )


            # ----------------------------------------------
            # PyBullet Continuous Angle
            # ->
            # Canonical Angle
            #
            # UART 只传输：
            #
            # [-180°, 180°)
            # ----------------------------------------------

            canonical_state_angles_deg = [
                normalize_angle_deg(
                    angle
                )
                for angle
                in actual_angles_deg
            ]


            # ----------------------------------------------
            # 构造 Python -> MCU 的 0x81 状态帧
            # ----------------------------------------------

            state_frame = build_joint_frame(
                CMD_JOINT_STATE,
                canonical_state_angles_deg
            )


            # ----------------------------------------------
            # 发回 QEMU UART
            # ----------------------------------------------

            sock.sendall(
                state_frame
            )


            last_state_tx_time = (
                now
            )


            # 每约 1 秒打印一次
            if (
                now
                -
                last_state_print_time
                >=
                1.0
            ):

                rounded_angles = [
                    round(
                        angle,
                        2
                    )
                    for angle
                    in canonical_state_angles_deg
                ]


                print(
                    "TX 0x81 Canonical 实际角度:",
                    rounded_angles
                )


                last_state_print_time = (
                    now
                )


        # --------------------------------------------------
        # F. 约 240 Hz 仿真
        # --------------------------------------------------

        time.sleep(
            1.0 / 240.0
        )


# ==========================================================
# 15. 退出
# ==========================================================

except KeyboardInterrupt:

    print()
    print(
        "控制桥停止。"
    )


finally:

    sock.close()

    p.disconnect()