"""
文件：ur5_uart_bridge.py

用途：
连接：

    QEMU Cortex-M4 / FreeRTOS UART
    <->
    Python TCP Bridge
    <->
    PyBullet UR5

本模块负责：

1. QEMU UART TCP 连接；
2. PyBullet UR5 生命周期；
3. Protocol Frame Application Handling；
4. 六轴目标位置下发；
5. PyBullet 实际关节状态读取；
6. 周期发送 JOINT_STATE。

协议编解码统一由 protocol_codec.py 提供。

角度规范化和连续角恢复统一由
angle_utils.py 提供。
"""

import math
import select
import socket
import time
from pathlib import Path

import pybullet as p
import pybullet_data

from angle_utils import (
    normalize_angle_deg,
    unwrap_angle_deg,
)

from protocol_codec import (
    CMD_JOINT_STATE,
    CMD_JOINT_STATE_ACK,
    CMD_SET_JOINT_TARGETS,
    JOINT_COUNT,
    build_joint_frame,
    extract_frames,
    parse_joint_frame,
)


# ==========================================================
# TCP Configuration
# ==========================================================

# QEMU UART0 TCP Endpoint。
HOST = "127.0.0.1"
PORT = 5555

# QEMU 尚未启动时的重连间隔，单位 second。
SOCKET_RETRY_INTERVAL_S = 0.5


# ==========================================================
# Simulation Timing
# ==========================================================

# Python -> MCU Joint State 发送周期，单位 second。
#
# 0.2 s = 5 Hz。
STATE_TX_INTERVAL_S = 0.2

# Terminal 状态打印周期，单位 second。
STATE_PRINT_INTERVAL_S = 1.0

# PyBullet 基础仿真循环频率，单位 Hz。
SIMULATION_STEP_HZ = 240.0


# ==========================================================
# UR5 Joint Configuration
# ==========================================================

# 顺序必须与 Cortex-M4 六轴 Joint Array 一致。
UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# PyBullet Position Control 最大 Force。
#
# 前三轴使用较大输出力，
# Wrist 三轴使用较小输出力。
#
# 顺序与 UR5_JOINT_NAMES 一一对应。
UR5_JOINT_FORCES = [
    150,
    150,
    150,
    28,
    28,
    28,
]


# ==========================================================
# UR5 Path
# ==========================================================

def get_ur5_path() -> Path:
    """
    获取项目内 UR5 URDF 的绝对路径。

    Returns:
        UR5 ur5.urdf Path。
    """
    current_file = Path(
        __file__
    ).resolve()

    simulation_dir = (
        current_file.parent.parent
    )

    return (
        simulation_dir
        / "pybullet_ur5"
        / "models"
        / "ur5"
        / "urdf"
        / "ur5.urdf"
    )


# ==========================================================
# QEMU Connection
# ==========================================================

def connect_qemu(
    host: str,
    port: int,
) -> socket.socket:
    """
    持续尝试连接 QEMU UART TCP Server。

    Args:
        host:
            TCP Host。

        port:
            TCP Port。

    Returns:
        已建立连接的 socket。

    Note:
        QEMU 尚未监听时，
        每 SOCKET_RETRY_INTERVAL_S
        秒重新连接一次。
    """
    print()

    print(
        f"正在连接 QEMU UART: "
        f"{host}:{port}"
    )

    while True:
        try:
            sock = socket.create_connection(
                (
                    host,
                    port,
                )
            )

            print(
                "已连接到 QEMU UART。"
            )

            return sock

        except ConnectionRefusedError:
            print(
                "QEMU 尚未监听，"
                f"{SOCKET_RETRY_INTERVAL_S:.1f} 秒后重试..."
            )

            time.sleep(
                SOCKET_RETRY_INTERVAL_S
            )


# ==========================================================
# PyBullet Setup
# ==========================================================

def start_pybullet(
    ur5_path: Path,
):
    """
    创建 PyBullet GUI 并加载 UR5。

    Args:
        ur5_path:
            UR5 URDF 路径。

    Returns:
        Tuple：

        (
            robot_id,
            controlled_joint_indices,
        )

    Raises:
        RuntimeError:
            PyBullet GUI 启动失败，
            或 UR5 缺少需要控制的 Joint。
    """
    physics_client = p.connect(
        p.GUI
    )

    if physics_client < 0:
        raise RuntimeError(
            "无法启动 PyBullet GUI"
        )

    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )

    # 地球标准重力加速度，单位 m/s^2。
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
            0.4,
        ],
    )

    p.loadURDF(
        "plane.urdf"
    )

    robot_id = p.loadURDF(
        str(
            ur5_path
        ),

        # UR5 Base 与 World Origin 重合。
        basePosition=[
            0,
            0,
            0,
        ],

        useFixedBase=True,
    )

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

    missing_joint_names = [
        joint_name
        for joint_name in UR5_JOINT_NAMES
        if joint_name not in joint_map
    ]

    if missing_joint_names:
        raise RuntimeError(
            "UR5 模型缺少关节: "
            + ", ".join(
                missing_joint_names
            )
        )

    controlled_joint_indices = [
        joint_map[
            joint_name
        ]
        for joint_name in UR5_JOINT_NAMES
    ]

    print(
        "Robot ID:",
        robot_id
    )

    print(
        "Controlled joint indices:",
        controlled_joint_indices
    )

    return (
        robot_id,
        controlled_joint_indices,
    )


# ==========================================================
# UR5 Joint IO
# ==========================================================

def set_joint_targets(
    robot_id: int,
    controlled_joint_indices: list[int],
    target_positions_deg: list[float],
) -> None:
    """
    将六轴连续目标角发送给 PyBullet Position Controller。

    Args:
        robot_id:
            PyBullet UR5 Body ID。

        controlled_joint_indices:
            六个主动关节的 PyBullet Index。

        target_positions_deg:
            六轴连续目标角，单位 degree。

    Note:
        PyBullet Position Control 使用 radian，
        因此发送前统一进行 degree -> radian 转换。
    """
    target_positions_rad = [
        math.radians(
            angle_deg
        )
        for angle_deg in target_positions_deg
    ]

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=controlled_joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=target_positions_rad,
        forces=UR5_JOINT_FORCES,
    )


def get_joint_positions_deg(
    robot_id: int,
    controlled_joint_indices: list[int],
) -> list[float]:
    """
    读取 PyBullet 六轴实际关节角。

    Args:
        robot_id:
            PyBullet UR5 Body ID。

        controlled_joint_indices:
            六个主动关节 Index。

    Returns:
        六轴实际关节角，单位 degree。
    """
    positions_deg = []

    for joint_index in controlled_joint_indices:
        joint_state = p.getJointState(
            robot_id,
            joint_index
        )

        # getJointState()[0]
        # 为当前 Joint Position，单位 radian。
        positions_deg.append(
            math.degrees(
                joint_state[0]
            )
        )

    return positions_deg


# ==========================================================
# Protocol Application Handling
# ==========================================================

def handle_received_frame(
    frame: bytes,
    target_positions_deg: list[float],
) -> list[float]:
    """
    处理 MCU -> Python 的 Joint Protocol Frame。

    Args:
        frame:
            完整协议帧。

        target_positions_deg:
            当前 PyBullet 连续目标角，
            单位 degree。

    Returns:
        更新后的连续目标角。

    Note:
        CMD_SET_JOINT_TARGETS 中携带的是
        Canonical Angle。

        为避免跨越 ±180° 时产生
        约 360° 的错误跳变，
        这里使用当前 Continuous Target
        作为 reference 进行 unwrap。
    """
    command, angles_deg = (
        parse_joint_frame(
            frame
        )
    )

    if (
        command == CMD_SET_JOINT_TARGETS
        and angles_deg is not None
    ):
        print(
            "RX 0x01 Canonical 目标角:",
            angles_deg
        )

        new_target_positions_deg = [
            unwrap_angle_deg(
                angles_deg[index],
                target_positions_deg[index],
            )
            for index in range(
                JOINT_COUNT
            )
        ]

        print(
            "PyBullet Continuous 目标角:",
            [
                round(
                    value,
                    2
                )
                for value
                in new_target_positions_deg
            ]
        )

        return new_target_positions_deg

    if (
        command == CMD_JOINT_STATE_ACK
        and angles_deg is not None
    ):
        print(
            "RX 0x82 MCU 已确认状态:",
            angles_deg
        )

    return target_positions_deg


# ==========================================================
# Main Bridge Loop
# ==========================================================

def run_bridge(
    sock: socket.socket,
    robot_id: int,
    controlled_joint_indices: list[int],
) -> None:
    """
    运行 QEMU UART <-> PyBullet 双向控制循环。

    Args:
        sock:
            已连接 QEMU UART TCP Socket。

        robot_id:
            PyBullet UR5 Body ID。

        controlled_joint_indices:
            六轴主动关节 Index。

    Note:
        主循环依次执行：

        A. 非阻塞检查 QEMU RX；
        B. 提取完整 Protocol Frame；
        C. 更新 UR5 Position Target；
        D. 推进 PyBullet；
        E. 周期发送 Joint State；
        F. 控制仿真循环频率。
    """
    target_positions_deg = [
        0.0
    ] * JOINT_COUNT

    rx_buffer = bytearray()

    last_state_tx_time = (
        time.monotonic()
    )

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

    while True:
        # --------------------------------------------------
        # A. QEMU UART RX
        # --------------------------------------------------

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            0,
        )

        if readable:
            # 单次读取最多 1024 Byte。
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
        # B. Protocol Frame Extraction
        # --------------------------------------------------

        frames = extract_frames(
            rx_buffer
        )

        for frame in frames:
            target_positions_deg = (
                handle_received_frame(
                    frame,
                    target_positions_deg,
                )
            )

        # --------------------------------------------------
        # C. UR5 Position Control
        # --------------------------------------------------

        set_joint_targets(
            robot_id,
            controlled_joint_indices,
            target_positions_deg,
        )

        # --------------------------------------------------
        # D. Physics Step
        # --------------------------------------------------

        p.stepSimulation()

        # --------------------------------------------------
        # E. Joint State TX
        # --------------------------------------------------

        now = time.monotonic()

        if (
            now - last_state_tx_time
            >= STATE_TX_INTERVAL_S
        ):
            actual_angles_deg = (
                get_joint_positions_deg(
                    robot_id,
                    controlled_joint_indices,
                )
            )

            # UART Wire Format 只发送 Canonical Angle，
            # 连续多圈状态在发送前重新映射到：
            #
            # [-180°, 180°)。
            canonical_state_angles_deg = [
                normalize_angle_deg(
                    angle_deg
                )
                for angle_deg in actual_angles_deg
            ]

            state_frame = build_joint_frame(
                CMD_JOINT_STATE,
                canonical_state_angles_deg,
            )

            sock.sendall(
                state_frame
            )

            last_state_tx_time = now

            if (
                now - last_state_print_time
                >= STATE_PRINT_INTERVAL_S
            ):
                rounded_angles = [
                    round(
                        angle_deg,
                        2
                    )
                    for angle_deg
                    in canonical_state_angles_deg
                ]

                print(
                    "TX 0x81 Canonical 实际角度:",
                    rounded_angles
                )

                last_state_print_time = now

        # --------------------------------------------------
        # F. Simulation Rate
        # --------------------------------------------------

        time.sleep(
            1.0 / SIMULATION_STEP_HZ
        )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
    """
    启动完整 QEMU UART <-> PyBullet Bridge。
    """
    ur5_path = get_ur5_path()

    print(
        "UR5 path:",
        ur5_path
    )

    print(
        "UR5 exists:",
        ur5_path.exists()
    )

    if not ur5_path.exists():
        raise FileNotFoundError(
            f"找不到 UR5 模型: {ur5_path}"
        )

    sock = connect_qemu(
        HOST,
        PORT
    )

    try:
        robot_id, controlled_joint_indices = (
            start_pybullet(
                ur5_path
            )
        )

        try:
            run_bridge(
                sock,
                robot_id,
                controlled_joint_indices,
            )

        except KeyboardInterrupt:
            print()
            print(
                "控制桥停止。"
            )

        finally:
            p.disconnect()

    finally:
        sock.close()


if __name__ == "__main__":
    main()