"""
文件：ur5_uart_bridge.py

用途：
连接：

    QEMU Cortex-M4 / FreeRTOS UART
    <->
    Python TCP Bridge
    <->
    PyBullet UR5

本文件只负责：

1. TCP 连接；
2. PyBullet UR5 生命周期；
3. Protocol Frame 的 Application 处理；
4. 目标关节角下发；
5. 实际关节状态反馈。

协议编解码统一由 protocol_codec.py 提供。

角度规范化和连续角恢复统一由 angle_utils.py 提供。
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
# TCP
# ==========================================================

HOST = "127.0.0.1"

PORT = 5555


# ==========================================================
# Simulation
# ==========================================================

STATE_TX_INTERVAL_S = 0.2

STATE_PRINT_INTERVAL_S = 1.0

SIMULATION_STEP_HZ = 240.0

SOCKET_RETRY_INTERVAL_S = 0.5


UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


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
    positions_deg = []

    for joint_index in controlled_joint_indices:
        joint_state = p.getJointState(
            robot_id,
            joint_index
        )

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
    command, angles_deg = (
        parse_joint_frame(
            frame
        )
    )

    if (
        command
        == CMD_SET_JOINT_TARGETS
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
                for value in new_target_positions_deg
            ]
        )

        return new_target_positions_deg

    if (
        command
        == CMD_JOINT_STATE_ACK
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
        # A. 接收 QEMU UART TCP 数据
        # --------------------------------------------------

        readable, _, _ = select.select(
            [sock],
            [],
            [],
            0,
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
        # B. 提取并处理完整协议帧
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
        # C. 控制 UR5
        # --------------------------------------------------

        set_joint_targets(
            robot_id,
            controlled_joint_indices,
            target_positions_deg,
        )

        # --------------------------------------------------
        # D. 推进物理仿真
        # --------------------------------------------------

        p.stepSimulation()

        # --------------------------------------------------
        # E. 周期返回 Joint State
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
        # F. 约 240 Hz 仿真
        # --------------------------------------------------

        time.sleep(
            1.0 / SIMULATION_STEP_HZ
        )


# ==========================================================
# Entry
# ==========================================================

def main() -> None:
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