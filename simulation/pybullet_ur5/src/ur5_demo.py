"""
文件：ur5_demo.py

用途：
验证 UR5 模型在 PyBullet 中的基本关节控制。

当前功能：

1. 加载 UR5 URDF；
2. 构造 Joint Name -> Joint Index 映射；
3. 使用 GUI Slider 控制 shoulder_lift_joint；
4. 使用 Position Control 驱动关节；
5. 周期打印 Target / Actual Angle。

本脚本主要用于人工验证单关节运动和
PyBullet Joint Control API。
"""

import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data


# ==========================================================
# UR5 Model Path
# ==========================================================

# 当前文件：
#
# simulation/pybullet_ur5/src/ur5_demo.py
current_file = Path(__file__).resolve()

# pybullet_ur5 目录。
project_dir = current_file.parent.parent

ur5_path = (
    project_dir
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)

print(
    "UR5 path:",
    ur5_path
)

print(
    "UR5 exists:",
    ur5_path.exists()
)


# ==========================================================
# PyBullet Setup
# ==========================================================

physics_client = p.connect(
    p.GUI
)

# PyBullet 自带资源目录，
# 用于加载 plane.urdf。
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

plane_id = p.loadURDF(
    "plane.urdf"
)

print(
    "Plane ID:",
    plane_id
)


# ==========================================================
# UR5
# ==========================================================

robot_id = p.loadURDF(
    str(ur5_path),

    # UR5 Base 位于 World Origin。
    basePosition=[
        0,
        0,
        0,
    ],

    # 工业机械臂固定底座。
    useFixedBase=True,
)

print(
    "Robot ID:",
    robot_id
)


# ==========================================================
# Joint Map
# ==========================================================

joint_count = p.getNumJoints(
    robot_id
)

print(
    "Joint count:",
    joint_count
)

print()


# Joint Name -> PyBullet Joint Index。
joint_map = {}

for joint_index in range(
    joint_count
):
    joint_info = p.getJointInfo(
        robot_id,
        joint_index
    )

    # getJointInfo()[1]：
    # Joint Name，bytes。
    joint_name = (
        joint_info[1]
        .decode("utf-8")
    )

    # getJointInfo()[2]：
    # PyBullet Joint Type。
    joint_type = joint_info[2]

    joint_map[
        joint_name
    ] = joint_index

    print(
        "index:",
        joint_index,
        "| name:",
        joint_name,
        "| type:",
        joint_type
    )


# ==========================================================
# Controlled Joint
# ==========================================================

# 当前 Demo 控制 UR5 第二轴：
#
# Joint 1:
# shoulder_pan_joint
#
# Joint 2:
# shoulder_lift_joint
controlled_joint_name = (
    "shoulder_lift_joint"
)

controlled_joint = joint_map[
    controlled_joint_name
]

print()

print(
    "Controlled joint:",
    controlled_joint_name,
    "| index:",
    controlled_joint
)


# ==========================================================
# GUI Control
# ==========================================================

# GUI Slider 输入单位：
# degree。
#
# 当前人工测试范围：
# -90° ~ +90°。
slider_id = p.addUserDebugParameter(
    "Shoulder Lift (deg)",
    -90,
    90,
    0,
)

print()
print(
    "Move the slider in the PyBullet GUI."
)
print(
    "Range: -90 deg to +90 deg"
)
print()


# ==========================================================
# Simulation Loop
# ==========================================================

try:
    while True:
        # GUI Slider 返回 degree。
        target_deg = (
            p.readUserDebugParameter(
                slider_id
            )
        )

        # PyBullet Joint Position 使用 radian。
        target_rad = math.radians(
            target_deg
        )

        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=controlled_joint,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_rad,

            # 当前 Shoulder Lift
            # 最大控制力参数。
            force=150,
        )

        p.stepSimulation()

        joint_state = p.getJointState(
            robot_id,
            controlled_joint
        )

        # getJointState()[0]：
        # 当前关节位置，单位 radian。
        actual_rad = joint_state[0]

        actual_deg = math.degrees(
            actual_rad
        )

        # 大约每 0.5 second 打印一次状态，
        # 避免 240 Hz 仿真循环刷满终端。
        if (
            int(time.time() * 2)
            !=
            int(
                (time.time() - 1 / 240)
                * 2
            )
        ):
            print(
                "Target:",
                round(
                    target_deg,
                    1
                ),
                "deg",
                "| Actual:",
                round(
                    actual_deg,
                    1
                ),
                "deg"
            )

        # PyBullet 常用基础仿真频率：
        # 240 Hz。
        time.sleep(
            1.0 / 240.0
        )

except KeyboardInterrupt:
    print()
    print(
        "Simulation stopped."
    )

finally:
    p.disconnect()