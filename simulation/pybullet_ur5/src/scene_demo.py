"""
文件：scene_demo.py

用途：
创建基础 PyBullet UR5 仿真场景。

当前场景包括：

1. 固定底座 UR5；
2. Ground Plane；
3. 静态 Box 障碍物；
4. 三种 Debug Camera View；
5. 240 Hz 基础物理仿真循环。

本脚本主要用于验证 UR5 模型、场景对象
和 PyBullet GUI 环境。
"""

import time
from pathlib import Path

import pybullet as p
import pybullet_data


# ==========================================================
# UR5 Model Path
# ==========================================================

current_file = Path(__file__).resolve()

# 当前目录结构：
#
# simulation/
# └── pybullet_ur5/
#     ├── models/
#     └── src/
#
# 因此 src 的父目录即 pybullet_ur5。
project_dir = current_file.parent.parent

ur5_path = (
    project_dir
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)


# ==========================================================
# PyBullet Setup
# ==========================================================

p.connect(
    p.GUI
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


# ==========================================================
# Ground Plane
# ==========================================================

p.loadURDF(
    "plane.urdf"
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

    # 工业机械臂底座固定。
    useFixedBase=True,
)


# ==========================================================
# Environment Object
# ==========================================================

# Box Half Extents，单位 meter。
#
# 实际 Box Size：
#
# 0.20 m × 0.20 m × 0.20 m。
box_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        0.10,
        0.10,
        0.10,
    ],
)

box_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[
        0.10,
        0.10,
        0.10,
    ],
)

# 静态环境物体：
#
# baseMass = 0
# 表示该物体不参与动态运动。
box_id = p.createMultiBody(
    baseMass=0.0,
    baseCollisionShapeIndex=box_collision,
    baseVisualShapeIndex=box_visual,
    basePosition=[
        0.55,
        0.20,
        0.10,
    ],
)

print(
    "Robot ID:",
    robot_id
)

print(
    "Environment box ID:",
    box_id
)


# ==========================================================
# Camera Control
# ==========================================================

# Camera View：
#
# 0 = Isometric
# 1 = Front
# 2 = Top
view_selector = p.addUserDebugParameter(
    "Camera View",
    0,
    2,
    0,
)


def set_camera_view(
    view_id: int,
) -> None:
    """
    根据 View ID 切换 PyBullet Debug Camera。

    Args:
        view_id:
            Camera View ID。

            0：
            等轴视角。

            1：
            正视图。

            2：
            俯视图。
    """
    if view_id == 0:
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

    elif view_id == 1:
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[
                0,
                0,
                0.4,
            ],
        )

    elif view_id == 2:
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=0,
            cameraPitch=-89,
            cameraTargetPosition=[
                0,
                0,
                0,
            ],
        )


# 使用 -1 确保第一次循环一定设置默认 Camera。
current_view = -1


# ==========================================================
# Simulation Loop
# ==========================================================

try:
    while True:
        selected_view = int(
            round(
                p.readUserDebugParameter(
                    view_selector
                )
            )
        )

        # Camera 没有变化时不重复更新 GUI View。
        if selected_view != current_view:
            current_view = selected_view

            set_camera_view(
                current_view
            )

            print(
                "Camera view:",
                current_view
            )

        p.stepSimulation()

        # PyBullet 常用基础仿真频率：
        #
        # 240 Hz
        # =
        # 1 / 240 second per step。
        time.sleep(
            1.0 / 240.0
        )

except KeyboardInterrupt:
    pass

finally:
    p.disconnect()