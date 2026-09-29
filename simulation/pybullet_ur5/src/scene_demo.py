import time
from pathlib import Path

import pybullet as p
import pybullet_data


# ==========================================================
# 1. UR5 模型路径
# ==========================================================

current_file = Path(__file__).resolve()
project_dir = current_file.parent.parent

ur5_path = (
    project_dir
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)


# ==========================================================
# 2. 启动 PyBullet
# ==========================================================

p.connect(p.GUI)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(
    0,
    0,
    -9.81
)


# ==========================================================
# 3. 加载地面
# ==========================================================

p.loadURDF(
    "plane.urdf"
)


# ==========================================================
# 4. 加载 UR5
# ==========================================================

robot_id = p.loadURDF(
    str(ur5_path),
    basePosition=[0, 0, 0],
    useFixedBase=True
)


# ==========================================================
# 5. 添加基础环境物体
# ==========================================================

# 创建一个箱子的碰撞模型
box_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        0.10,
        0.10,
        0.10
    ]
)

# 创建一个箱子的可视模型
box_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[
        0.10,
        0.10,
        0.10
    ]
)

# 创建实际物体
box_id = p.createMultiBody(
    baseMass=0.0,

    baseCollisionShapeIndex=box_collision,

    baseVisualShapeIndex=box_visual,

    basePosition=[
        0.55,
        0.20,
        0.10
    ]
)

print("Robot ID:", robot_id)
print("Environment box ID:", box_id)


# ==========================================================
# 6. 创建视角选择滑块
# ==========================================================

view_selector = p.addUserDebugParameter(
    "Camera View",
    0,
    2,
    0
)


# ==========================================================
# 7. 三种视角
# ==========================================================

def set_camera_view(view_id):

    # ------------------------------------------------------
    # 0：等轴视角
    # ------------------------------------------------------

    if view_id == 0:

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


    # ------------------------------------------------------
    # 1：正视图
    # ------------------------------------------------------

    elif view_id == 1:

        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=90,
            cameraPitch=0,
            cameraTargetPosition=[
                0,
                0,
                0.4
            ]
        )


    # ------------------------------------------------------
    # 2：俯视图
    # ------------------------------------------------------

    elif view_id == 2:

        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=0,
            cameraPitch=-89,
            cameraTargetPosition=[
                0,
                0,
                0
            ]
        )


# 默认使用等轴视角
current_view = -1


# ==========================================================
# 8. 仿真主循环
# ==========================================================

try:

    while True:

        # 读取 GUI 滑块
        selected_view = int(
            round(
                p.readUserDebugParameter(
                    view_selector
                )
            )
        )


        # 只有切换视角时才重新设置摄像机
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

        time.sleep(
            1.0 / 240.0
        )


except KeyboardInterrupt:

    pass


finally:

    p.disconnect()