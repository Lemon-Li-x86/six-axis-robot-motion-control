import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data


# ==========================================================
# 1. 找到 UR5 模型路径
# ==========================================================

# 当前文件：
# simulation/pybullet_ur5/src/ur5_demo.py
current_file = Path(__file__).resolve()

# pybullet_ur5 目录
project_dir = current_file.parent.parent

# UR5 的 URDF 文件
ur5_path = (
    project_dir
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)

print("UR5 path:", ur5_path)
print("UR5 exists:", ur5_path.exists())


# ==========================================================
# 2. 启动 PyBullet
# ==========================================================

physics_client = p.connect(p.GUI)

# PyBullet 自带资源目录，用于加载 plane.urdf
p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

# 设置重力
p.setGravity(
    0,
    0,
    -9.81
)

# 调整摄像机视角，方便观察机械臂
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.4]
)

# 加载地面
plane_id = p.loadURDF(
    "plane.urdf"
)

print("Plane ID:", plane_id)


# ==========================================================
# 3. 加载 UR5
# ==========================================================

robot_id = p.loadURDF(
    str(ur5_path),

    # 机械臂底座位置
    basePosition=[0, 0, 0],

    # 工业机械臂底座固定
    useFixedBase=True
)

print("Robot ID:", robot_id)


# ==========================================================
# 4. 读取所有关节信息
# ==========================================================

joint_count = p.getNumJoints(
    robot_id
)

print("Joint count:", joint_count)
print()


# 保存：
# 关节名字 -> 关节 index
joint_map = {}


for joint_index in range(joint_count):

    joint_info = p.getJointInfo(
        robot_id,
        joint_index
    )

    # joint_info[1] 是关节名称，类型是 bytes
    joint_name = joint_info[1].decode("utf-8")

    # joint_info[2] 是关节类型
    joint_type = joint_info[2]

    joint_map[joint_name] = joint_index

    print(
        "index:",
        joint_index,
        "| name:",
        joint_name,
        "| type:",
        joint_type
    )


# ==========================================================
# 5. 选择要控制的关节
# ==========================================================

# 先控制 UR5 第二轴
# 第一轴是 shoulder_pan_joint
# 第二轴是 shoulder_lift_joint
controlled_joint_name = "shoulder_lift_joint"

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
# 6. 创建 GUI 滑块
# ==========================================================

# 滑块显示单位：degree
# 控制范围：-90° 到 +90°
slider_id = p.addUserDebugParameter(
    "Shoulder Lift (deg)",
    -90,
    90,
    0
)

print()
print("Move the slider in the PyBullet GUI.")
print("Range: -90 deg to +90 deg")
print()


# ==========================================================
# 7. 仿真主循环
# ==========================================================

try:

    while True:

        # --------------------------------------------------
        # 读取 GUI 滑块
        # --------------------------------------------------

        # 读取到的单位是 degree
        target_deg = p.readUserDebugParameter(
            slider_id
        )


        # --------------------------------------------------
        # degree -> rad
        # --------------------------------------------------

        target_rad = math.radians(
            target_deg
        )


        # --------------------------------------------------
        # 发送关节位置控制命令
        # --------------------------------------------------

        p.setJointMotorControl2(
            bodyUniqueId=robot_id,

            jointIndex=controlled_joint,

            # 位置控制模式
            controlMode=p.POSITION_CONTROL,

            # 目标角度，单位 rad
            targetPosition=target_rad,

            # 最大电机输出力矩
            force=150
        )


        # --------------------------------------------------
        # 推进一步物理仿真
        # --------------------------------------------------

        p.stepSimulation()


        # --------------------------------------------------
        # 读取当前实际关节角
        # --------------------------------------------------

        joint_state = p.getJointState(
            robot_id,
            controlled_joint
        )

        actual_rad = joint_state[0]

        actual_deg = math.degrees(
            actual_rad
        )


        # --------------------------------------------------
        # 每隔一段时间打印状态
        # 避免终端疯狂刷屏
        # --------------------------------------------------

        # 当前时间乘 2 后取整变化时，大约每 0.5 秒打印一次
        if int(time.time() * 2) != int((time.time() - 1 / 240) * 2):

            print(
                "Target:",
                round(target_deg, 1),
                "deg",
                "| Actual:",
                round(actual_deg, 1),
                "deg"
            )


        # PyBullet 常用仿真频率 240 Hz
        time.sleep(
            1.0 / 240.0
        )


# ==========================================================
# 8. Ctrl + C 退出
# ==========================================================

except KeyboardInterrupt:

    print()
    print("Simulation stopped.")


finally:

    p.disconnect()