"""
文件：hello_pybullet.py

用途：
验证本地 PyBullet GUI 环境可以正常启动，
并完成最基本的 Physics Simulation。

当前验证内容：

1. PyBullet GUI Connection；
2. pybullet_data Model Path；
3. Gravity Configuration；
4. Plane URDF Loading；
5. Fixed-Step Simulation Loop。
"""

import os
import time

import pybullet as p
import pybullet_data


# ==========================================================
# Simulation Configuration
# ==========================================================

# 地球标准重力加速度，单位 m/s^2。
GRAVITY_Z_MPS2 = -9.81

# PyBullet 常用基础仿真频率，单位 Hz。
SIMULATION_FREQUENCY_HZ = 240.0

# 每步循环等待时间，单位 second。
SIMULATION_STEP_PERIOD_S = (
    1.0 / SIMULATION_FREQUENCY_HZ
)


# ==========================================================
# PyBullet Initialization
# ==========================================================

# 启动带可视化窗口的 PyBullet Client。
physics_client = p.connect(
    p.GUI
)

# 获取 PyBullet 自带 URDF / Asset 目录。
data_path = pybullet_data.getDataPath()

print(
    "PyBullet data path:",
    data_path
)

p.setGravity(
    0,
    0,
    GRAVITY_Z_MPS2
)


# ==========================================================
# Scene
# ==========================================================

plane_path = os.path.join(
    data_path,
    "plane.urdf"
)

print(
    "Plane path:",
    plane_path
)

plane_id = p.loadURDF(
    plane_path
)

print("PyBullet started.")
print(
    "Plane ID:",
    plane_id
)


# ==========================================================
# Simulation Loop
# ==========================================================

try:
    while True:
        p.stepSimulation()

        time.sleep(
            SIMULATION_STEP_PERIOD_S
        )

except KeyboardInterrupt:
    pass

finally:
    p.disconnect(
        physics_client
    )