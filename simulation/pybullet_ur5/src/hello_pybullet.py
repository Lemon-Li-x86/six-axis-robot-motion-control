import os
import time

import pybullet as p
import pybullet_data


# 启动 PyBullet GUI
physics_client = p.connect(p.GUI)

# PyBullet 自带模型目录
data_path = pybullet_data.getDataPath()

print("PyBullet data path:", data_path)

# 设置重力
p.setGravity(0, 0, -9.81)

# 使用完整路径加载地面 URDF
plane_path = os.path.join(data_path, "plane.urdf")

print("Plane path:", plane_path)

plane_id = p.loadURDF(plane_path)

print("PyBullet started.")
print("Plane ID:", plane_id)


try:
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

except KeyboardInterrupt:
    pass

finally:
    p.disconnect()