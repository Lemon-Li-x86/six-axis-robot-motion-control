
from pathlib import Path
import struct

import numpy as np
import roboticstoolbox as rtb
from scipy.spatial.transform import Rotation
from spatialmath import SE3

DATA_PATH = Path("build/fk_validation/fk_samples.bin")
COUNT = 128

# 必须与 Cortex-M4 的 fk_sample_t 内存布局一致
RECORD = struct.Struct("<6h16fi")

robot = rtb.models.DH.UR5()

# 对齐 UR5 几何参数
robot.links[0].d = 0.089159

# 对齐 base_link / ee_link
robot.base = SE3.Rz(np.pi)

robot.tool = SE3(np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1],
], dtype=float))

payload = DATA_PATH.read_bytes()

expected_bytes = COUNT * RECORD.size
if len(payload) != expected_bytes:
    raise ValueError(
        f"Binary size mismatch: "
        f"{len(payload)} != {expected_bytes}"
    )

position_errors = []
rotation_errors = []
records = []

for index in range(COUNT):
    offset = index * RECORD.size
    values = RECORD.unpack_from(payload, offset)

    joints_raw = np.array(values[:6], dtype=np.int16)
    c_matrix = np.array(
        values[6:22], dtype=np.float64
    ).reshape(4, 4)

    status = values[22]

    if status != 0:
        raise RuntimeError(
            f"C FK failed: case={index}, status={status}"
        )

    q_rad = np.deg2rad(
        joints_raw.astype(float) * 0.01
    )

    ref_matrix = robot.fkine(q_rad).A.copy()
    ref_matrix[:3, 3] *= 1000.0

    position_error = np.linalg.norm(
        c_matrix[:3, 3] - ref_matrix[:3, 3]
    )

    relative_rotation = (
        ref_matrix[:3, :3].T @ c_matrix[:3, :3]
    )

    rotation_error_deg = np.rad2deg(
        Rotation.from_matrix(
            relative_rotation
        ).magnitude()
    )

    position_errors.append(position_error)
    rotation_errors.append(rotation_error_deg)

    records.append((
        index,
        joints_raw.tolist(),
        position_error,
        rotation_error_deg
    ))

p = np.array(position_errors)
r = np.array(rotation_errors)

print("\nCortex-M4 FK vs Robotics Toolbox")
print("=" * 48)
print("Samples:", COUNT)

print("\nPosition error (mm)")
print("Mean:", f"{np.mean(p):.9f}")
print("Max: ", f"{np.max(p):.9f}")
print("RMSE:", f"{np.sqrt(np.mean(p**2)):.9f}")

print("\nRotation error (deg)")
print("Mean:", f"{np.mean(r):.9f}")
print("Max: ", f"{np.max(r):.9f}")
print("RMSE:", f"{np.sqrt(np.mean(r**2)):.9f}")

worst_p = records[int(np.argmax(p))]
worst_r = records[int(np.argmax(r))]

print("\nWorst position case:", worst_p)
print("Worst rotation case:", worst_r)

# 暂定验收阈值，可结合实际误差分布再审查
passed = (
    np.all(p <= 0.05)
    and np.all(r <= 0.05)
)

print("\nRESULT:", "PASS" if passed else "FAIL")
