"""
文件：fk_cross_validation.py

用途：
使用 PyBullet UR5 URDF
交叉验证 Cortex-M4 C 端 Forward Kinematics。
"""

import math
from pathlib import Path

import pybullet as p


# ==========================================================
# UR5 Model Path
# ==========================================================

current_file = Path(__file__).resolve()

pybullet_ur5_dir = (
    current_file.parent.parent
)

ur5_path = (
    pybullet_ur5_dir
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)


# ==========================================================
# Test Configuration
# ==========================================================

# UR5 六个主动 Revolute Joint，
# 顺序必须与固件 robot_joint_angles_t 一致。
UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Rotation Matrix Element 最大允许绝对误差。
ROTATION_TOLERANCE = 0.001

# Translation 最大允许绝对误差，单位 mm。
TRANSLATION_TOLERANCE_MM = 0.05


# ==========================================================
# Expected Results
# ==========================================================

# 与 Cortex-M4 C 端 FK Self Test
# 使用相同测试姿态和 Expected Result。
#
# Transform：
#
# [ R00 R01 R02 Tx ]
# [ R10 R11 R12 Ty ]
# [ R20 R21 R22 Tz ]
# [  0   0   0   1 ]
#
# Rotation：
# dimensionless。
#
# Translation：
# mm。
TEST_CASES = [
    {
        "name": "Zero Configuration",

        "joints_deg": [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        "expected": [
            [0.0, 1.0, 0.0, 817.25],
            [1.0, 0.0, 0.0, 191.45],
            [0.0, 0.0, -1.0, -5.491],
            [0.0, 0.0, 0.0, 1.0],
        ],
    },

    {
        "name": "Joint 1 = +60 deg",

        "joints_deg": [
            60.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        "expected": [
            [-0.866025, 0.5, 0.0, 242.824436],
            [0.5, 0.866025, 0.0, 803.484261],
            [0.0, 0.0, -1.0, -5.491],
            [0.0, 0.0, 0.0, 1.0],
        ],
    },

    {
        "name": "Joint 2 = -90 deg",

        "joints_deg": [
            0.0,
            -90.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        "expected": [
            [0.0, 0.0, 1.0, 94.65],
            [1.0, 0.0, 0.0, 191.45],
            [0.0, 1.0, 0.0, 906.409],
            [0.0, 0.0, 0.0, 1.0],
        ],
    },
]


# ==========================================================
# PyBullet Setup
# ==========================================================

# DIRECT Mode：
#
# 不创建 GUI，
# 适合自动化几何验证。
physics_client = p.connect(
    p.DIRECT
)

robot_id = p.loadURDF(
    str(ur5_path),

    # World Frame 与 UR5 Base Origin 重合。
    basePosition=[
        0.0,
        0.0,
        0.0,
    ],

    # Identity Quaternion。
    baseOrientation=[
        0.0,
        0.0,
        0.0,
        1.0,
    ],

    useFixedBase=True,
)


# ==========================================================
# Joint Map
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
        .decode("utf-8")
    )

    joint_map[
        joint_name
    ] = joint_index


controlled_joint_indices = [
    joint_map[name]
    for name in UR5_JOINT_NAMES
]

# ee_fixed_joint 对应当前公共 ee_link。
ee_link_index = joint_map[
    "ee_fixed_joint"
]


# ==========================================================
# Joint Configuration
# ==========================================================

def set_joint_configuration(
    joints_deg,
) -> None:
    """
    直接设置六轴 Joint State。

    Args:
        joints_deg:
            六个关节角，单位 degree。

    Note:
        本函数使用 resetJointState()，
        不使用 POSITION_CONTROL，
        也不执行 stepSimulation()。

        因此当前测试验证的是纯几何 FK，
        与重力、Motor、PID 和 Dynamics 无关。
    """
    for (
        joint_index,
        angle_deg
    ) in zip(
        controlled_joint_indices,
        joints_deg
    ):
        p.resetJointState(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            targetValue=math.radians(
                angle_deg
            ),
        )


# ==========================================================
# PyBullet Forward Kinematics
# ==========================================================

def get_ee_transform():
    """
    获取 ee_link 相对于 World Frame 的齐次变换矩阵。

    Returns:
        4 × 4 Transform。

        Rotation：
        dimensionless。

        Translation：
        mm。

    Note:
        当前配置：

        basePosition = [0, 0, 0]
        baseOrientation = Identity
        useFixedBase = True

        因此：

        World Frame
        =
        UR5 Base Frame。

        PyBullet Position 原始单位为 meter，
        返回前转换为 mm。
    """
    link_state = p.getLinkState(
        robot_id,
        ee_link_index,
        computeForwardKinematics=True,
    )

    # World Position，单位 meter。
    position_m = link_state[4]

    # World Orientation Quaternion。
    quaternion = link_state[5]

    rotation_flat = (
        p.getMatrixFromQuaternion(
            quaternion
        )
    )

    transform = [
        [
            rotation_flat[0],
            rotation_flat[1],
            rotation_flat[2],
            position_m[0] * 1000.0,
        ],

        [
            rotation_flat[3],
            rotation_flat[4],
            rotation_flat[5],
            position_m[1] * 1000.0,
        ],

        [
            rotation_flat[6],
            rotation_flat[7],
            rotation_flat[8],
            position_m[2] * 1000.0,
        ],

        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    ]

    return transform


# ==========================================================
# Transform Comparison
# ==========================================================

def compare_transforms(
    actual,
    expected,
):
    """
    比较两个 4 × 4 Transform。

    Args:
        actual:
            PyBullet FK Transform。

        expected:
            Cortex-M4 C FK Expected Transform。

    Returns:
        Tuple：

        (
            passed,
            max_rotation_error,
            max_translation_error_mm,
        )

    Note:
        Translation Column：

        row 0..2
        column 3

        使用 TRANSLATION_TOLERANCE_MM。

        其余 Matrix Element
        使用 ROTATION_TOLERANCE。
    """
    max_rotation_error = 0.0
    max_translation_error_mm = 0.0

    passed = True

    for row in range(4):
        for column in range(4):
            error = abs(
                actual[row][column]
                - expected[row][column]
            )

            if (
                column == 3
                and row < 3
            ):
                max_translation_error_mm = max(
                    max_translation_error_mm,
                    error
                )

                if error > TRANSLATION_TOLERANCE_MM:
                    passed = False

            else:
                max_rotation_error = max(
                    max_rotation_error,
                    error
                )

                if error > ROTATION_TOLERANCE:
                    passed = False

    return (
        passed,
        max_rotation_error,
        max_translation_error_mm,
    )


# ==========================================================
# Matrix Output
# ==========================================================

def print_matrix(
    matrix,
) -> None:
    """
    以固定小数格式打印 4 × 4 Matrix。

    Args:
        matrix:
            待打印二维 Matrix。
    """
    for row in matrix:
        print(
            "  ["
            + ", ".join(
                f"{value:11.6f}"
                for value in row
            )
            + "]"
        )


# ==========================================================
# Cross Validation
# ==========================================================

print()
print(
    "=============================================="
)
print(
    "UR5 Forward Kinematics Cross Validation"
)
print(
    "C FK expected result vs PyBullet URDF"
)
print(
    "=============================================="
)
print()


all_passed = True


for test_case in TEST_CASES:
    print(
        "Test:",
        test_case["name"]
    )

    print(
        "Joint angles:",
        test_case["joints_deg"]
    )

    set_joint_configuration(
        test_case["joints_deg"]
    )

    actual = get_ee_transform()

    expected = (
        test_case["expected"]
    )

    (
        passed,
        max_rotation_error,
        max_translation_error_mm,
    ) = compare_transforms(
        actual,
        expected
    )

    print()
    print(
        "PyBullet:"
    )

    print_matrix(
        actual
    )

    print()
    print(
        "C FK expected:"
    )

    print_matrix(
        expected
    )

    print()

    print(
        "Max rotation error:",
        f"{max_rotation_error:.8f}"
    )

    print(
        "Max translation error:",
        f"{max_translation_error_mm:.6f} mm"
    )

    if passed:
        print(
            "Result: PASS"
        )

    else:
        print(
            "Result: FAIL"
        )

        all_passed = False

    print()
    print(
        "----------------------------------------------"
    )
    print()


# ==========================================================
# Final Result
# ==========================================================

if all_passed:
    print(
        "FINAL RESULT: ALL FK TESTS PASSED"
    )

else:
    print(
        "FINAL RESULT: FK CROSS VALIDATION FAILED"
    )


p.disconnect()