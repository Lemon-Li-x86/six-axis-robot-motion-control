import math
from pathlib import Path

import pybullet as p


# ==========================================================
# 1. UR5 Model Path
# ==========================================================

current_file = Path(__file__).resolve()

pybullet_ur5_dir = current_file.parent.parent

ur5_path = (
    pybullet_ur5_dir
    / "models"
    / "ur5"
    / "urdf"
    / "ur5.urdf"
)


# ==========================================================
# 2. Test Configuration
# ==========================================================

UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


ROTATION_TOLERANCE = 0.001

TRANSLATION_TOLERANCE_MM = 0.05


# ==========================================================
# 3. Expected Results
#
# 与 Cortex-M4 C 端 FK Self Test 使用相同测试姿态。
#
# Matrix:
#
# [ R00 R01 R02 Tx ]
# [ R10 R11 R12 Ty ]
# [ R20 R21 R22 Tz ]
# [  0   0   0   1 ]
#
# Translation unit: mm
# ==========================================================

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
# 4. Load PyBullet
# ==========================================================

physics_client = p.connect(
    p.DIRECT
)


robot_id = p.loadURDF(
    str(ur5_path),

    basePosition=[
        0.0,
        0.0,
        0.0,
    ],

    baseOrientation=[
        0.0,
        0.0,
        0.0,
        1.0,
    ],

    useFixedBase=True
)


# ==========================================================
# 5. Joint Map
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
    for name
    in UR5_JOINT_NAMES
]


ee_link_index = joint_map[
    "ee_fixed_joint"
]


# ==========================================================
# 6. Set Exact Joint Configuration
# ==========================================================

def set_joint_configuration(
    joints_deg
):
    """
    直接设置关节状态。

    不使用：
    POSITION_CONTROL

    不执行：
    stepSimulation()

    因此这里验证的是纯几何 FK，
    与重力、电机控制、PID 无关。
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
            )
        )


# ==========================================================
# 7. Read PyBullet FK
# ==========================================================

def get_ee_transform():
    """
    获取 ee_link 相对于 World Frame 的位姿。

    当前：
    basePosition = [0, 0, 0]
    baseOrientation = Identity
    useFixedBase = True

    因此当前测试中：

    World Frame
    =
    base_link Frame

    Translation 返回单位从 m 转为 mm。
    """

    link_state = p.getLinkState(
        robot_id,
        ee_link_index,
        computeForwardKinematics=True
    )


    position_m = link_state[4]

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
# 8. Matrix Compare
# ==========================================================

def compare_transforms(
    actual,
    expected
):
    max_rotation_error = 0.0

    max_translation_error_mm = 0.0


    passed = True


    for row in range(4):

        for column in range(4):

            error = abs(
                actual[row][column]
                -
                expected[row][column]
            )


            if (
                column == 3
                and
                row < 3
            ):

                max_translation_error_mm = max(
                    max_translation_error_mm,
                    error
                )


                if (
                    error
                    >
                    TRANSLATION_TOLERANCE_MM
                ):
                    passed = False

            else:

                max_rotation_error = max(
                    max_rotation_error,
                    error
                )


                if (
                    error
                    >
                    ROTATION_TOLERANCE
                ):
                    passed = False


    return (
        passed,
        max_rotation_error,
        max_translation_error_mm
    )


# ==========================================================
# 9. Matrix Printer
# ==========================================================

def print_matrix(
    matrix
):

    for row in matrix:

        print(
            "  ["
            +
            ", ".join(
                f"{value:11.6f}"
                for value
                in row
            )
            +
            "]"
        )


# ==========================================================
# 10. Run Tests
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
        max_translation_error_mm
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
# 11. Final Result
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