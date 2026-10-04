"""
文件：angle_utils.py

用途：
统一 Python 仿真与测试侧的关节角处理规则。

与固件保持一致：

1. Canonical Angle：
   [-180°, 180°)

2. UART Joint Angle：
   int16
   1 unit = 0.01 degree

3. Continuous Angle：
   根据上一连续角度恢复距离最近的等价角。
"""

ANGLE_FULL_TURN_DEG = 360.0
ANGLE_HALF_TURN_DEG = 180.0

ANGLE_FULL_TURN_RAW = 36000
ANGLE_HALF_TURN_RAW = 18000

JOINT_ANGLE_UNIT_DEG = 0.01


def normalize_angle_deg(angle_deg: float) -> float:
    """
    将任意 degree 角度规范化到 [-180°, 180°)。
    """
    return (
        (angle_deg + ANGLE_HALF_TURN_DEG)
        % ANGLE_FULL_TURN_DEG
    ) - ANGLE_HALF_TURN_DEG


def normalize_angle_raw(raw_angle: int) -> int:
    """
    将任意 0.01° 整数角规范化到 [-18000, 18000)。
    """
    return (
        (raw_angle + ANGLE_HALF_TURN_RAW)
        % ANGLE_FULL_TURN_RAW
    ) - ANGLE_HALF_TURN_RAW


def degree_to_canonical_raw(angle_deg: float) -> int:
    """
    degree -> Canonical int16 raw。

    仍沿用当前 Python 侧 round() 量化规则，
    避免本轮结构重构改变既有行为。
    """
    raw_angle = int(
        round(
            angle_deg / JOINT_ANGLE_UNIT_DEG
        )
    )

    return normalize_angle_raw(
        raw_angle
    )


def canonical_raw_to_degree(raw_angle: int) -> float:
    """
    Canonical raw -> degree。
    """
    return (
        normalize_angle_raw(raw_angle)
        * JOINT_ANGLE_UNIT_DEG
    )


def unwrap_angle_deg(
    canonical_angle_deg: float,
    reference_angle_deg: float,
) -> float:
    """
    根据上一连续角 reference，
    将 Canonical Angle 恢复为距离 reference
    最近的等价连续角。

    例如：

        reference = 179°
        canonical = -179°

    返回：

        181°
    """
    delta = normalize_angle_deg(
        canonical_angle_deg
        - reference_angle_deg
    )

    return (
        reference_angle_deg
        + delta
    )