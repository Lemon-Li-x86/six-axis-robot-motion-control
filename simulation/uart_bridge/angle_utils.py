"""
文件：angle_utils.py

用途：
统一 Python 仿真与测试侧的关节角处理规则。

与 Cortex-M4 固件保持一致：

1. Canonical Angle 范围：
   [-180°, 180°)

2. UART Joint Angle：
   signed int16
   1 unit = 0.01 degree

3. Continuous Angle：
   根据上一连续角恢复距离 reference 最近的等价角。
"""


# ==========================================================
# Angle Constants
# ==========================================================

# 一整圈角度，单位 degree。
ANGLE_FULL_TURN_DEG = 360.0

# Canonical Angle 半圈边界，单位 degree。
ANGLE_HALF_TURN_DEG = 180.0

# UART Raw Angle 使用 0.01 degree / unit。
ANGLE_FULL_TURN_RAW = 36000
ANGLE_HALF_TURN_RAW = 18000

# 固件和 Python 通信统一角度量化单位。
JOINT_ANGLE_UNIT_DEG = 0.01


# ==========================================================
# Canonical Angle
# ==========================================================

def normalize_angle_deg(angle_deg: float) -> float:
    """
    将任意 degree 角度规范化到 [-180°, 180°)。

    Args:
        angle_deg: 输入角度，单位 degree。

    Returns:
        Canonical Angle，单位 degree。
    """
    return (
        (angle_deg + ANGLE_HALF_TURN_DEG)
        % ANGLE_FULL_TURN_DEG
    ) - ANGLE_HALF_TURN_DEG


def normalize_angle_raw(raw_angle: int) -> int:
    """
    将任意整数角规范化到 [-18000, 18000)。

    Args:
        raw_angle: 输入整数角，单位 0.01 degree。

    Returns:
        Canonical Raw Angle，单位 0.01 degree。
    """
    return (
        (raw_angle + ANGLE_HALF_TURN_RAW)
        % ANGLE_FULL_TURN_RAW
    ) - ANGLE_HALF_TURN_RAW


# ==========================================================
# Degree / Raw Conversion
# ==========================================================

def degree_to_canonical_raw(angle_deg: float) -> int:
    """
    将 degree 角度转换为 Canonical UART Raw Angle。

    Args:
        angle_deg: 输入角度，单位 degree。

    Returns:
        Canonical Raw Angle，单位 0.01 degree。

    Note:
        当前继续使用 Python round() 进行量化，
        保持既有仿真和测试行为不变。
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
    将 Canonical UART Raw Angle 转换为 degree。

    Args:
        raw_angle: 输入整数角，单位 0.01 degree。

    Returns:
        Canonical Angle，单位 degree。
    """
    return (
        normalize_angle_raw(raw_angle)
        * JOINT_ANGLE_UNIT_DEG
    )


# ==========================================================
# Continuous Angle
# ==========================================================

def unwrap_angle_deg(
    canonical_angle_deg: float,
    reference_angle_deg: float,
) -> float:
    """
    将 Canonical Angle 恢复为最接近 reference 的连续角。

    Args:
        canonical_angle_deg:
            当前 Canonical Angle，单位 degree。

        reference_angle_deg:
            上一连续角或参考连续角，单位 degree。

    Returns:
        距离 reference 最近的等价连续角，单位 degree。

    Example:
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