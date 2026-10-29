"""Stage5 simulation-only APF -> production C IK -> whole-arm gate.

No actuation. Converts actual PyBullet static AABB(s) into the existing
production C APF, then production C UR5 IK/selector/FK and trapezoid
timings, then checks the *resulting* joint route against all robot links.

Fail closed: any failed stage raises RejectedRoute; no call to a UART or
motion command exists in this module. This is sampled joint-space checking,
not a continuous no-collision certificate and not a runtime re-planner.
"""
from __future__ import annotations

import ctypes
import math
from pathlib import Path
from typing import Sequence

from stage5_static_obstacle_probe import Point, AABB, Config, point
from stage5_collision_validation import validate_joint_path, InvalidPath

JOINT_COUNT = 6
MAX_APF_OBSTACLES = 8  # mirrored C ABI bound; runtime geometry comes from PyBullet


class RejectedRoute(RuntimeError):
    pass


class JointPositions(ctypes.Structure):
    _fields_ = [("value", ctypes.c_int32 * JOINT_COUNT)]


class JointRates(ctypes.Structure):
    _fields_ = [("value", ctypes.c_float * JOINT_COUNT)]


class JointAccelerations(ctypes.Structure):
    _fields_ = [("value", ctypes.c_float * JOINT_COUNT)]


class JointRouteSegment(ctypes.Structure):
    _fields_ = [("target", JointPositions), ("segment_duration_s", ctypes.c_double)]


def preflight_joint_avoidance(
    library: Path,
    physics,
    robot_id: int,
    obstacle_ids: Sequence[int],
    joint_indices: Sequence[int],
    end_link: int,
    goal_mm: Sequence[float],
    *,
    apf_config: Config,
    max_velocity_deg_s: Sequence[float],
    max_acceleration_deg_s2: Sequence[float],
    max_fk_position_error_mm: float,
    max_fk_rotation_element_error: float,
    waypoint_capacity: int,
    max_joint_step_rad: float,
    max_collision_samples: int,
    physics_client_id: int,
    minimum_full_link_clearance_m: float | None = None,
) -> dict:
    """Compute C path and C joint targets, and reject unsafe joint routes."""
    # Cartesian APF point clearance and physical link clearance are
    # different constraints. An EE point does not include wrist/link volume.
    # Preserve the original API default for callers that did not specify
    # a separate full-link requirement.
    if minimum_full_link_clearance_m is None:
        minimum_full_link_clearance_m = apf_config.clearance_mm / 1000.0
    if (not math.isfinite(minimum_full_link_clearance_m) or
            minimum_full_link_clearance_m < 0.0):
        raise RejectedRoute("invalid full-link collision clearance")
    if not library.is_file():
        raise FileNotFoundError(library)
    if (not isinstance(waypoint_capacity, int) or waypoint_capacity <= 0
            or not obstacle_ids or len(obstacle_ids) > MAX_APF_OBSTACLES
            or len(joint_indices) != JOINT_COUNT
            or len(goal_mm) != 3
            or len(max_velocity_deg_s) != JOINT_COUNT
            or len(max_acceleration_deg_s2) != JOINT_COUNT
            or not all(math.isfinite(float(x)) for x in goal_mm)):
        raise RejectedRoute("invalid route or robot configuration")

    c = ctypes.CDLL(str(library.resolve()))
    c.robot_apf_plan_path.restype = ctypes.c_int
    c.robot_apf_plan_path.argtypes = [
        ctypes.POINTER(Point), ctypes.POINTER(Point),
        ctypes.POINTER(AABB), ctypes.c_size_t,
        ctypes.POINTER(Config), ctypes.POINTER(Point),
        ctypes.c_size_t, ctypes.POINTER(ctypes.c_size_t),
    ]
    c.stage5_apf_points_to_joint_route.restype = ctypes.c_int
    c.stage5_apf_points_to_joint_route.argtypes = [
        ctypes.POINTER(JointPositions), ctypes.POINTER(Point),
        ctypes.c_size_t, ctypes.POINTER(JointRates),
        ctypes.POINTER(JointAccelerations),
        ctypes.c_float, ctypes.c_float,
        ctypes.POINTER(JointRouteSegment), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
    ]

    initial_rad = [
        physics.getJointState(robot_id, joint,
                              physicsClientId=physics_client_id)[0]
        for joint in joint_indices
    ]
    start_raw = JointPositions(
        (ctypes.c_int32 * JOINT_COUNT)(
            *(round(math.degrees(x)*100) for x in initial_rad)))
    end_position_m = physics.getLinkState(
        robot_id, end_link, computeForwardKinematics=True,
        physicsClientId=physics_client_id)[4]
    start = point([1000*x for x in end_position_m])
    goal = point(goal_mm)

    boxes = []
    for obstacle in obstacle_ids:
        low, high = physics.getAABB(obstacle, physicsClientId=physics_client_id)
        boxes.append(AABB(point([1000*x for x in low]),
                          point([1000*x for x in high])))
    obstacle_array = (AABB * len(boxes))(*boxes)
    apf_points = (Point * waypoint_capacity)()
    apf_n = ctypes.c_size_t(0)
    apf_status = c.robot_apf_plan_path(
        ctypes.byref(start), ctypes.byref(goal),
        obstacle_array, len(boxes), ctypes.byref(apf_config),
        apf_points, waypoint_capacity, ctypes.byref(apf_n))
    if apf_status:
        raise RejectedRoute(f"production C APF route rejected: status={apf_status}")
    if not apf_n.value:
        raise RejectedRoute("goal already reached; refusing empty motion request")

    segments = (JointRouteSegment * waypoint_capacity)()
    joint_n = ctypes.c_size_t(0)
    velocity = JointRates((ctypes.c_float * JOINT_COUNT)(*max_velocity_deg_s))
    acceleration = JointAccelerations(
        (ctypes.c_float * JOINT_COUNT)(*max_acceleration_deg_s2))
    ik_status = c.stage5_apf_points_to_joint_route(
        ctypes.byref(start_raw), apf_points, apf_n.value,
        ctypes.byref(velocity), ctypes.byref(acceleration),
        max_fk_position_error_mm, max_fk_rotation_element_error,
        segments, waypoint_capacity, ctypes.byref(joint_n))
    if ik_status or joint_n.value != apf_n.value:
        raise RejectedRoute(
            f"production C IK/FK/rate route rejected: status={ik_status}, "
            f"converted={joint_n.value}/{apf_n.value}")

    path = [tuple(initial_rad)]
    durations = []
    for segment in segments[:joint_n.value]:
        path.append(tuple(math.radians(raw/100)
                          for raw in segment.target.value))
        durations.append(segment.segment_duration_s)
    try:
        gate = validate_joint_path(
            physics, robot_id, obstacle_ids, joint_indices, path,
            maximum_joint_step_rad=max_joint_step_rad,
            clearance_m=minimum_full_link_clearance_m,
            maximum_samples=max_collision_samples,
            physics_client_id=physics_client_id,
        )
    except InvalidPath as error:
        raise RejectedRoute(f"full-arm geometry rejected: {error}") from error
    if not gate["passed"]:
        raise RejectedRoute(
            "full-arm collision gate rejected: " + str(gate["collision"]))

    return {
        "passed": True,
        "scope": "simulation_preflight_only_no_uart_no_actuation",
        "cartesian_waypoints": apf_n.value,
        "joint_waypoints_rad": path,
        "segment_duration_s": durations,
        "sampled_full_arm_states": gate["sampled_states"],
        "total_minimum_rest_to_rest_duration_s": sum(durations),
        "static_collision_clearance_m": minimum_full_link_clearance_m,
        "cartesian_point_clearance_m": apf_config.clearance_mm/1000,
        "not_certified": [
            "continuous swept collision", "self collision",
            "dynamic obstacles", "FreeRTOS command/feedback execution",
        ],
    }
