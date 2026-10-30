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
    max_fk_position_error_mm: float | None,
    max_fk_rotation_element_error: float | None,
    waypoint_capacity: int,
    max_joint_step_rad: float,
    max_collision_samples: int,
    physics_client_id: int,
    minimum_full_link_clearance_m: float | None = None,
    via_points_mm: Sequence[Sequence[float]] = (),
    shortcut_cartesian: bool = False,
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
    # There is exactly one authority for IK FK residual ceilings:
    # kinematics.h in the production C build. Scene JSON never redefines it.
    c.stage5_apf_joint_pose_limits.restype = ctypes.c_int
    c.stage5_apf_joint_pose_limits.argtypes = (
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float))
    c_position, c_rotation = ctypes.c_float(), ctypes.c_float()
    status = c.stage5_apf_joint_pose_limits(
        ctypes.byref(c_position), ctypes.byref(c_rotation))
    if status:
        raise RejectedRoute(f"C IK acceptance limits unavailable: {status}")
    if max_fk_position_error_mm is None:
        max_fk_position_error_mm = c_position.value
    if max_fk_rotation_element_error is None:
        max_fk_rotation_element_error = c_rotation.value
    if (not math.isfinite(max_fk_position_error_mm) or
            not math.isfinite(max_fk_rotation_element_error) or
            not (0 < max_fk_position_error_mm <= c_position.value) or
            not (0 < max_fk_rotation_element_error <= c_rotation.value)):
        raise RejectedRoute("IK residual gates cannot exceed production C limits")
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
    # Stage6-compatible multi-leg Cartesian route: the caller supplies
    # intermediate *task* waypoints. Every leg is still planned solely by
    # production C APF. This Python application bridge never implements an
    # independent attraction/repulsion planner or blind point interpolation.
    intermediate = []
    for index, waypoint in enumerate(via_points_mm):
        if (len(waypoint) != 3 or
                not all(math.isfinite(float(x)) for x in waypoint)):
            raise RejectedRoute(f"invalid intermediate waypoint {index}")
        intermediate.append(tuple(float(x) for x in waypoint))
    targets = intermediate + [tuple(goal.mm)]
    apf_points = (Point * waypoint_capacity)()
    total_points = 0
    leg_end_counts = []
    leg_start = start
    for leg, target_mm in enumerate(targets):
        remaining = waypoint_capacity - total_points
        if remaining <= 0:
            raise RejectedRoute("C APF multi-leg route exceeded shared waypoint capacity")
        leg_points = (Point * remaining)()
        leg_count = ctypes.c_size_t(0)
        leg_goal = point(target_mm)
        apf_status = c.robot_apf_plan_path(
            ctypes.byref(leg_start), ctypes.byref(leg_goal),
            obstacle_array, len(boxes), ctypes.byref(apf_config),
            leg_points, remaining, ctypes.byref(leg_count))
        if apf_status:
            raise RejectedRoute(
                f"production C APF route leg {leg+1}/{len(targets)} "
                f"rejected: status={apf_status}")
        if leg_count.value:
            for i in range(leg_count.value):
                apf_points[total_points+i] = leg_points[i]
            total_points += leg_count.value
            leg_end_counts.append(total_points)
            leg_start = leg_points[leg_count.value-1]
        # Do not silently make a jump between a reached-via tolerance
        # and the next segment, nor accept a partially reached task target.
        if math.dist(list(leg_start.mm), list(leg_goal.mm)) > (
                apf_config.goal_tolerance_mm + 1e-6):
            raise RejectedRoute(f"C APF leg {leg+1} did not reach goal tolerance")
    if total_points == 0:
        raise RejectedRoute("goal already reached; refusing empty motion request")
    raw_apf_waypoints = total_points
    # Clone the C APF output before evaluating simplification candidates:
    # ctypes array aliases must not make a rejected trial alter raw data.
    raw_points = tuple(point(tuple(value.mm))
                       for value in apf_points[:total_points])
    velocity = JointRates((ctypes.c_float * JOINT_COUNT)(*max_velocity_deg_s))
    acceleration = JointAccelerations(
        (ctypes.c_float * JOINT_COUNT)(*max_acceleration_deg_s2))

    def validate_candidate(candidate):
        """No candidate is executable until BOTH real safety gates pass."""
        candidate_array = (Point * len(candidate))(*candidate)
        segments = (JointRouteSegment * len(candidate))()
        joint_n = ctypes.c_size_t(0)
        ik_status = c.stage5_apf_points_to_joint_route(
            ctypes.byref(start_raw), candidate_array, len(candidate),
            ctypes.byref(velocity), ctypes.byref(acceleration),
            max_fk_position_error_mm, max_fk_rotation_element_error,
            segments, len(candidate), ctypes.byref(joint_n))
        if ik_status or joint_n.value != len(candidate):
            raise RejectedRoute(
                f"production C IK/FK/rate route rejected: status={ik_status}, "
                f"converted={joint_n.value}/{len(candidate)}")
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
                physics_client_id=physics_client_id)
        except InvalidPath as error:
            raise RejectedRoute(
                f"full-arm geometry rejected: {error}") from error
        if not gate["passed"]:
            raise RejectedRoute(
                "full-arm collision gate rejected: " + str(gate["collision"]))
        return path, durations, gate

    shortcut_attempts = []
    chosen_span = None
    if shortcut_cartesian:
        # Coarse-to-fine search: never reduce the link-clearance threshold
        # or accept a partial path. Last rung is the untouched C APF route.
        from stage5_cartesian_shortcut import (
            shortcut_c_apf_legs, InvalidShortcut,
            VERIFIED_SHORTCUT_SPANS,
        )
        for span in VERIFIED_SHORTCUT_SPANS:
            try:
                selected = shortcut_c_apf_legs(
                    c, start, raw_points, leg_end_counts,
                    boxes, apf_config.clearance_mm,
                    max_original_span=span)
                path, durations, gate = validate_candidate(selected)
            except (InvalidShortcut, RejectedRoute) as error:
                shortcut_attempts.append({
                    "max_original_span": span,
                    "passed": False,
                    "reason": str(error),
                })
                continue
            chosen_span = span
            chosen_points = selected
            shortcut_attempts.append({
                "max_original_span": span,
                "passed": True,
                "cartesian_waypoints": len(chosen_points),
            })
            break
        else:
            raise RejectedRoute(
                "all C APF shortcut spans failed full-arm safety: "
                + str(shortcut_attempts))
    else:
        chosen_points = raw_points
        path, durations, gate = validate_candidate(chosen_points)

    return {
        "passed": True,
        "scope": "simulation_preflight_only_no_uart_no_actuation",
        "cartesian_waypoints": len(chosen_points),
        "raw_apf_waypoints": raw_apf_waypoints,
        "cartesian_shortcut_used": bool(shortcut_cartesian),
        "shortcut_accepted_max_original_span": chosen_span,
        "shortcut_attempts": shortcut_attempts,
        "via_points_mm": intermediate,
        "cartesian_route_mm": [list(value.mm) for value in chosen_points],
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
