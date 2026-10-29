"""Simulation-only bridge: declarative scene -> production C geometry/APF/IK -> full-link gate.

This module owns scene setup, NOT robot kinematics, joint limits, PID, or
FreeRTOS timing. Simulation fixtures are read from scenarios JSON. Stage6
mission sequences may reuse the same planner and safety gate for each leg.
There are no trial-and-error obstacle placements or safety-threshold bypasses.
"""
from __future__ import annotations

import ctypes
import json
import math
from pathlib import Path

from stage4_dynamic_closed_loop import DEFAULT_JSON
from stage5_static_obstacle_probe import Point, AABB, Config, point
from stage5_avoidance_preflight import preflight_joint_avoidance, RejectedRoute

SCENE_CATALOG = Path(__file__).resolve().parents[1] / "scenarios" / "stage5_static_obstacles.json"


def _vec3(value, name: str) -> tuple[float, float, float]:
    if (not isinstance(value, list) or len(value) != 3 or
            any(type(v) not in (int, float) or not math.isfinite(v)
                for v in value)):
        raise ValueError(f"{name}: expected three finite numbers")
    return tuple(float(v) for v in value)


def load_scene(route_case: str) -> tuple[dict, dict]:
    """Validate a versioned simulation fixture without silently substituting defaults."""
    catalog = json.loads(SCENE_CATALOG.read_text(encoding="utf-8"))
    if catalog.get("schema_version") != 1:
        raise ValueError("unsupported scene catalog schema")
    try:
        scene = catalog["scenes"][route_case]
    except KeyError as error:
        raise ValueError(f"unknown scene: {route_case}") from error
    _vec3(scene["goal_offset_mm"], "goal_offset_mm")
    vias = scene.get("via_offsets_ee_mm", [])
    if not isinstance(vias, list):
        raise ValueError("via_offsets_ee_mm must be a list")
    for index, via in enumerate(vias):
        _vec3(via, f"via_offsets_ee_mm[{index}]")
    box = scene["obstacle"]
    has_world = "world_center_m" in box
    has_offset = "center_offset_ee_mm" in box
    if has_world == has_offset:
        raise ValueError("obstacle requires exactly one world/EE-relative center")
    _vec3(box["world_center_m"] if has_world else box["center_offset_ee_mm"],
          "obstacle center")
    halves = _vec3(box["half_extents_m"], "half_extents_m")
    if any(h <= 0.0 for h in halves):
        raise ValueError("box half extents must be strictly positive")
    apf_values = tuple(scene["apf"][name] for name, _ in Config._fields_)
    if any(type(v) not in (float, int) or not math.isfinite(v)
           for v in apf_values):
        raise ValueError("nonfinite or invalid APF configuration")
    if not (apf_values[0] > 0.0 and apf_values[1] >= 0.0 and
            all(v > 0.0 for v in apf_values[2:])):
        raise ValueError("invalid APF gains/distances")
    safety = scene["minimum_full_link_clearance_mm"]
    if type(safety) not in (float, int) or not math.isfinite(safety) or safety < 0:
        raise ValueError("invalid full-link clearance")
    capacity = scene["waypoint_capacity"]
    if type(capacity) is not int or capacity <= 0:
        raise ValueError("invalid waypoint capacity")
    if type(scene["expect_direct_blocked"]) is not bool:
        raise ValueError("expect_direct_blocked must be a boolean")
    if type(scene.get("shortcut_cartesian", False)) is not bool:
        raise ValueError("shortcut_cartesian must be a boolean")
    policy = catalog["preflight"]
    if not (math.isfinite(policy["max_joint_step_rad"]) and
            policy["max_joint_step_rad"] > 0 and
            type(policy["max_collision_samples"]) is int and
            policy["max_collision_samples"] > 0):
        raise ValueError("invalid full-link validation budget")
    return scene, policy


def _direct_is_blocked(library: Path, start_mm: tuple[float, ...],
                       goal_mm: tuple[float, ...], bounds: AABB,
                       clearance_mm: float) -> bool:
    """Call the same C swept AABB geometry used by the production APF step."""
    c = ctypes.CDLL(str(library.resolve()))
    c.robot_apf_segment_blocked.restype = ctypes.c_int
    c.robot_apf_segment_blocked.argtypes = (
        ctypes.POINTER(Point), ctypes.POINTER(Point),
        ctypes.POINTER(AABB), ctypes.c_size_t,
        ctypes.c_double, ctypes.POINTER(ctypes.c_uint))
    start, goal = point(start_mm), point(goal_mm)
    hit = ctypes.c_uint(0)
    status = c.robot_apf_segment_blocked(
        ctypes.byref(start), ctypes.byref(goal),
        ctypes.byref(bounds), 1, float(clearance_mm), ctypes.byref(hit))
    if status != 0:
        raise RejectedRoute(f"C segment geometry rejected: status={status}")
    return bool(hit.value)


def make_avoidance_preflight(library: Path, route_case: str):
    scene, safety = load_scene(route_case)
    # The robot's velocity and acceleration limits remain solely in the
    # existing firmware config. Scene JSON contains no duplicate joint data.
    motion = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))["stage4_motion"]
    apf = Config(*(float(scene["apf"][name]) for name, _ in Config._fields_))

    def prepare(physics, robot, joints, client):
        if len(joints) != 6:
            raise ValueError("UR5 requires exactly six controlled joints")
        end_link = next(
            (index for index in range(physics.getNumJoints(
                robot, physicsClientId=client))
             if physics.getJointInfo(
                 robot, index, physicsClientId=client)[12] == b"ee_link"),
            None)
        if end_link is None:
            raise ValueError("UR5 ee_link not found")
        end_m = physics.getLinkState(
            robot, end_link, computeForwardKinematics=True,
            physicsClientId=client)[4]
        start_mm = tuple(1000.0 * c for c in end_m)
        goal_mm = tuple(a + b for a, b in zip(
            start_mm, _vec3(scene["goal_offset_mm"], "goal_offset_mm")))
        box = scene["obstacle"]
        if "world_center_m" in box:
            center_m = _vec3(box["world_center_m"], "world_center_m")
        else:
            offset = _vec3(box["center_offset_ee_mm"], "center_offset_ee_mm")
            center_m = tuple(v + d / 1000.0 for v, d in zip(end_m, offset))
        half_extents_m = _vec3(box["half_extents_m"], "half_extents_m")
        shape = physics.createCollisionShape(
            physics.GEOM_BOX, halfExtents=half_extents_m,
            physicsClientId=client)
        obstacle = physics.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=shape,
            basePosition=center_m, physicsClientId=client)
        # Both tests use actual PyBullet geometry rather than a duplicate
        # Python box intersection implementation.
        low, high = physics.getAABB(obstacle, physicsClientId=client)
        bounds = AABB(point([1000*x for x in low]),
                      point([1000*x for x in high]))
        direct_blocked = _direct_is_blocked(
            library, start_mm, goal_mm, bounds, apf.clearance_mm)
        if direct_blocked != scene["expect_direct_blocked"]:
            raise RejectedRoute(
                f"direct obstacle fixture invalid, blocked={direct_blocked}, "
                f"expected={scene['expect_direct_blocked']}")

        # Via points describe application mission geometry; they do not
        # change C APF's force law or the embedded UR5 kinematics layer.
        via_points = [
            tuple(a + b for a, b in zip(
                start_mm, _vec3(offset, "via_offsets_ee_mm")))
            for offset in scene.get("via_offsets_ee_mm", [])
        ]
        result = preflight_joint_avoidance(
            library, physics, robot, [obstacle], joints, end_link, goal_mm,
            apf_config=apf,
            max_velocity_deg_s=motion["max_velocity_deg_s"],
            max_acceleration_deg_s2=motion["max_acceleration_deg_s2"],
            # None asks the bridge for the single authoritative C IK limits.
            max_fk_position_error_mm=None,
            max_fk_rotation_element_error=None,
            waypoint_capacity=scene["waypoint_capacity"],
            max_joint_step_rad=safety["max_joint_step_rad"],
            max_collision_samples=safety["max_collision_samples"],
            physics_client_id=client,
            via_points_mm=via_points,
            shortcut_cartesian=scene.get("shortcut_cartesian", False),
            minimum_full_link_clearance_m=(
                scene["minimum_full_link_clearance_mm"] / 1000.0))
        result.update(
            obstacle_ids=[obstacle],
            route_case=route_case,
            direct_cartesian_segment_blocked=direct_blocked,
            obstacle_center_m=center_m,
            obstacle_half_extents_m=half_extents_m,
        )
        return result

    return prepare
