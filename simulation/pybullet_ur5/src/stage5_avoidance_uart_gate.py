"""Explicit simulation-only static-box fixture and C-verified avoidance gate.

Actual UR5 angles, AABB, configuration, and model come from their existing
authoritative sources. No pre-canned joint trajectory or bypass of C IK.
Nothing sends a UART command until all-link preflight reports success.
"""
from __future__ import annotations
import ctypes
import json
import math
from pathlib import Path

from stage4_dynamic_closed_loop import DEFAULT_JSON
from stage5_static_obstacle_probe import Config, Point, AABB, point
from stage5_avoidance_preflight import preflight_joint_avoidance


def make_avoidance_preflight(library: Path, route_case: str):
    if route_case not in {"clear", "blocked"}:
        raise ValueError("Unknown avoidance fixture")
    defaults = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))
    motion = defaults["stage4_motion"]

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
        x, y, z = physics.getLinkState(
            robot, end_link, computeForwardKinematics=True,
            physicsClientId=client)[4]
        if route_case == "clear":
            # First integration fixture: the complete motor path is tested
            # with a real obstacle outside the swept workcell. This proves
            # route serialization/feedback without claiming a detour.
            center = (5.0, 5.0, 5.0)
            half_extents = (0.05, 0.05, 0.05)
            goal = (1000*x+3.0, 1000*y+1.5, 1000*z)
            apf = Config(1.0, 0.0, 30.0, 5.0, 1.5, 0.1)
            capacity = 16
        else:
            # Experiment fixture: off-center box blocks the nominal straight
            # EE segment. Reject if C APF/IK or full-link geometry is unsafe.
            center = (x-0.075, y+0.025, z)
            half_extents = (0.015, 0.020, 0.020)
            goal = (1000*x-150.0, 1000*y+60.0, 1000*z)
            apf = Config(1.0, 50000.0, 150.0, 10.0, 10.0, 2.0)
            capacity = 96

        shape = physics.createCollisionShape(
            physics.GEOM_BOX, halfExtents=half_extents,
            physicsClientId=client)
        obstacle = physics.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=shape,
            basePosition=center, physicsClientId=client)
        direct_blocked = False
        if route_case == "blocked":
            # The box must really intersect the direct point segment.
            # Check with the SAME production C segment/AABB intersection,
            # not an independent Python geometry approximation.
            low, high = physics.getAABB(
                obstacle, physicsClientId=client)
            bounds = AABB(point([1000*v for v in low]),
                          point([1000*v for v in high]))
            start_mm = point((1000*x, 1000*y, 1000*z))
            goal_point = point(goal)
            direct = Config(
                apf.attraction_gain, 0.0, apf.influence_distance_mm,
                apf.clearance_mm, math.dist(tuple(start_mm.mm), goal),
                apf.goal_tolerance_mm)
            c = ctypes.CDLL(str(library.resolve()))
            c.robot_apf_step.restype = ctypes.c_int
            c.robot_apf_step.argtypes = [
                ctypes.POINTER(Point), ctypes.POINTER(Point),
                ctypes.POINTER(AABB), ctypes.c_size_t,
                ctypes.POINTER(Config), ctypes.POINTER(Point),
                ctypes.c_void_p,
            ]
            candidate = Point()
            straight_status = c.robot_apf_step(
                ctypes.byref(start_mm), ctypes.byref(goal_point),
                ctypes.byref(bounds), 1,
                ctypes.byref(direct), ctypes.byref(candidate), None)
            if straight_status != -10:  # Existing C NO_SOLUTION enum.
                raise ValueError(
                    "Fixture did not block a valid direct segment: "
                    f"C_APF_status={straight_status}")
            direct_blocked = True
        result = preflight_joint_avoidance(
            library, physics, robot, [obstacle], joints, end_link, goal,
            apf_config=apf,
            max_velocity_deg_s=motion["max_velocity_deg_s"],
            max_acceleration_deg_s2=motion["max_acceleration_deg_s2"],
            max_fk_position_error_mm=0.5,
            max_fk_rotation_element_error=0.003,
            waypoint_capacity=capacity,
            max_joint_step_rad=0.02,
            max_collision_samples=3000,
            physics_client_id=client)
        result["obstacle_ids"] = [obstacle]
        result["route_case"] = route_case
        result["direct_cartesian_segment_blocked"] = direct_blocked
        result["obstacle_center_m"] = center
        result["obstacle_half_extents_m"] = half_extents
        return result

    return prepare
