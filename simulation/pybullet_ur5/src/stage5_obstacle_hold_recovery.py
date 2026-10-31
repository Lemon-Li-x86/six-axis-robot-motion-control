"""Stage5 simulation-only recovery authorization from an observed six-axis pose.

Production C APF/IK generates Cartesian/IK intermediate goals.  The original
six-axis HOLD goal is appended and *the entire joint path* is checked against
actual PyBullet robot/obstacle geometry before any UART request may be issued.
This module does not actuate a robot or grant native GUI mouse-picking safety.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Sequence

from stage4_dynamic_closed_loop import DEFAULT_JSON
from stage5_avoidance_uart_gate import load_scene
from stage5_avoidance_preflight import preflight_joint_avoidance, RejectedRoute
from stage5_collision_validation import validate_joint_path, InvalidPath
from stage5_static_obstacle_probe import Config

JOINT_COUNT = 6


def plan_obstacle_hold_recovery(
    library: Path, physics, robot: int, joints: Sequence[int],
    obstacles: Sequence[int], client: int,
    original_goal_m: Sequence[float],
    exact_goal_joint_rad: Sequence[float], route_case: str,
    *, minimum_extra_clearance_m: float = 0.0,
) -> dict:
    """Plan from ACTUAL live joint pose, fail closed until whole-arm tested.

    The first plan honors the current EE orientation; the final original
    joint pose may differ in orientation and is a separate C-firmware joint
    request, checked as a complete additional interpolated joint leg.
    Everything uses the same physical obstacle IDs and authoritative scene.
    """
    scene, safety = load_scene(route_case)
    if not (len(joints) == len(exact_goal_joint_rad) == JOINT_COUNT
            and len(original_goal_m) == 3
            and obstacles
            and all(math.isfinite(float(v)) for v in
                    (*original_goal_m, *exact_goal_joint_rad))
            and math.isfinite(minimum_extra_clearance_m)
            and minimum_extra_clearance_m >= 0.0):
        raise RejectedRoute("invalid dynamic recovery goal/geometry/policy")

    required_clearance = (
        scene["minimum_full_link_clearance_mm"] / 1000.0
        + minimum_extra_clearance_m
    )
    end_link = next((
        index for index in range(physics.getNumJoints(robot, physicsClientId=client))
        if physics.getJointInfo(robot, index, physicsClientId=client)[12] == b"ee_link"
    ), None)
    if end_link is None:
        raise RejectedRoute("UR5 end effector link absent")

    current = tuple(physics.getJointState(
        robot, joint, physicsClientId=client)[0] for joint in joints)
    # The existing authoritative collision gate includes the start pose.
    # If a human already drove into the exclusion boundary, motion must
    # remain blocked; we do not claim recovery from an unsafe start.
    try:
        start_gate = validate_joint_path(
            physics, robot, obstacles, joints, [current],
            maximum_joint_step_rad=safety["max_joint_step_rad"],
            clearance_m=required_clearance,
            maximum_samples=safety["max_collision_samples"],
            physics_client_id=client)
    except InvalidPath as error:
        raise RejectedRoute(f"recovery start pose invalid: {error}") from error
    if not start_gate["passed"]:
        raise RejectedRoute(
            "unsafe recovery start pose: " + str(start_gate["collision"]))

    motion = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))["stage4_motion"]
    apf = Config(*(float(scene["apf"][key]) for key, _ in Config._fields_))
    c_route = preflight_joint_avoidance(
        library, physics, robot, obstacles, joints, end_link,
        [1000.0 * v for v in original_goal_m],
        apf_config=apf,
        max_velocity_deg_s=motion["max_velocity_deg_s"],
        max_acceleration_deg_s2=motion["max_acceleration_deg_s2"],
        max_fk_position_error_mm=None,
        max_fk_rotation_element_error=None,
        waypoint_capacity=scene["waypoint_capacity"],
        max_joint_step_rad=safety["max_joint_step_rad"],
        max_collision_samples=safety["max_collision_samples"],
        physics_client_id=client,
        minimum_full_link_clearance_m=required_clearance,
        # Original via-offsets are tied to the INITIAL mission start, not
        # this displaced pose. No fabricated waypoint for recovery.
        via_points_mm=(),
        shortcut_cartesian=scene.get("shortcut_cartesian", False),
    )
    last = tuple(float(x) for x in exact_goal_joint_rad)
    path = list(c_route["joint_waypoints_rad"])
    if any(abs(a-b) > 1e-7 for a,b in zip(last, path[-1])):
        path.append(last)
    if len(path) < 2:
        raise RejectedRoute("recovery route has no actual motion")
    try:
        full_gate = validate_joint_path(
            physics, robot, obstacles, joints, path,
            maximum_joint_step_rad=safety["max_joint_step_rad"],
            clearance_m=required_clearance,
            maximum_samples=safety["max_collision_samples"],
            physics_client_id=client)
    except InvalidPath as error:
        raise RejectedRoute(f"complete recovery joint path invalid: {error}") from error
    if not full_gate["passed"]:
        raise RejectedRoute(
            "complete recovery full-link route rejected: "
            + str(full_gate["collision"]))
    return {
        "passed": True,
        "scope": "unactuated_discrete_recovery_from_measured_pose",
        "joint_waypoints_rad": path,
        "C_apf_ik_joint_segments": len(c_route["joint_waypoints_rad"]) - 1,
        "exact_final_joint_goal_segment": (
            len(path) > len(c_route["joint_waypoints_rad"])),
        "minimum_full_link_clearance_m": required_clearance,
        "minimum_measured_full_link_clearance_m":
            full_gate["minimum_measured_clearance_m"],
        "sampled_full_arm_states": full_gate["sampled_states"],
        "original_goal_m": list(original_goal_m),
        "start_joint_rad": list(current),
        "target_joint_rad": list(last),
        "C_apf_ik_plan": c_route,
        "not_certified": [
            "physical swept inter-sample clearance",
            "manual mouse-picking safety",
            "dynamic obstacles",
            "UART execution",
        ],
    }
