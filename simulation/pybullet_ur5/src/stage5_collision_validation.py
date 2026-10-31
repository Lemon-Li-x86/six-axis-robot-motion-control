"""Simulation-only full-link static-obstacle gate for a *joint* waypoint path.

The caller must convert C APF Cartesian waypoints using the project's
kinematics/IK and enforce task-time limits separately. This gate does not
perform IK and never commands production actuators.

Pose interpolation is discrete: a finite sample spacing does not prove
continuous collision-free motion between adjacent samples.
"""
from __future__ import annotations

import math
from typing import Sequence


class InvalidPath(ValueError):
    """Malformed/over-budget simulation path; do not treat as passed."""


def validate_joint_path(
    physics,
    robot_id: int,
    obstacle_ids: Sequence[int],
    joint_indices: Sequence[int],
    path_rad: Sequence[Sequence[float]],
    *,
    maximum_joint_step_rad: float,
    clearance_m: float,
    maximum_samples: int,
    physics_client_id: int,
) -> dict:
    """Sample every robot link, reject any static-obstacle proximity.

    Arguments intentionally inject numerical policy. Joint limits come
    only from getJointInfo() on the authoritative loaded URDF; no duplicate
    UR5 bounds. Reverts the robot's exact preflight joint positions/velocities
    even when a collision or a PyBullet exception occurs.
    """
    if not (math.isfinite(maximum_joint_step_rad) and
            maximum_joint_step_rad > 0.0 and
            math.isfinite(clearance_m) and clearance_m >= 0.0 and
            isinstance(maximum_samples, int) and maximum_samples > 0):
        raise InvalidPath("invalid path validation policy")
    joints = tuple(joint_indices)
    obstacles = tuple(obstacle_ids)
    if (not joints or not obstacles or len(joints) != len(set(joints)) or
            len(obstacles) != len(set(obstacles)) or robot_id in obstacles):
        raise InvalidPath("joint or obstacle indices must be nonempty, unique and distinct")
    count = physics.getNumJoints(robot_id, physicsClientId=physics_client_id)
    if any(not isinstance(j, int) or j < 0 or j >= count for j in joints):
        raise InvalidPath("joint index outside URDF")
    poses = tuple(tuple(float(x) for x in pose) for pose in path_rad)
    if not poses or any(len(pose) != len(joints) or
                        not all(math.isfinite(x) for x in pose) for pose in poses):
        raise InvalidPath("joint path contains missing, nonfinite or wrong-width angles")

    limits = []
    for joint in joints:
        info = physics.getJointInfo(robot_id, joint, physicsClientId=physics_client_id)
        if info[2] != physics.JOINT_REVOLUTE:
            raise InvalidPath("only explicit revolute UR5 joints are supported")
        lo, hi = float(info[8]), float(info[9])
        if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
            raise InvalidPath("URDF joint has no finite model travel bounds")
        limits.append((lo, hi))

    for pose in poses:
        if any(angle < lo - 1e-9 or angle > hi + 1e-9
               for angle, (lo, hi) in zip(pose, limits)):
            raise InvalidPath("path exceeds authoritative URDF joint limits")

    # Budget the entire scan before any physics mutation. Include first pose.
    samples_needed = 1
    intervals = []
    for start, end in zip(poses, poses[1:]):
        largest = max(abs(b-a) for a,b in zip(start,end))
        subdivisions = max(1, math.ceil(largest / maximum_joint_step_rad))
        intervals.append(subdivisions)
        samples_needed += subdivisions
        if samples_needed > maximum_samples:
            raise InvalidPath("sampling budget exhausted, refusing path")
    if samples_needed > maximum_samples:
        raise InvalidPath("sampling budget exhausted, refusing path")

    previous = tuple(
        physics.getJointState(robot_id, j, physicsClientId=physics_client_id)[:2]
        for j in joints
    )
    sampled = 0
    minimum_measured_clearance_m = float("inf")
    nearest_link_index = None
    # Same real Bullet geometry; a wider *measurement* search radius does
    # not relax the 10 mm (or scene-specific) rejection threshold.
    measurement_radius_m = max(clearance_m, 0.1)
    try:
        def inspect(pose):
            nonlocal sampled, minimum_measured_clearance_m, nearest_link_index
            for joint, angle in zip(joints, pose):
                physics.resetJointState(
                    robot_id, joint, targetValue=angle, targetVelocity=0.0,
                    physicsClientId=physics_client_id)
            physics.performCollisionDetection(physicsClientId=physics_client_id)
            sampled += 1
            # Explicitly enumerate *all* links, including base (-1);
            # relying on default getClosestPoints link filters is ambiguous.
            for link in range(-1, count):
                for obstacle in obstacles:
                    contacts = physics.getClosestPoints(
                        robot_id, obstacle, measurement_radius_m,
                        linkIndexA=link, linkIndexB=-1,
                        physicsClientId=physics_client_id)
                    if contacts:
                        distance = float(min(c[8] for c in contacts))
                        if distance < minimum_measured_clearance_m:
                            minimum_measured_clearance_m = distance
                            nearest_link_index = link
                        if distance < clearance_m:
                            return {
                                "passed": False,
                                "sampled_states": sampled,
                                "collision": {
                                    "robot_link_index": link,
                                    "obstacle_id": obstacle,
                                    "signed_distance_m": distance,
                                },
                                "minimum_measured_clearance_m":
                                    minimum_measured_clearance_m,
                                "nearest_link_index": nearest_link_index,
                                "scope": "discrete_full_link_static_obstacle_clearance",
                            }
            return None

        collision = inspect(poses[0])
        if collision is not None:
            return collision
        for segment, subdivisions in enumerate(intervals):
            start, finish = poses[segment], poses[segment+1]
            for step in range(1, subdivisions+1):
                alpha = step / subdivisions
                angles = tuple(a+(b-a)*alpha for a,b in zip(start,finish))
                collision = inspect(angles)
                if collision is not None:
                    return collision
        return {
            "passed": True,
            "sampled_states": sampled,
            "collision": None,
            "minimum_measured_clearance_m": (
                minimum_measured_clearance_m if math.isfinite(minimum_measured_clearance_m)
                else None),
            "nearest_link_index": nearest_link_index,
            "clearance_search_radius_m": measurement_radius_m,
            "scope": "discrete_full_link_static_obstacle_clearance",
        }
    finally:
        for joint, (position, velocity) in zip(joints, previous):
            physics.resetJointState(
                robot_id, joint, targetValue=position,
                targetVelocity=velocity, physicsClientId=physics_client_id)
        physics.performCollisionDetection(physicsClientId=physics_client_id)
