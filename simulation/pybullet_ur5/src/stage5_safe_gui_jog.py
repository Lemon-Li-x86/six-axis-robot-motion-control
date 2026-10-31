"""Opt-in bounded GUI *button jog* for Stage5 simulated obstacle HOLD.

Never enable raw PyBullet native mouse picking in obstacle scenes. Every
requested joint increment is checked against the *loaded GUI joint limits*
and the authoritative full-link Bullet obstacle scene BEFORE any physics
mutation. The actual C APF/IK hold recovery is performed by the caller.

This is controlled simulated repositioning, not continuous physical torque
or a preventive safety system for unrestricted mouse dragging.
"""
from __future__ import annotations

import math
from typing import Sequence

from stage5_collision_validation import InvalidPath, validate_joint_path

JOG_AXIS = 0
JOG_OFFSET_DEG = 2.0
JOG_EXTRA_CLEARANCE_M = 0.002
JOG_MAX_SAMPLE_STEP_RAD = math.radians(0.25)


class SafeGuiJog:
    """One-shot guarded button action; no uncontrolled native mouse constraints."""

    def __init__(self, physics, robot: int, joints: Sequence[int],
                 obstacles: Sequence[int], client: int,
                 joint_limits_rad: Sequence[tuple[float, float]],
                 *, maximum_joint_step_rad: float,
                 minimum_clearance_m: float, maximum_samples: int):
        if len(joints) != 6 or len(joint_limits_rad) != 6 or not obstacles:
            raise ValueError("Safe GUI jog needs loaded six-axis robot and obstacles")
        self.physics = physics
        self.robot = robot
        self.joints = tuple(joints)
        self.obstacles = tuple(obstacles)
        self.client = client
        self.joint_limits_rad = tuple(joint_limits_rad)
        self.maximum_joint_step_rad = maximum_joint_step_rad
        self.minimum_clearance_m = minimum_clearance_m
        self.maximum_samples = maximum_samples
        self.events: list[dict] = []
        self.applied = 0
        self.refused = 0
        self.available = True
        self.buttons = {
            1: physics.addUserDebugParameter(
                "SAFE APF JOG: J1 +2 deg (once in HOLD)", 1, 0, 0,
                physicsClientId=client),
            -1: physics.addUserDebugParameter(
                "SAFE APF JOG: J1 -2 deg (once in HOLD)", 1, 0, 0,
                physicsClientId=client),
        }
        self.previous_counts = {
            sign: int(physics.readUserDebugParameter(
                widget, physicsClientId=client))
            for sign, widget in self.buttons.items()
        }

    def _authorize(self, sign: int) -> dict:
        """Preview the entire displacement; leave Bullet unchanged on rejection."""
        p = self.physics
        current = tuple(p.getJointState(
            self.robot, joint, physicsClientId=self.client)[0]
            for joint in self.joints)
        proposed = list(current)
        proposed[JOG_AXIS] += math.radians(sign * JOG_OFFSET_DEG)
        lower, upper = self.joint_limits_rad[JOG_AXIS]
        if not (lower <= proposed[JOG_AXIS] <= upper):
            return {"event": "safe_gui_jog_rejected",
                    "reason": "GUI_joint_limit", "sign": sign}
        try:
            gate = validate_joint_path(
                p, self.robot, self.obstacles, self.joints,
                [current, proposed],
                maximum_joint_step_rad=min(
                    self.maximum_joint_step_rad, JOG_MAX_SAMPLE_STEP_RAD),
                clearance_m=(self.minimum_clearance_m
                             + JOG_EXTRA_CLEARANCE_M),
                maximum_samples=self.maximum_samples,
                physics_client_id=self.client)
        except InvalidPath as exc:
            return {"event": "safe_gui_jog_rejected",
                    "reason": "invalid_full_arm_path",
                    "detail": str(exc), "sign": sign}
        if not gate["passed"]:
            return {"event": "safe_gui_jog_rejected",
                    "reason": "full_arm_clearance",
                    "collision": gate["collision"], "sign": sign}
        # Validation restores all states (including velocities), so the only
        # state modification occurs AFTER successful authorization.
        p.resetJointState(
            self.robot, self.joints[JOG_AXIS],
            targetValue=proposed[JOG_AXIS], targetVelocity=0.0,
            physicsClientId=self.client)
        return {"event": "safe_gui_jog_applied",
                "sign": sign, "axis": JOG_AXIS,
                "displacement_deg": sign * JOG_OFFSET_DEG,
                "required_clearance_m": (self.minimum_clearance_m
                                         + JOG_EXTRA_CLEARANCE_M),
                "minimum_predicted_clearance_m":
                    gate["minimum_measured_clearance_m"],
                "sampled_states": gate["sampled_states"]}

    def __call__(self, physics, robot, joints, sim_time, done, ending):
        # Process button edges even before HOLD but DISCARD early commands,
        # so a clicked jog cannot unexpectedly apply later.
        for sign, widget in self.buttons.items():
            count = int(physics.readUserDebugParameter(
                widget, physicsClientId=self.client))
            previous = self.previous_counts[sign]
            self.previous_counts[sign] = count
            if count <= previous:
                continue
            if not done or ending or not self.available:
                event = {"event": "safe_gui_jog_rejected", "sign": sign,
                         "reason": "not_in_idle_hold_or_already_used"}
            else:
                event = self._authorize(sign)
                if event["event"] == "safe_gui_jog_applied":
                    self.available = False
                    self.applied += 1
                else:
                    self.refused += 1
            self.events.append(event)
            print("[SAFE GUI JOG] " + str(event), flush=True)
