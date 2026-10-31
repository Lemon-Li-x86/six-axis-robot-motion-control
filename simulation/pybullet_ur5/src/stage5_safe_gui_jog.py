"""Stage5 opt-in bounded six-axis GUI jog; no native physics mouse picking.

Only a named PyBullet GUI panel button can request a simulated incremental
joint move. Before changing Bullet state, validate: current URDF/GUI travel,
the entire sampled physical-link outbound joint path, and a production C
APF/IK recovery path from the proposed pose back to the exact six-axis goal.
A refused request NEVER applies the displacement or sends UART motion.

This is NOT a safety certificate for physical motors or continuous mouse
dragging; resetJointState is a deterministic simulation-only test actuator.
"""
from __future__ import annotations

import math
from typing import Callable, Sequence

from stage5_collision_validation import InvalidPath, validate_joint_path

JOG_EXTRA_CLEARANCE_M = 0.002
JOG_MAX_SAMPLE_STEP_RAD = math.radians(0.25)
JOG_MIN_DEG = 0.5
JOG_MAX_DEG = 2.0
JOG_DEG_INCREMENT = 0.5
JOG_MAX_SUCCESSES = 3


class SafeGuiJog:
    """Six-axis selected GUI jog, fail-closed preflight, one action per recovery."""

    def __init__(
        self, physics, robot: int, joints: Sequence[int],
        obstacles: Sequence[int], client: int,
        joint_limits_rad: Sequence[tuple[float, float]],
        *, maximum_joint_step_rad: float, minimum_clearance_m: float,
        maximum_samples: int, preview_return: Callable[[], dict],
    ):
        if (len(joints) != 6 or len(joint_limits_rad) != 6 or
                not obstacles or not callable(preview_return) or
                not math.isfinite(minimum_clearance_m) or
                minimum_clearance_m < 0.010):
            raise ValueError("Safe GUI jog needs six loaded joints, obstacles, 10mm minimum and C planner")
        self.physics = physics
        self.robot = robot
        self.joints = tuple(joints)
        self.obstacles = tuple(obstacles)
        self.client = client
        self.joint_limits_rad = tuple(joint_limits_rad)
        self.maximum_joint_step_rad = maximum_joint_step_rad
        self.minimum_clearance_m = minimum_clearance_m
        self.maximum_samples = maximum_samples
        self.preview_return = preview_return
        self.extra_clearance_m = JOG_EXTRA_CLEARANCE_M
        self.events: list[dict] = []
        self.applied = 0
        self.refused = 0
        self.completed = 0
        self.available = True
        self.axis_widget = physics.addUserDebugParameter(
            "SAFE JOG: selected joint J1..J6", 1.0, 6.0, 1.0,
            physicsClientId=client)
        self.step_widget = physics.addUserDebugParameter(
            "SAFE JOG: step deg (0.5,1,1.5,2)", JOG_MIN_DEG,
            JOG_MAX_DEG, JOG_MAX_DEG, physicsClientId=client)
        self.buttons = {
            1: physics.addUserDebugParameter(
                "SAFE APF JOG: selected J + step", 1, 0, 0,
                physicsClientId=client),
            -1: physics.addUserDebugParameter(
                "SAFE APF JOG: selected J - step", 1, 0, 0,
                physicsClientId=client),
        }
        self.previous_counts = {
            sign: int(physics.readUserDebugParameter(
                widget, physicsClientId=client))
            for sign, widget in self.buttons.items()
        }

    def _selection(self):
        raw_axis = float(self.physics.readUserDebugParameter(
            self.axis_widget, physicsClientId=self.client))
        raw_step = float(self.physics.readUserDebugParameter(
            self.step_widget, physicsClientId=self.client))
        if not (math.isfinite(raw_axis) and math.isfinite(raw_step) and
                1 <= raw_axis <= 6 and JOG_MIN_DEG <= raw_step <= JOG_MAX_DEG):
            raise ValueError("out-of-range GUI jog selection")
        # GUI slider values are continuous. Snap to the documented discrete
        # settings, never silently allow a move larger than 2 degrees.
        axis = min(5, max(0, math.floor(raw_axis + 0.5) - 1))
        step = min(JOG_MAX_DEG, max(JOG_MIN_DEG,
            math.floor(raw_step / JOG_DEG_INCREMENT + 0.5)
            * JOG_DEG_INCREMENT))
        return axis, step

    def complete_recovery(self):
        """Only the actual ACK+physical-settled C trajectory may re-arm the GUI."""
        if self.available or self.completed >= self.applied:
            raise RuntimeError("Safe jog recovery completion without an in-flight jog")
        self.completed += 1
        self.available = self.applied < JOG_MAX_SUCCESSES

    def _authorize(self, sign: int) -> dict:
        """Authorize outgoing and return legs BEFORE the single pose injection."""
        p = self.physics
        try:
            axis, step_deg = self._selection()
        except (ValueError, TypeError) as error:
            return {"event": "safe_gui_jog_rejected",
                    "reason": "invalid_selection", "detail": str(error), "sign": sign}
        current_states = tuple(p.getJointState(
            self.robot, j, physicsClientId=self.client)[:2] for j in self.joints)
        current = tuple(v[0] for v in current_states)
        candidate = list(current)
        candidate[axis] += math.radians(sign * step_deg)
        base = {"sign": sign, "axis": axis, "joint": axis + 1,
                "displacement_deg": sign * step_deg}
        lower, upper = self.joint_limits_rad[axis]
        if not (lower <= candidate[axis] <= upper):
            return {"event": "safe_gui_jog_rejected", "reason": "GUI_joint_limit", **base}
        required = self.minimum_clearance_m + JOG_EXTRA_CLEARANCE_M
        try:
            gate = validate_joint_path(
                p, self.robot, self.obstacles, self.joints, [current, candidate],
                maximum_joint_step_rad=min(
                    self.maximum_joint_step_rad, JOG_MAX_SAMPLE_STEP_RAD),
                clearance_m=required, maximum_samples=self.maximum_samples,
                physics_client_id=self.client)
        except InvalidPath as exc:
            return {"event": "safe_gui_jog_rejected",
                    "reason": "invalid_full_arm_path", "detail": str(exc), **base}
        if not gate["passed"]:
            return {"event": "safe_gui_jog_rejected",
                    "reason": "full_arm_clearance", "collision": gate["collision"], **base}

        # The preview C recovery planner reads the *current* robot pose.
        # Temporarily present the proposed state to the planner, and restore
        # every actual joint position AND velocity even on planner failure.
        try:
            for j, q in zip(self.joints, candidate):
                p.resetJointState(
                    self.robot, j, targetValue=q, targetVelocity=0,
                    physicsClientId=self.client)
            preview = self.preview_return()
        except (ValueError, RuntimeError) as exc:
            return {"event": "safe_gui_jog_rejected",
                    "reason": "recovery_preflight_failed",
                    "detail": str(exc), **base}
        finally:
            for j, (position, velocity) in zip(self.joints, current_states):
                p.resetJointState(
                    self.robot, j, targetValue=position,
                    targetVelocity=velocity, physicsClientId=self.client)
            p.performCollisionDetection(physicsClientId=self.client)
        if (not preview.get("passed") or
                preview.get("minimum_full_link_clearance_m", 0) < self.minimum_clearance_m):
            return {"event": "safe_gui_jog_rejected",
                    "reason": "recovery_preflight_failed", **base}

        # The only permanent state change follows all checks.
        p.resetJointState(
            self.robot, self.joints[axis], targetValue=candidate[axis],
            targetVelocity=0.0, physicsClientId=self.client)
        return {
            "event": "safe_gui_jog_applied", **base,
            "required_clearance_m": required,
            "minimum_predicted_clearance_m": gate["minimum_measured_clearance_m"],
            "return_plan_minimum_clearance_m":
                preview.get("minimum_measured_full_link_clearance_m"),
            "return_segments": len(preview["joint_waypoints_rad"]) - 1,
            "sampled_states": gate["sampled_states"],
        }

    def __call__(self, physics, robot, joints, sim_time, done, ending):
        # Always consume button events immediately. No early click can queue
        # a surprise movement when the controller later enters HOLD.
        for sign, widget in self.buttons.items():
            count = int(physics.readUserDebugParameter(
                widget, physicsClientId=self.client))
            previous = self.previous_counts[sign]
            self.previous_counts[sign] = count
            if count <= previous:
                continue
            if (not done or ending or not self.available or
                    self.applied >= JOG_MAX_SUCCESSES):
                event = {"event": "safe_gui_jog_rejected", "sign": sign,
                         "reason": "not_idle_or_recovery_pending_or_limit"}
                self.refused += 1
            else:
                event = self._authorize(sign)
                if event["event"] == "safe_gui_jog_applied":
                    self.available = False
                    self.applied += 1
                else:
                    self.refused += 1
            self.events.append(event)
            print("[SAFE GUI JOG] " + str(event), flush=True)
