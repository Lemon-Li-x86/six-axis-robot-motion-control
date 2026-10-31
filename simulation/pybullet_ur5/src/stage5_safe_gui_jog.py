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
import ctypes
import os
import time
from typing import Callable, Sequence

from stage5_collision_validation import InvalidPath, validate_joint_path

JOG_EXTRA_CLEARANCE_M = 0.002
JOG_MAX_SAMPLE_STEP_RAD = math.radians(0.25)
JOG_MIN_DEG = 0.5
JOG_MAX_DEG = 2.0
JOG_DEG_INCREMENT = 0.5
JOG_MAX_SUCCESSES = 3


def set_native_gui_status(text: str) -> bool:
    """OpenGL2 does not reliably render debug text; use the Windows caption.

    Only touch visible windows owned by this physics process. This is best-
    effort presentation and must never change the motion verdict.
    """
    if os.name != "nt":
        return False
    try:
        user32 = ctypes.WinDLL("user32", use_last_error=True)
        callback_type = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_void_p,
                                          ctypes.c_ssize_t)
        user32.EnumWindows.argtypes = (callback_type, ctypes.c_ssize_t)
        user32.GetWindowThreadProcessId.argtypes = (
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong))
        user32.IsWindowVisible.argtypes = (ctypes.c_void_p,)
        user32.SendMessageTimeoutW.argtypes = (
            ctypes.c_void_p, ctypes.c_uint, ctypes.c_size_t, ctypes.c_void_p,
            ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ctypes.c_size_t))
        user32.SendMessageTimeoutW.restype = ctypes.c_size_t
        caption = ctypes.create_unicode_buffer(text)
        updated = False

        @callback_type
        def visit(hwnd, _):
            nonlocal updated
            pid = ctypes.c_ulong()
            user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
            if pid.value == os.getpid() and user32.IsWindowVisible(hwnd):
                result = ctypes.c_size_t()
                # WM_SETTEXT, SMTO_ABORTIFHUNG: presentation must not wait
                # indefinitely on an unresponsive native rendering thread.
                updated = bool(user32.SendMessageTimeoutW(
                    hwnd, 0x000C, 0, ctypes.cast(caption, ctypes.c_void_p),
                    0x0002, 50, ctypes.byref(result))) or updated
            return 1

        user32.EnumWindows(visit, 0)
        return updated
    except (OSError, AttributeError, ValueError):
        return False


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
        self.finish_requested = False
        self.cancel_requested = False
        self.polls = 0
        self.last_poll_log_at = float("-inf")
        self.last_caption = None
        self.last_caption_attempt_at = float("-inf")
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
        # Dedicated orderly-exit action. Closing the native window X tears
        # down the physics server before the final UART verdict is recorded.
        self.finish_widget = physics.addUserDebugParameter(
            "SAFE JOG FINISH/EXIT: PASS after JOG, CANCEL otherwise", 1, 0, 0,
            physicsClientId=client)
        self.previous_finish_count = int(physics.readUserDebugParameter(
            self.finish_widget, physicsClientId=client))

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

    def status_text(self, done: bool) -> str:
        """Persistent feedback; a small completed jog can be hard to see."""
        if not done:
            phase = "WAIT: initial route"
        elif self.cancel_requested:
            phase = "CANCELLING"
        elif self.finish_requested:
            phase = "FINISH: checking diagnostics"
        elif self.completed < self.applied:
            phase = "BUSY: C return pending"
        elif self.events and self.events[-1]["event"] in (
                "safe_gui_jog_rejected", "safe_gui_jog_finish_rejected"):
            phase = "REFUSED: " + self.events[-1]["reason"]
        elif self.completed:
            phase = "RETURN COMPLETE: click FINISH"
        else:
            phase = "READY: choose joint/step, click +/-"
        return (f"SAFE JOG {phase} | applied={self.applied} "
                f"returned={self.completed} refused={self.refused}")

    def _authorize(self, sign: int, *, selection: tuple[int, float] | None = None) -> dict:
        """Authorize outgoing and return legs BEFORE the single pose injection."""
        p = self.physics
        try:
            axis, step_deg = (
                self._selection() if selection is None else selection)
            if selection is not None and not (
                    axis == len(self.joints) - 1 and step_deg == JOG_MIN_DEG):
                raise ValueError("CI scripted jog permits only final joint at minimum step")
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
        print("[SAFE GUI JOG PHASE] outbound_collision_preflight_started", flush=True)
        preflight_started = time.monotonic()
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
        print(
            "[SAFE GUI JOG PHASE] outbound_collision_preflight_finished "
            f"wall_s={time.monotonic()-preflight_started:.3f}",
            flush=True)

        # The preview C recovery planner reads the *current* robot pose.
        # Temporarily present the proposed state to the planner, and restore
        # every actual joint position AND velocity even on planner failure.
        try:
            for j, q in zip(self.joints, candidate):
                p.resetJointState(
                    self.robot, j, targetValue=q, targetVelocity=0,
                    physicsClientId=self.client)
            print("[SAFE GUI JOG PHASE] c_return_preview_started", flush=True)
            preview_started = time.monotonic()
            preview = self.preview_return()
            print(
                "[SAFE GUI JOG PHASE] c_return_preview_finished "
                f"wall_s={time.monotonic()-preview_started:.3f}",
                flush=True)
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

    def request_ci_scripted_j6_half_degree(self) -> dict:
        """Opt-in integration fixture: identical collision/C preview as GUI click.

        No UART request, motor bypass, fake feedback or alternate planner.
        Deliberately not called in the ordinary GUI user path.
        """
        if (self.applied != 0 or not self.available or
                self.finish_requested or self.cancel_requested):
            raise RuntimeError("CI jog may only be injected once from idle HOLD")
        event = self._authorize(
            1, selection=(len(self.joints) - 1, JOG_MIN_DEG))
        if event["event"] == "safe_gui_jog_applied":
            self.applied += 1
            self.available = False
        else:
            self.refused += 1
        event["source"] = "ci_scripted_native_gui_jog"
        self.events.append(event)
        print("[SAFE GUI JOG CI] " + str(event), flush=True)
        if event["event"] != "safe_gui_jog_applied":
            raise RuntimeError(
                "CI native GUI J6 jog refused by real collision/C planner gates: "
                + str(event))
        return event

    def request_ci_finish(self):
        """Require a settled real C/UART recovery before ending CI GUI HOLD."""
        if (self.applied != 1 or self.completed != 1
                or self.available is False or self.finish_requested
                or self.cancel_requested):
            raise RuntimeError("CI finish requires one completed real C recovery")
        self.finish_requested = True
        event = {"event": "safe_gui_jog_finish_requested",
                 "source": "ci_scripted_native_gui_jog",
                 "completed_recoveries": self.completed}
        self.events.append(event)
        print("[SAFE GUI JOG CI] " + str(event), flush=True)

    def __call__(self, physics, robot, joints, sim_time, done, ending):
        self.polls += 1
        caption = self.status_text(done)
        if (caption != self.last_caption and
                time.monotonic() - self.last_caption_attempt_at >= 1.0):
            self.last_caption_attempt_at = time.monotonic()
            if set_native_gui_status(caption):
                self.last_caption = caption
        # Bounded input telemetry distinguishes a live poll loop with no
        # received click from a worker blocked inside a native GUI call.
        if time.monotonic() - self.last_poll_log_at >= 5.0:
            print(f"[SAFE GUI INPUT] polls={self.polls} "
                  f"buttons={self.previous_counts} "
                  f"finish={self.previous_finish_count} "
                  + self.status_text(done), flush=True)
            self.last_poll_log_at = time.monotonic()
        # Always consume button events immediately. No early click can queue
        # a surprise movement when the controller later enters HOLD.
        for sign, widget in self.buttons.items():
            count = int(physics.readUserDebugParameter(
                widget, physicsClientId=self.client))
            previous = self.previous_counts[sign]
            self.previous_counts[sign] = count
            if count <= previous:
                continue
            # Persist receipt BEFORE any selection read, Bullet query or C
            # preflight so a blocked native call cannot erase the click evidence.
            self.events.append({"event": "safe_gui_jog_click_received",
                                "sign": sign, "count": count,
                                "poll": self.polls})
            print("[SAFE GUI INPUT] " + str(self.events[-1]), flush=True)
            if (not done or ending or self.finish_requested
                    or self.cancel_requested or not self.available or
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

        # Consume and validate the finish-button edge every physics frame,
        # including clicks during motion. Never queue an early finish.
        finish_count = int(physics.readUserDebugParameter(
            self.finish_widget, physicsClientId=self.client))
        finish_changed = finish_count > self.previous_finish_count
        self.previous_finish_count = finish_count
        if finish_changed:
            if (not done or ending or self.finish_requested or
                    self.cancel_requested or self.completed != self.applied):
                event = {"event": "safe_gui_jog_finish_rejected",
                         "reason": "recovery_pending_or_not_idle"}
            elif self.applied == 0:
                # A test with no jog must have an exit path. Record an
                # explicit cancellation, never a successful recovery.
                self.cancel_requested = True
                event = {"event": "safe_gui_jog_cancel_requested",
                         "reason": "no_guarded_jog_requested"}
            else:
                self.finish_requested = True
                event = {"event": "safe_gui_jog_finish_requested",
                         "completed_recoveries": self.completed}
            self.events.append(event)
            print("[SAFE GUI JOG] " + str(event), flush=True)
