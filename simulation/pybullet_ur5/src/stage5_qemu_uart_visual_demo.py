"""可视化 QEMU Cortex-M4 / FreeRTOS / UART / PyBullet 多段关节闭环演示。

职责边界：
    - ARM 固件负责关节轨迹、PID、限幅和任务调度。
    - 本脚本仅负责模拟 UART 对端、PyBullet 物理推进、显示和记录。
    - PyBullet POSITION_CONTROL 是模拟执行器，不等同于真实电机驱动。

与 stage5_qemu_uart_closed_loop.py 的自动验收相互独立；本脚本不修改
固件配置、PID 参数、UR5 几何或已有自动验收判据。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import select
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from collections import deque
from pathlib import Path
from typing import Callable, Sequence

import qemu_uart_receiver as wire
from qemu_clock_gate import QemuClockGate
import stage5_qemu_uart_closed_loop as integration
from stage4_dynamic_closed_loop import (
    JOINT_NAMES,
    ROOT,
    START_RAW,
    URDF,
    DEFAULT_JSON,
    angles_raw,
    joint_indices,
)


# ============================================================
# Demonstration-only fixtures and thresholds
# ============================================================

# Centidegrees, in the canonical six-joint order from the shared model.
# These are motion *requests*, not robot limits or PID parameters.
SCENARIOS: dict[str, tuple[tuple[int, ...], ...]] = {
    "joint_sweep": (
        tuple(a + b for a, b in zip(START_RAW, (0, 1500, 0, 0, 0, 0))),
        tuple(a + b for a, b in zip(START_RAW, (0, -1000, 0, 0, 0, 0))),
        tuple(START_RAW),
    ),
    "six_axis": (
        tuple(a + 800 for a in START_RAW),
        tuple(a - 800 for a in START_RAW),
        tuple(START_RAW),
    ),
}

PHYSICS_STEP_S = integration.PHYSICS_DT_S
FEEDBACK_STEP_S = integration.FEEDBACK_PERIOD_S
SIMULATED_STEPS_PER_TARGET = round(FEEDBACK_STEP_S / PHYSICS_STEP_S)
# Preserve exactly the old 100 internal warm-up steps, independent of GUI API calls.
INITIAL_SETTLE_INTERNAL_STEPS = 100
SETTLE_TOLERANCE_DEG = 0.15
SETTLE_FRAME_COUNT = 15
# Interactive HOLD policies (not UR5 model parameters or PID gains).
HOLD_DEVIATION_TRIGGER_DEG = 0.5
HOLD_STAGNANT_PERIOD_S = 0.5
# A failed/stopped C trajectory may need one bounded rearm. Never restart a
# slowly moving C plan merely because measured joint movement is small.
HOLD_RETRY_COOLDOWN_S = 3.0
HOLD_MAX_REPLAN_ATTEMPTS = 3
HOLD_TRACE_LIFETIME_S = 10.0
# Log costly UART batches at most once a second. Diagnostic-only, no pacing.
UART_BATCH_WARNING_FRAMES = 24
UART_BATCH_WARNING_S = 0.15
DIAGNOSTICS_TIMEOUT_S = 5.0  # Default; simulator observation can override.
UART_SILENCE_TIMEOUT_S = 8.0  # Existing live QEMU UART watchdog window.
# Acceptance criterion for physically meaningful temporal correspondence.
# Counts are diagnostic estimates, not proof of exact FreeRTOS tick alignment.
CLOCK_ALIGNMENT_MAX_RELATIVE_ERROR = 0.25
# GUI-only physics solver settings; never robot model/firmware joint limits.
GUI_JOINT_STOP_SOLVER_ITERATIONS = 80
GUI_JOINT_STOP_FORCE_MULTIPLIER = 20.0
# Numerical penetration allowance for the native Bullet joint-limit solver.
# This is NOT extra authorized travel and is not fed to the C firmware as a limit.
GUI_JOINT_STOP_NUMERICAL_SLOP_DEG = 0.10
# Simulation GUI picking policy only, NOT a substitute for UR5 URDF/C travel.
# Firmware and DIRECT validation retain the full manufacturer-model range.
GUI_JOINT_TRAVEL_MODES = ("single-turn", "urdf")
DEFAULT_GUI_JOINT_TRAVEL_MODE = "single-turn"
TRACE_HEADER = (
    "simulated_time_s", "segment", "tool_x_m", "tool_y_m", "tool_z_m",
    *(f"command_{name}_deg" for name in JOINT_NAMES),
    *(f"servo_target_{name}_deg" for name in JOINT_NAMES),
    *(f"actual_{name}_deg" for name in JOINT_NAMES),
    *(f"error_{name}_deg" for name in JOINT_NAMES),
)


@dataclass(frozen=True)
class DemoOptions:
    """运行时演示设置；时间以秒计，距离以米计。"""

    scenario: str
    headless: bool
    frame_delay_s: float
    timeout_s: float
    hold_window: bool
    interactive_hold: bool = False
    hold_duration_s: float | None = None
    opengl2: bool = False  # Native PyBullet GUI renderer option; DIRECT is unchanged.
    diagnostics_timeout_s: float = DIAGNOSTICS_TIMEOUT_S
    hold_observe_only: bool = False  # Passive PID observation, no replan requests.
    gui_joint_travel: str = DEFAULT_GUI_JOINT_TRAVEL_MODE
    # Internal DIRECT/QEMU synthetic disturbance regression ONLY.
    # Explicitly not enabled for unrestricted human GUI mouse dragging.
    test_obstacle_hold_recovery: bool = False
    # Explicit native GUI guarded-button trial, never native mouse picking.
    gui_safe_jog: bool = False
    # CI opt-in only: scripts exactly one guarded J6 minimum-step jog in
    # native p.GUI; NEVER used by operator runs or on real motors.
    ci_scripted_safe_jog: bool = False


class Stage5GuiDisconnectedError(RuntimeError):
    """Native PyBullet client went away before a final measured verdict."""

    fault_type = "stage5_gui_disconnected"


class Stage5UnvalidatedHoldRecoveryError(ValueError):
    """Obstacle-aware recovery route unavailable; refuse blind C joint rearm."""

    fault_type = "stage5_unvalidated_hold_recovery_blocked"

    def __init__(self, measured_raw: Sequence[int], goal_raw: Sequence[int]):
        self.measured_raw = tuple(measured_raw)
        self.goal_raw = tuple(goal_raw)
        self.deviation_deg = max(
            abs(actual - goal) / wire.JOINT_ANGLE_SCALE
            for actual, goal in zip(self.measured_raw, self.goal_raw))
        super().__init__(
            "Stage5 obstacle HOLD recovery refused: measured deviation "
            f"{self.deviation_deg:.2f}deg. No collision-validated return route "
            "exists from the dragged pose. Firmware rearm NOT sent.")


def apf_hold_mouse_picking_allowed(
    headless: bool, has_avoidance_plan: bool,
) -> bool:
    """Never expose raw GUI mouse constraints next to Stage5 obstacles."""
    return not headless and not has_avoidance_plan


def enforce_collision_aware_hold_policy(
    has_obstacles: bool, measured_raw: Sequence[int],
    goal_raw: Sequence[int],
) -> None:
    """Fail *before* any blind straight joint-space recovery is queued.

    The initial C APF route is certified only from its original start pose;
    manual displacement changes its start and invalidates that certification.
    """
    if has_obstacles:
        raise Stage5UnvalidatedHoldRecoveryError(measured_raw, goal_raw)


def hold_replan_allowed(
    measured_quiet_frames: int,
    command_quiet_frames: int,
    attempts: int,
    frames_since_last_request: int,
    frames_since_last_gui_stop: int | None = None,
) -> bool:
    """A stalled HOLD needs BOTH motor command and physical position quiet.

    C trajectories may legitimately take >20s to unwind a full rotation at
    the configured max speed. Neither a 0.5s measured pause alone nor a
    large remaining goal error means that the C planner has stopped.
    """
    quiet_frames = math.ceil(HOLD_STAGNANT_PERIOD_S / FEEDBACK_STEP_S)
    cooldown_frames = math.ceil(HOLD_RETRY_COOLDOWN_S / FEEDBACK_STEP_S)
    return (
        0 <= attempts < HOLD_MAX_REPLAN_ATTEMPTS
        and measured_quiet_frames >= quiet_frames
        and command_quiet_frames >= quiet_frames
        and (attempts == 0 or frames_since_last_request >= cooldown_frames)
        # The user may still be holding the native mouse constraint against
        # a joint stop. Don't exhaust C retry attempts while still dragging.
        and (frames_since_last_gui_stop is None
             or frames_since_last_gui_stop >= quiet_frames)
    )


def assess_clock_alignment(trajectory_samples: int | None,
                           target_frames: int,
                           trajectory_period_s: float) -> tuple[float | None, bool]:
    """Compare reported RTOS planning duration against physical simulation.

    The C producer's trajectory_samples is the authoritative count of
    published planning samples. Both this count and PyBullet's consumed UART
    frames must describe comparable physical time, or joint arrival alone
    is not sufficient evidence of a real closed-loop trajectory.
    """
    if (trajectory_samples is None or trajectory_samples < 1 or
            target_frames < 1 or trajectory_period_s <= 0 or
            not math.isfinite(trajectory_period_s)):
        return None, False
    ratio = (trajectory_samples * trajectory_period_s) / (target_frames * FEEDBACK_STEP_S)
    return ratio, abs(ratio - 1.0) <= CLOCK_ALIGNMENT_MAX_RELATIVE_ERROR


def configure_physics_stepping(bullet, headless: bool) -> tuple[int, int]:
    """Return (API calls per feedback, warm-up calls).

    In GUI mode one client/server call advances a full feedback interval
    using Bullet's native fixed substeps. This keeps the 1 ms integrator and
    10 ms feedback interval but avoids 10 GUI thread round trips per UART
    target. DIRECT preserves the previously validated stepping behavior.
    """
    if headless:
        bullet.setTimeStep(PHYSICS_STEP_S)
        return SIMULATED_STEPS_PER_TARGET, INITIAL_SETTLE_INTERNAL_STEPS
    bullet.setTimeStep(FEEDBACK_STEP_S)
    bullet.setPhysicsEngineParameter(numSubSteps=SIMULATED_STEPS_PER_TARGET)
    if INITIAL_SETTLE_INTERNAL_STEPS % SIMULATED_STEPS_PER_TARGET:
        raise ValueError("Warm-up step count must divide native substep count")
    return 1, INITIAL_SETTLE_INTERNAL_STEPS // SIMULATED_STEPS_PER_TARGET


def configure_native_urdf_joint_stops(
    bullet, arm: int, joints: Sequence[int],
    travel_mode: str = "urdf",
) -> tuple[tuple[float, float], ...]:
    """Reinforce Bullet's own URDF revolute limit constraints for native picking.

    A strong native mouse constraint can drive a reduced-coordinate multibody
    slightly beyond the limit at finite solver force. Configure the *existing*
    native joint-limit constraint with the URDF's authoritative bounds and a
    larger simulated stop impulse. No extra ghost robot or mouse
    controller is created; the separate GUI hard-stop guard is required
    because native picking can still exceed finite impulse limits.
    """
    if len(joints) != wire.JOINT_COUNT:
        raise ValueError("URDF stop setup requires six actuated joints")
    if travel_mode not in GUI_JOINT_TRAVEL_MODES:
        raise ValueError("Unknown GUI joint travel policy")
    limits = []
    for name, index in zip(JOINT_NAMES, joints):
        info = bullet.getJointInfo(arm, index)
        low, high = float(info[8]), float(info[9])
        if info[2] != bullet.JOINT_REVOLUTE or not (
                math.isfinite(low) and math.isfinite(high) and low < high):
            raise ValueError(f"{name} lacks a finite revolute URDF travel range")
        effort = float(info[10])
        if not math.isfinite(effort) or effort <= 0:
            raise ValueError(f"{name} lacks positive URDF joint effort")
        if travel_mode == "single-turn":
            # The UART wire angle is canonical to a 360deg turn, but the
            # URDF mechanical model legitimately permits wider travel.
            # Limit only this *interactive GUI experiment* to +/-180deg.
            # Intersect with URDF, never extend an actual model stop.
            low, high = max(low, -math.pi), min(high, math.pi)
            if low >= high:
                raise ValueError(f"{name} has no single-turn GUI travel")
        bullet.changeDynamics(
            arm, index, jointLowerLimit=low, jointUpperLimit=high,
            jointLimitForce=effort * GUI_JOINT_STOP_FORCE_MULTIPLIER,
        )
        limits.append((low, high))
    return tuple(limits)


def joint_motor_efforts_from_urdf(
    bullet, arm: int, joints: Sequence[int],
) -> tuple[float, ...]:
    """Read the loaded URDF per-axis torque ratings for Bullet actuators.

    Joint stop constraint strength and joint motor effort are separate: a
    strong GUI picking hard stop must never inflate wrist actuator torque.
    """
    if len(joints) != wire.JOINT_COUNT:
        raise ValueError("URDF actuator lookup requires six joints")
    efforts = []
    for name, index in zip(JOINT_NAMES, joints):
        info = bullet.getJointInfo(arm, index)
        effort = float(info[10])
        if info[2] != bullet.JOINT_REVOLUTE or not (
                math.isfinite(effort) and effort > 0.0):
            raise ValueError(f"{name} lacks a positive revolute URDF effort")
        efforts.append(effort)
    return tuple(efforts)


def joint_motor_velocity_caps(
    bullet, arm: int, joints: Sequence[int],
    firmware_max_velocity_deg_s: Sequence[float],
) -> tuple[float, ...]:
    """Bound GUI actuator speed by both the firmware config and loaded URDF.

    Bullet does not automatically enforce the URDF's velocity field in
    POSITION_CONTROL. The physical mouse constraint can still exert external
    forces; this limits the commanded actuator, not arbitrary picking forces.
    """
    if len(joints) != wire.JOINT_COUNT or len(firmware_max_velocity_deg_s) != wire.JOINT_COUNT:
        raise ValueError("Motor velocity caps require six joints and six C limits")
    caps = []
    for name, joint, firmware_deg_s in zip(
            JOINT_NAMES, joints, firmware_max_velocity_deg_s):
        urdf_rad_s = float(bullet.getJointInfo(arm, joint)[11])
        if not (math.isfinite(urdf_rad_s) and urdf_rad_s > 0
                and math.isfinite(firmware_deg_s) and firmware_deg_s > 0):
            raise ValueError(f"{name} has an invalid URDF/firmware motor velocity limit")
        caps.append(min(urdf_rad_s, math.radians(firmware_deg_s)))
    return tuple(caps)


def set_joint_position_motors(
    bullet, arm: int, joints: Sequence[int],
    targets_rad: Sequence[float], forces_nm: Sequence[float],
    gui_velocity_caps_rad_s: Sequence[float] | None,
) -> None:
    """DIRECT keeps the proven batch call; GUI enforces real motor velocity caps.

    PyBullet setJointMotorControlArray does not accept maxVelocities on the
    installed wheels. setJointMotorControl2 exposes maxVelocity per joint.
    The six GUI calls preserve the original POSITION_CONTROL physics.
    """
    if not (len(joints) == len(targets_rad) == len(forces_nm) == wire.JOINT_COUNT):
        raise ValueError("Motor control requires six joint targets and forces")
    if gui_velocity_caps_rad_s is None:
        bullet.setJointMotorControlArray(
            arm, joints, bullet.POSITION_CONTROL,
            targetPositions=targets_rad, forces=forces_nm)
    else:
        if len(gui_velocity_caps_rad_s) != wire.JOINT_COUNT:
            raise ValueError("GUI motor control requires six velocity caps")
        for joint, target, force, cap in zip(
                joints, targets_rad, forces_nm, gui_velocity_caps_rad_s):
            bullet.setJointMotorControl2(
                arm, joint, bullet.POSITION_CONTROL,
                targetPosition=target, force=force, maxVelocity=cap)


def hold_continuous_goal_raw(
    canonical_goal: Sequence[int],
    measured: Sequence[int],
    joint_limits_rad: Sequence[tuple[float, float]],
) -> tuple[int, ...]:
    """Mirror the C motion_limits_unwrap_nearest choice for a HOLD replan.

    The canonical +30deg goal may legitimately become continuous -330deg
    when a joint was picked near -340deg. Comparing raw -330 against +30
    would produce a false 360deg recovery error.
    """
    positions_rad = closest_physical_motor_targets(
        canonical_goal, measured, joint_limits_rad)
    return tuple(round(math.degrees(value) * wire.JOINT_ANGLE_SCALE)
                 for value in positions_rad)


def enforce_gui_joint_hard_stops(
    bullet, arm: int, joints: Sequence[int],
    joint_limits_rad: Sequence[tuple[float, float]],
) -> list[dict]:
    """Keep the *actual* simulated GUI robot within URDF travel.

    Bullet's native picking constraint can overpower finite joint-limit
    impulses. After each GUI physics step, snap ONLY an out-of-range joint
    to its URDF hard stop with zero outward velocity. This is an explicit
    kinematic interaction guard, not a claim of physically simulated impact.
    It is never called in DIRECT/QEMU acceptance tests and never changes
    ordinary in-range states or firmware target commands.

    Returns the *pre-correction* physical angle and the applied stop so the
    non-physical correction is auditable instead of silently faked.
    """
    if len(joints) != wire.JOINT_COUNT or len(joint_limits_rad) != len(joints):
        raise ValueError("GUI joint-stop guard requires six URDF joint limits")
    states = bullet.getJointStates(arm, list(joints))
    if len(states) != len(joints):
        raise RuntimeError("PyBullet returned incomplete joint states")
    contacts = []
    for index, (joint, state, (low, high)) in enumerate(
            zip(joints, states, joint_limits_rad)):
        position = float(state[0])
        velocity = float(state[1])
        if not (math.isfinite(position) and math.isfinite(velocity)):
            raise JointPhysicalSafetyError(
                index, 0, "nonfinite_physical_joint_state",
                "PyBullet position or velocity is non-finite")
        if position < low or position > high:
            stop = low if position < low else high
            # Apply correction to the ONE authoritative physics body, not a
            # display mirror. Future dynamics and canonical UART feedback
            # continue from this bounded physical state.
            bullet.resetJointState(
                arm, joint, targetValue=stop, targetVelocity=0.0)
            contacts.append(dict(
                joint_index=index, joint=JOINT_NAMES[index],
                before_deg=math.degrees(position),
                stop_deg=math.degrees(stop),
                overshoot_deg=math.degrees(position - stop),
            ))
    return contacts


def advance_physics_step_batch(
    bullet, step_calls: int, start_time_s: float,
    on_physics: Callable | None, arm, joints, done: bool, ending: bool,
) -> None:
    """Apply each physical disturbance for every DIRECT 1 ms integration step.

    Bullet external torque/force applies to the next simulation step, not the
    whole following 10 ms feedback interval. GUI uses one native 10-substep
    call and therefore retains exactly one callback per UART frame.
    All time intervals derive from the existing FEEDBACK_STEP_S source.
    """
    if step_calls not in (1, SIMULATED_STEPS_PER_TARGET):
        raise ValueError("Unexpected physics steps per feedback interval")
    substep_s = FEEDBACK_STEP_S / step_calls
    for index in range(step_calls):
        if on_physics is not None:
            on_physics(
                bullet, arm, joints, start_time_s + index * substep_s,
                done, ending,
            )
        bullet.stepSimulation()



def safe_gui_jog_recovery_pending(jog) -> bool:
    """A fully preflighted jog is still pending even if a servo hides its error."""
    return (jog is not None and jog.applied > jog.completed
            and not jog.available)


def hold_recovery_requested(deviation_deg: float, gui_safe_jog: bool, jog) -> bool:
    """Keep the ordinary HOLD threshold, but honor an authorized jog event."""
    return (deviation_deg > HOLD_DEVIATION_TRIGGER_DEG
            or (gui_safe_jog and safe_gui_jog_recovery_pending(jog)))


def execute_guarded_gui_jog(
    jog, bullet, arm, joints, stamp, done, ending,
    motor_forces_nm, gui_motor_velocity_caps,
    *, ci_scripted_trigger: bool = False,
) -> None:
    """Preserve an authorized injected pose until the C UART recovery takes over.

    The original Bullet POSITION_CONTROL target is the final HOLD pose.
    Without this update a 0.5-degree jog can be erased within one physics
    frame, before HOLD deviation sampling and any C recovery request.
    """
    previous_applied = jog.applied
    jog(bullet, arm, joints, stamp, done, ending)
    if ci_scripted_trigger:
        if not done or ending:
            raise RuntimeError("Scripted CI jog outside idle HOLD")
        jog.request_ci_scripted_j6_half_degree()
    if jog.applied > previous_applied:
        displaced_rad = tuple(
            bullet.getJointState(arm, joint, physicsClientId=jog.client)[0]
            for joint in joints)
        set_joint_position_motors(
            bullet, arm, joints, displaced_rad,
            motor_forces_nm, gui_motor_velocity_caps)

class MotionSequencer:
    """按真实反馈到位情况依次发出关节请求，不覆盖未完成的请求。"""

    def __init__(self, goals: Sequence[Sequence[int]],
                 *, first_sequence: int = 41) -> None:
        if type(first_sequence) is not int or not 0 <= first_sequence <= 255:
            raise ValueError("Invalid motion UART sequence base")
        self.first_sequence = first_sequence
        self.goals = validate_goals(goals)
        self.index = -1
        self.sequence = -1
        self.ack_pending = False
        self.accepted = False
        self.settled_frames = 0
        self.completed = 0
        self.requested_at = 0.0
        self.segment_final_errors_deg: list[float] = []

    @property
    def done(self) -> bool:
        return self.completed == len(self.goals)

    @property
    def goal(self) -> tuple[int, ...] | None:
        return self.goals[self.index] if self.index >= 0 else None

    def next_request(self, now: float) -> bytes:
        """构造下一段请求；必须在前一段确认到位后调用。"""
        if self.ack_pending or self.accepted:
            raise RuntimeError("Previous motion has not settled")
        if self.index + 1 >= len(self.goals):
            raise RuntimeError("No remaining motion segment")
        self.index += 1
        self.sequence = (self.first_sequence + self.index) & 0xFF
        self.requested_at = now
        self.ack_pending = True
        self.settled_frames = 0
        payload = bytes((self.sequence,)) + struct.pack(
            "<" + str(wire.JOINT_COUNT) + "h", *self.goals[self.index]
        )
        return integration.frame(wire.CMD_REQUEST_JOINT_MOVE, payload)

    def acknowledge(self, payload: bytes) -> None:
        """ACK=0 仅表示入队，不代表机械臂已经运动到位。"""
        if not self.ack_pending or len(payload) != wire.MOVE_ACK_PAYLOAD_LEN:
            raise RuntimeError("Unexpected or malformed movement ACK")
        seq, status = struct.unpack("<Bb", payload)
        if seq != self.sequence:
            raise RuntimeError(f"Unexpected movement ACK sequence {seq}")
        if status != 0:
            raise RuntimeError(f"Firmware rejected segment {self.index + 1}: {status}")
        self.ack_pending = False
        self.accepted = True

    def observe(self, actual_raw: Sequence[int]) -> bool:
        """只有连续到位多帧后才放行下一段，避免轨迹请求互相覆盖。"""
        if not self.accepted:
            return False
        assert self.goal is not None
        maximum_error_deg = max(
            abs(measured - target) / wire.JOINT_ANGLE_SCALE
            for measured, target in zip(actual_raw, self.goal)
        )
        self.settled_frames = (
            self.settled_frames + 1
            if maximum_error_deg <= SETTLE_TOLERANCE_DEG else 0
        )
        if self.settled_frames < SETTLE_FRAME_COUNT:
            return False
        self.segment_final_errors_deg.append(maximum_error_deg)
        self.completed += 1
        self.accepted = False
        self.settled_frames = 0
        return True


def validate_goals(goals: Sequence[Sequence[int]]) -> tuple[tuple[int, ...], ...]:
    """Validate canonical UART request angles; C remains joint-limit authority."""
    if not goals:
        raise ValueError("Demonstration needs at least one motion segment")
    normalized = []
    for goal in goals:
        if len(goal) != wire.JOINT_COUNT:
            raise ValueError("Each target must contain exactly six angles")
        half_turn = round(math.degrees(math.pi) * wire.JOINT_ANGLE_SCALE)
        if any(type(value) is not int or
               not -half_turn <= value < half_turn for value in goal):
            raise ValueError("Each requested UART angle must be canonical [-180,180)deg")
        normalized.append(tuple(goal))
    return tuple(normalized)


def canonical_physical_feedback(raw: Sequence[int]) -> tuple[int, ...]:
    """Encode unbounded physical angles using the existing C canonical contract.

    Firmware robot_types.h defines the UART angle as [-180, 180) degrees;
    motor_driver.c reconstructs continuous turns from successive samples.
    The continuous PyBullet measurement remains untouched in traces/physics.
    """
    if len(raw) != wire.JOINT_COUNT:
        raise ValueError("Physical feedback requires exactly six angles")
    full_turn = round(math.degrees(math.tau) * wire.JOINT_ANGLE_SCALE)
    half_turn = full_turn // 2
    canonical = []
    for index, value in enumerate(raw):
        if type(value) is not int:
            raise TypeError(f"Physical joint {JOINT_NAMES[index]} must be an integer")
        canonical.append((value + half_turn) % full_turn - half_turn)
    return tuple(canonical)


def closest_physical_motor_targets(
    canonical: Sequence[int], physical: Sequence[int],
    joint_limits_rad: Sequence[tuple[float, float]],
) -> tuple[float, ...]:
    """Lift canonical firmware servo commands onto nearby URDF-legal turns.

    The firmware's UART motor target encodes only orientation, not its winding.
    The URDF limits determine allowable equivalent physical rotations; use
    measured continuous PyBullet angles for the closest equivalent turn.
    Never alter the physical measurement or silently clamp an illegal target.
    """
    if (len(canonical) != wire.JOINT_COUNT
            or len(physical) != wire.JOINT_COUNT
            or len(joint_limits_rad) != wire.JOINT_COUNT):
        raise ValueError("Target lifting requires six axes and six joint limits")
    full_turn = round(math.degrees(math.tau) * wire.JOINT_ANGLE_SCALE)
    lifted = []
    for i, (cmd, actual, (lower_rad, upper_rad)) in enumerate(
            zip(canonical, physical, joint_limits_rad)):
        if type(cmd) is not int or type(actual) is not int:
            raise TypeError(f"Joint {JOINT_NAMES[i]} angles must be integers")
        if not math.isfinite(lower_rad) or not math.isfinite(upper_rad):
            raise ValueError(f"Joint {JOINT_NAMES[i]} limits must be finite")
        if lower_rad > upper_rad:
            raise ValueError(f"Joint {JOINT_NAMES[i]} has invalid limits")
        lower = math.degrees(lower_rad) * wire.JOINT_ANGLE_SCALE
        upper = math.degrees(upper_rad) * wire.JOINT_ANGLE_SCALE
        minimum_turn = math.ceil((lower - cmd) / full_turn)
        maximum_turn = math.floor((upper - cmd) / full_turn)
        if minimum_turn > maximum_turn:
            raise ValueError(
                f"Canonical motor command {cmd} for {JOINT_NAMES[i]} "
                "has no equivalent pose within URDF limits")
        nearest_turn = math.floor((actual - cmd) / full_turn + 0.5)
        turn = min(max(nearest_turn, minimum_turn), maximum_turn)
        lifted.append(math.radians((cmd + turn * full_turn) /
                                   wire.JOINT_ANGLE_SCALE))
    return tuple(lifted)


def continuous_firmware_servo_targets(
    canonical: Sequence[int],
    physical_feedback: Sequence[int],
    previous_targets_rad: Sequence[float],
    joint_limits_rad: Sequence[tuple[float, float]],
    rebase: bool,
) -> tuple[float, ...]:
    """Decode the C actuator's canonical UART target without turn-jumping.

    Normal frames unwrap against the previous continuous servo command, NOT
    the dragged physical pose. A newly accepted C motion generation is the
    only place the controller can reset its limiter from measured feedback:
    it must rebase the first following servo target to that measured pose.
    """
    if len(previous_targets_rad) != wire.JOINT_COUNT:
        raise ValueError("Continuous servo decode requires six previous targets")
    previous_raw = tuple(
        round(math.degrees(value) * wire.JOINT_ANGLE_SCALE)
        for value in previous_targets_rad)
    return closest_physical_motor_targets(
        canonical, physical_feedback if rebase else previous_raw,
        joint_limits_rad)


class JointPhysicalSafetyError(ValueError):
    """A real measured motion violates URDF travel or aliases UART turn tracking."""

    def __init__(self, index: int, raw_centideg: int,
                 fault_type: str, explanation: str):
        self.joint_index = index
        self.joint_name = JOINT_NAMES[index]
        self.raw_centideg = raw_centideg
        self.angle_deg = raw_centideg / wire.JOINT_ANGLE_SCALE
        self.fault_type = fault_type
        super().__init__(
            f"{self.joint_name} at {self.angle_deg:.2f}deg: {explanation}"
        )


def validate_physical_feedback_step(
    current: Sequence[int], previous: Sequence[int],
    joint_limits_rad: Sequence[tuple[float, float]],
    numerical_slop_deg: float = 0.0,
) -> None:
    """Guard C driver's shortest-delta reconstruction and URDF joint travel.

    Canonical UART feedback is only unambiguous if no actual joint moves
    180deg or more between samples. Joint travel is read from loaded URDF,
    not a second independently maintained limit table.
    """
    if not 0.0 <= numerical_slop_deg <= GUI_JOINT_STOP_NUMERICAL_SLOP_DEG:
        raise ValueError("Invalid Bullet solver penetration tolerance")
    if (len(current) != wire.JOINT_COUNT
            or len(previous) != wire.JOINT_COUNT
            or len(joint_limits_rad) != wire.JOINT_COUNT):
        raise ValueError("Physical feedback validation requires six joints")
    half_turn_raw = round(math.degrees(math.pi) * wire.JOINT_ANGLE_SCALE)
    for index, (actual, last, (lower, upper)) in enumerate(
            zip(current, previous, joint_limits_rad)):
        if type(actual) is not int or type(last) is not int:
            raise TypeError("Physical feedback raw values must be integers")
        # One centidegree for rounding plus the documented GUI-only
        # solver penetration allowance. Never clamp real feedback.
        slop_raw = round(numerical_slop_deg * wire.JOINT_ANGLE_SCALE)
        low_raw = round(math.degrees(lower) * wire.JOINT_ANGLE_SCALE) - 1 - slop_raw
        high_raw = round(math.degrees(upper) * wire.JOINT_ANGLE_SCALE) + 1 + slop_raw
        if actual < low_raw or actual > high_raw:
            raise JointPhysicalSafetyError(
                index, actual, "urdf_joint_travel_exceeded",
                f"outside URDF travel [{math.degrees(lower):.2f}, "
                f"{math.degrees(upper):.2f}]deg. Physics stopped, not clamped",
            )
        if abs(actual - last) >= half_turn_raw:
            raise JointPhysicalSafetyError(
                index, actual, "ambiguous_multi_turn_feedback",
                f"{(actual - last) / wire.JOINT_ANGLE_SCALE:.2f}deg "
                "change in one UART physical sample cannot be unwrapped "
                "unambiguously by the existing C motor driver",
            )


class JointFeedbackRangeError(ValueError):
    """A caller bypassed canonical conversion before encoding UART feedback."""

    def __init__(self, index: int, raw_centideg: int):
        self.joint_index = index
        self.joint_name = JOINT_NAMES[index]
        self.raw_centideg = raw_centideg
        self.angle_deg = raw_centideg / wire.JOINT_ANGLE_SCALE
        self.fault_type = "noncanonical_uart_feedback"
        super().__init__(
            f"{self.joint_name} UART feedback {self.angle_deg:.2f}deg "
            f"(raw={raw_centideg}) violates the firmware canonical "
            "[-180, 180)deg contract; no clipping performed"
        )


def joint_feedback_frame(raw: Sequence[int]) -> bytes:
    """Encode already-canonical raw angles; physical readings must be normalized first."""
    if len(raw) != wire.JOINT_COUNT:
        raise ValueError("Joint-state feedback requires exactly six angles")
    half_turn = round(math.degrees(math.pi) * wire.JOINT_ANGLE_SCALE)
    for index, value in enumerate(raw):
        if type(value) is not int:
            raise TypeError(f"Joint {JOINT_NAMES[index]} feedback is not an integer")
        if not -half_turn <= value < half_turn:
            raise JointFeedbackRangeError(index, value)
    payload = struct.pack("<" + str(wire.JOINT_COUNT) + "h", *raw)
    return integration.frame(wire.CMD_JOINT_STATE, payload)


class Stage5RuntimeClearanceError(ValueError):
    """Actual link-to-obstacle clearance is smaller than the scene safety rule."""

    fault_type = "stage5_runtime_full_arm_clearance_violation"

    def __init__(self, link_index: int, obstacle_id: int, distance_m: float,
                 required_m: float, phase: str):
        self.link_index = link_index
        self.obstacle_id = obstacle_id
        self.distance_m = distance_m
        self.required_m = required_m
        self.phase = phase
        super().__init__(
            f"Stage5 full-arm clearance violation in {phase}: "
            f"link={link_index}, obstacle={obstacle_id}, "
            f"distance_m={distance_m:.6f}, required_m={required_m:.6f}")


def enforce_runtime_obstacle_clearance(
    bullet, arm: int, obstacles: Sequence[int],
    required_m: float, phase: str,
    measurement_radius_m: float | None = None,
) -> tuple[float | None, int | None]:
    """Fail closed on actual PyBullet link geometry in BOTH motion and HOLD.

    Positive distances below required_m are clearance faults, not contact.
    A drag-induced HOLD violation is still a fault, not a reason to quietly
    lower the C/scene clearance. Do not mutate physics or motor commands.
    """
    if not math.isfinite(required_m) or required_m < 0:
        raise ValueError("Invalid full-arm clearance")
    if phase not in ("planned_motion", "interactive_hold"):
        raise ValueError("Invalid obstacle clearance phase")
    if measurement_radius_m is not None and (
            not math.isfinite(measurement_radius_m) or
            measurement_radius_m < required_m):
        raise ValueError("Measurement radius cannot weaken the safety threshold")
    search_radius = (required_m if measurement_radius_m is None
                     else measurement_radius_m)
    closest = float("inf")
    closest_link = None
    bullet.performCollisionDetection()
    for link in range(-1, bullet.getNumJoints(arm)):
        for obstacle in obstacles:
            contacts = bullet.getClosestPoints(
                arm, obstacle, search_radius,
                linkIndexA=link, linkIndexB=-1)
            if contacts:
                distance = min(float(contact[8]) for contact in contacts)
                if distance < closest:
                    closest, closest_link = distance, link
                if distance < required_m:
                    raise Stage5RuntimeClearanceError(
                        link, obstacle, distance, required_m, phase)
    return (closest if math.isfinite(closest) else None), closest_link


def draw_avoidance_gui_evidence(bullet, preflight: dict) -> None:
    """Draw the actual C route vs direct EE shortcut in the native GUI.

    These are only visual overlays on the single authoritative Bullet scene.
    They do NOT replace C APF, geometry, UART or collision validation.
    """
    start = preflight["cartesian_start_m"]
    goal = preflight["cartesian_goal_m"]
    center = preflight["obstacle_center_m"]
    route = [start] + [
        [coord / 1000.0 for coord in waypoint]
        for waypoint in preflight["cartesian_route_mm"]
    ]
    bullet.addUserDebugLine(
        start, goal, [1.0, 0.15, 0.15], lineWidth=3.0, lifeTime=0)
    for before, after in zip(route, route[1:]):
        bullet.addUserDebugLine(
            before, after, [0.1, 1.0, 0.25], lineWidth=4.0, lifeTime=0)
    bullet.addUserDebugText(
        "RED: direct EE line (blocked) | GREEN: C APF route",
        [center[0], center[1], center[2] + 0.15],
        textColorRGB=[0.05, 0.9, 0.15], textSize=0.9, lifeTime=0)
    bullet.addUserDebugText(
        "PHYSICAL RED OBSTACLE",
        [center[0], center[1], center[2] + 0.07],
        textColorRGB=[1.0, 0.2, 0.1], textSize=1.1, lifeTime=0)
    print(
        "[APF SCENE] red visible collision box at "
        f"{tuple(round(c, 3) for c in center)}m; "
        f"direct_EE_blocked={preflight['direct_cartesian_segment_blocked']}; "
        f"green C path has {len(route) - 1} waypoints",
        flush=True)


def preserve_gui_after_physics_fault(
    bullet, qemu, stop_requested: Callable[[], bool] | None,
    error: ValueError,
) -> None:
    """Freeze the real GUI on an unsafe protocol state; let the user inspect and quit.

    Do not advance physics, send false UART feedback, or restart firmware.
    The original exception still propagates so the validation remains failed.
    """
    if qemu is not None and qemu.poll() is None:
        qemu.terminate()
    print(f"[SAFE STOP] {error}. Physics and QEMU stopped; press Q in the GUI to exit.",
          flush=True)
    try:
        bullet.addUserDebugText(
            "PHYSICAL FEEDBACK FAULT / SAFETY STOP / PHYSICS STOPPED / PRESS Q TO EXIT",
            [-0.7, 0.0, 0.85], textColorRGB=[1.0, 0.1, 0.1],
            textSize=1.35, lifeTime=0,
        )
    except bullet.error:
        # Keep the failure report even if the native renderer is disconnected.
        pass
    while bullet.isConnected():
        if stop_requested is not None and stop_requested():
            break
        try:
            if (bullet.getKeyboardEvents().get(ord("q"), 0)
                    & bullet.KEY_WAS_TRIGGERED):
                break
        except bullet.error:
            break
        time.sleep(0.05)


def write_trace(output_dir: Path, rows: list[dict]) -> None:
    """始终保留 CSV，方便在没有 matplotlib 的机器上离线审查。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "joint_trace.csv").open(
        "w", encoding="utf-8", newline=""
    ) as file:
        writer = csv.DictWriter(file, fieldnames=TRACE_HEADER)
        writer.writeheader()
        writer.writerows(rows)


def write_plot(output_dir: Path, rows: list[dict]) -> bool:
    """可选保存六轴跟踪曲线；缺少 matplotlib 不影响控制结果。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    if not rows:
        return False
    time_s = [row["simulated_time_s"] for row in rows]
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    for ax, name in zip(axes.flat, JOINT_NAMES):
        ax.plot(time_s, [r[f"servo_target_{name}_deg"] for r in rows],
                label="Physical actuator target (C UART + URDF turn)", linewidth=1.5)
        ax.plot(time_s, [r[f"actual_{name}_deg"] for r in rows],
                label="PyBullet measurement", linestyle="--", linewidth=1)
        ax.set_title(name)
        ax.set_ylabel("Angle (deg)")
        ax.grid(True, alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("Simulated physics time (s)")
    axes[0, 0].legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "joint_tracking.png", dpi=160)
    plt.close(fig)
    return True


def request_diagnostics(sock, name: str) -> None:
    """逐项查询固件权威计数器，响应帧本身不携带 selector。"""
    selector = wire.SIM_MOTION_DIAGNOSTIC_SELECTORS[name]
    sock.sendall(integration.frame(wire.CMD_GET_DIAGNOSTICS, bytes((selector,))))


def run(elf: Path, output_dir: Path, qemu_executable: str,
        options: DemoOptions,
        on_measurement: Callable[[float, int, tuple[int, ...]], None] | None = None,
        on_sample: Callable | None = None,
        stop_requested: Callable[[], bool] | None = None,
        on_physics: Callable | None = None,
        prepare_motion: Callable | None = None,
        ) -> dict:
    """启动真实 ARM 固件并与 PyBullet 双向交互，返回真实观测统计。"""
    import pybullet as p

    if not elf.is_file():
        raise FileNotFoundError(f"QEMU firmware ELF missing: {elf}")
    if not URDF.is_file():
        raise FileNotFoundError(f"UR5 model missing: {URDF}")
    if options.frame_delay_s < 0 or options.timeout_s <= 0:
        raise ValueError("Delay must be >= 0 and timeout must be > 0")
    if (not math.isfinite(options.diagnostics_timeout_s) or
            not DIAGNOSTICS_TIMEOUT_S <= options.diagnostics_timeout_s <= 60.0):
        raise ValueError("Diagnostics timeout must be between 5 and 60 seconds")
    if options.hold_observe_only and not options.interactive_hold:
        raise ValueError("Passive HOLD observation requires interactive HOLD")
    if options.gui_joint_travel not in GUI_JOINT_TRAVEL_MODES:
        raise ValueError("Unknown GUI joint travel policy")
    if options.test_obstacle_hold_recovery and (
            not options.headless or not options.interactive_hold or
            options.hold_observe_only or prepare_motion is None or
            on_physics is None or options.gui_safe_jog):
        raise ValueError(
            "C obstacle recovery experiment requires DIRECT, active HOLD, "
            "real C APF preflight and an explicitly supplied disturbance")
    if options.gui_safe_jog and (
            options.headless or not options.interactive_hold or
            options.hold_observe_only or prepare_motion is None or
            options.test_obstacle_hold_recovery):
        raise ValueError(
            "Guarded GUI jog requires native GUI, active HOLD and C APF; "
            "unrestricted native picking remains forbidden")
    if options.ci_scripted_safe_jog and not options.gui_safe_jog:
        raise ValueError("Scripted native GUI CI fixture requires guarded GUI jog")
    if not math.isclose(
        SIMULATED_STEPS_PER_TARGET * PHYSICS_STEP_S, FEEDBACK_STEP_S,
        rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError("Feedback period must be divisible by the physics step")

    sequencer = MotionSequencer(SCENARIOS[options.scenario])
    trace = deque(maxlen=6000) if options.interactive_hold else []
    diagnostic_values: dict[str, int] = {}
    names = tuple(wire.SIM_MOTION_DIAGNOSTIC_SELECTORS)
    parser = integration.FrameStream()
    firmware_defaults = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))
    trajectory_period_s = firmware_defaults["task_period_ms"]["trajectory"] / 1000.0
    monitor_period_s = firmware_defaults["task_period_ms"]["monitor"] / 1000.0
    safety_obstacles: tuple[int, ...] = ()
    safety_clearance_m = 0.0
    collision_scans = 0
    avoidance_report: dict | None = None
    recovery_plan_report: dict | None = None
    recovery_sequencer: MotionSequencer | None = None
    recovery_attempted = False
    recovery_segments_settled = 0
    recovery_sequence_next = None
    recovery_route_events = []
    safe_gui_jog = None
    firmware_max_velocity_deg_s = tuple(
        firmware_defaults["stage4_motion"]["max_velocity_deg_s"])
    step_calls_per_feedback = (SIMULATED_STEPS_PER_TARGET if options.headless else 1)
    feedback_acks = 0
    target_frames = 0
    bad_acks = 0
    diagnostics_index = 0
    diagnostics_pending: str | None = None
    diagnostics_pending_at = 0.0
    last_tool_xyz: tuple[float, float, float] | None = None
    debug_text_id = -1
    completed_successfully = False

    output_dir.mkdir(parents=True, exist_ok=True)
    # Remove stale verdicts before starting; a failed run must not inherit PASS.
    (output_dir / "result.json").unlink(missing_ok=True)
    audit = (output_dir / "uart_live.jsonl").open("w", encoding="utf-8", buffering=1)
    last_audit = 0.0
    last_uart_batch_warning = 0.0
    next_diagnostic = 0.0
    final_diagnostics = False
    hold_started = None
    settled_ee_xyz_m = None
    settled_ee_goal_error_mm = None
    hold_entry_clearance_m = None
    hold_entry_closest_link = None
    hold_entry_actual_joint_deg = None
    ending = False
    motor_target_rebase_pending = True
    hold_request_sequence = None
    hold_request_at = 0.0
    hold_request_goal_raw = None
    hold_goal_raw = tuple(sequencer.goals[-1])
    hold_rearms = 0
    hold_rearm_acks = 0
    hold_recoveries = 0
    gui_joint_stop_corrections = 0
    gui_joint_stop_last = None
    gui_joint_stop_last_log_at = float("-inf")
    last_gui_joint_stop_frame = -1_000_000
    hold_replan_needed = False
    recovering = False
    recovery_attempts = 0
    recovery_replan_sent = False
    recovery_quiet = 0
    recovery_settled = 0
    recovery_previous = None
    recovery_previous_motor_targets = None
    recovery_command_quiet = 0
    last_hold_replan_frame = 0
    recovery_stalled_reported = False
    last_measured = tuple(START_RAW)
    live_trace = (output_dir / "joint_trace.csv").open("w", encoding="utf-8", newline="")
    trace_writer = csv.DictWriter(live_trace, fieldnames=TRACE_HEADER)
    trace_writer.writeheader()
    live_trace.flush()
    qemu_error = (output_dir / "qemu_stderr.log").open("w", encoding="utf-8")
    port = integration.vacant_tcp_port()
    qmp_port = integration.vacant_tcp_port()
    while qmp_port == port:
        qmp_port = integration.vacant_tcp_port()
    native_picking_allowed = apf_hold_mouse_picking_allowed(
        options.headless, prepare_motion is not None)
    metadata = dict(elf=str(elf.resolve()),
        elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest(),
        config_sha256=hashlib.sha256(DEFAULT_JSON.read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
        python=sys.version, platform=sys.platform, qemu_executable=qemu_executable,
        pybullet_api_version=p.getAPIVersion(),
        physics_mode="DIRECT" if options.headless else "GUI",
        native_gui_picking=native_picking_allowed,
        avoidance_hold_recovery_policy=(
            "GUI_guarded_button_jog_requires_full_link_preview_then_C_route"
            if options.gui_safe_jog
            else "DIRECT_controlled_disturbance_requires_validated_C_route"
            if options.test_obstacle_hold_recovery
            else "refuse_unvalidated_return_and_disable_manual_picking"
            if prepare_motion is not None else "original_unobstructed_hold"),
        gui_safe_jog=options.gui_safe_jog,
        ci_scripted_safe_jog=options.ci_scripted_safe_jog,
        gui_joint_travel=(options.gui_joint_travel if not options.headless
                          else "urdf (DIRECT reference)"),
        qemu_clock="adaptive_icount" if sys.platform == "win32" else "wall",
        physics_clock_gate="QMP stop/cont per UART target batch",
        feedback_step_s=FEEDBACK_STEP_S, physics_step_s=PHYSICS_STEP_S,
        pybullet_step_calls_per_feedback=step_calls_per_feedback,
        internal_substeps_per_feedback=SIMULATED_STEPS_PER_TARGET,
        note="config hash is current source; use preset build for matching ELF")
    (output_dir / "run_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    qemu = None
    client = -1
    try:
        qemu = subprocess.Popen(
            [qemu_executable, "-M", "mps2-an386", "-kernel", str(elf.resolve()),
             "-display", "none", "-monitor", "none",
             "-qmp", f"tcp:127.0.0.1:{qmp_port},server=on,wait=off",
             # Windows wall-clock timer bursts can advance RTOS control without
             # corresponding Python feedback. Pace virtual time adaptively; this
             # remains live ARM execution with real UART feedback, not trace replay.
             *(["-icount", "shift=auto,sleep=on"] if sys.platform == "win32" else []),
             "-serial", f"tcp:127.0.0.1:{port},server,nowait"],
            stdout=subprocess.DEVNULL,
            stderr=qemu_error,
        )
        with (QemuClockGate.connect(qmp_port, qemu) as clock_gate,
              integration.join_qemu_serial(port, qemu, stop_requested) as sock):
            # One authoritative physics world. GUI owns both native picking
            # and simulation; DIRECT runs the identical physics path in CI.
            client = (p.connect(p.DIRECT) if options.headless else
                      p.connect(p.GUI, options="--opengl2" if options.opengl2 else ""))
            if client < 0:
                raise RuntimeError("PyBullet connection failed")
            p.setRealTimeSimulation(0)
            if not options.headless:
                p.configureDebugVisualizer(
                    p.COV_ENABLE_MOUSE_PICKING,
                    1 if native_picking_allowed else 0)
                if not native_picking_allowed:
                    print(
                        "[APF HOLD SAFETY] Native mouse dragging DISABLED. "
                        + ("Guarded GUI jog buttons use a separate full-arm "
                           "preview before any pose change."
                           if options.gui_safe_jog else
                           "Unvalidated manual recovery replans forbidden."),
                        flush=True)
                # Keep native debug controls visible. Hiding them made the
                # sole physical GUI look like an empty rendering viewport.
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.setGravity(0, 0, -9.81)
            step_calls_per_feedback, warmup_calls = configure_physics_stepping(
                p, options.headless)
            if not options.headless:
                p.setPhysicsEngineParameter(
                    numSolverIterations=GUI_JOINT_STOP_SOLVER_ITERATIONS)
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            motor_forces_nm = joint_motor_efforts_from_urdf(p, arm, joints)
            motor_velocity_caps_rad_s = joint_motor_velocity_caps(
                p, arm, joints, firmware_max_velocity_deg_s)
            # Only GUI mouse-picking needs an additional motor-side speed
            # safeguard. Keep the established DIRECT acceptance path intact.
            gui_motor_velocity_caps = (
                motor_velocity_caps_rad_s if not options.headless else None)
            if not options.headless:
                joint_limits_rad = configure_native_urdf_joint_stops(
                    p, arm, joints, travel_mode=options.gui_joint_travel)
                print(
                    "[GUI JOINT LIMITS] " + options.gui_joint_travel + ": "
                    + ", ".join(
                        f"J{i + 1} [{math.degrees(low):.1f}, {math.degrees(high):.1f}]deg"
                        for i, (low, high) in enumerate(joint_limits_rad)),
                    flush=True)
            else:
                joint_limits_rad = tuple(
                    (p.getJointInfo(arm, index)[8], p.getJointInfo(arm, index)[9])
                    for index in joints
                )
            for idx, raw in zip(joints, START_RAW):
                p.resetJointState(arm, idx, math.radians(raw / wire.JOINT_ANGLE_SCALE))
            physical_motor_targets_rad = tuple(
                math.radians(v / wire.JOINT_ANGLE_SCALE) for v in START_RAW
            )
            set_joint_position_motors(
                p, arm, joints, physical_motor_targets_rad, motor_forces_nm,
                gui_motor_velocity_caps)
            for _ in range(warmup_calls):
                p.stepSimulation()
            if prepare_motion is not None:
                # Optional Stage5 route preflight only. The validated GUI/HOLD
                # motor and angular unwrapping path below stays unchanged.
                plan = prepare_motion(p, arm, joints, client)
                targets = [tuple(
                    int(round(math.degrees(q) * wire.JOINT_ANGLE_SCALE))
                    for q in pose)
                    for pose in plan["joint_waypoints_rad"][1:]]
                sequencer = MotionSequencer(targets)
                hold_goal_raw = tuple(sequencer.goals[-1])
                safety_obstacles = tuple(plan["obstacle_ids"])
                safety_clearance_m = float(plan["static_collision_clearance_m"])
                avoidance_report = {
                    key: value for key, value in plan.items() if key != "obstacle_ids"
                }
                (output_dir / "avoidance_preflight.json").write_text(
                    json.dumps(avoidance_report, indent=2), encoding="utf-8")
                if not options.headless:
                    draw_avoidance_gui_evidence(p, avoidance_report)
                if options.gui_safe_jog:
                    from stage5_avoidance_uart_gate import load_scene
                    from stage5_safe_gui_jog import SafeGuiJog, JOG_EXTRA_CLEARANCE_M
                    scene, collision_policy = load_scene(plan["route_case"])
                    safe_gui_jog = SafeGuiJog(
                        p, arm, joints, safety_obstacles, client,
                        joint_limits_rad,
                        maximum_joint_step_rad=collision_policy[
                            "max_joint_step_rad"],
                        minimum_clearance_m=safety_clearance_m,
                        maximum_samples=collision_policy["max_collision_samples"],
                        preview_return=lambda: prepare_motion.plan_hold_recovery(
                            p, arm, joints, client, plan,
                            minimum_extra_clearance_m=JOG_EXTRA_CLEARANCE_M))
                    previous_on_physics = on_physics
                    scripted_ci_jog_sent = False
                    scripted_ci_finish_sent = False

                    def guarded_jog_callback(bullet, body, axes, stamp, done, ending):
                        nonlocal scripted_ci_jog_sent, scripted_ci_finish_sent
                        if previous_on_physics is not None:
                            previous_on_physics(
                                bullet, body, axes, stamp, done, ending)
                        ci_inject = bool(
                            options.ci_scripted_safe_jog
                            and done and not ending
                            and not scripted_ci_jog_sent)
                        # Latch before calling the planner, fail closed if rejected.
                        if ci_inject:
                            scripted_ci_jog_sent = True
                        execute_guarded_gui_jog(
                            safe_gui_jog, bullet, body, axes, stamp, done, ending,
                            motor_forces_nm, gui_motor_velocity_caps,
                            ci_scripted_trigger=ci_inject)
                        if (options.ci_scripted_safe_jog
                                and scripted_ci_jog_sent
                                and not scripted_ci_finish_sent
                                and safe_gui_jog.applied == 1
                                and safe_gui_jog.completed == 1):
                            safe_gui_jog.request_ci_finish()
                            scripted_ci_finish_sent = True
                    on_physics = guarded_jog_callback
                    print(
                        "[SAFE GUI JOG] C APF obstacle scene: use left panel "
                        "SAFE APF JOG +/- buttons and J1..J6 selector. "
                        "Use during HOLD; 0.5..2.0deg increments, up to 3 "
                        "complete returns per run. Outbound AND return routes "
                        "are checked BEFORE application. "
                        "Original mouse dragging remains OFF; "
                        "FINISH/EXIT validates or cancels; console Q can exit.",
                        flush=True)
            if not options.headless:
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.05, cameraYaw=50, cameraPitch=-26,
                    cameraTargetPosition=[0, 0, 0.35],
                )

            # 等待固件 RX task 就绪。未就绪时提前写入的首帧可能丢失，
            # 而 ProtocolTX 又会等待有效反馈，造成双方永久等待。
            sock.sendall(integration.frame(wire.CMD_GET_DIAGNOSTICS, bytes((0,))))
            ready_deadline = time.monotonic() + 5.0
            ready = False
            while time.monotonic() < ready_deadline:
                if stop_requested is not None and stop_requested():
                    raise InterruptedError("UART readiness cancelled by observer")
                try:
                    ready_bytes = sock.recv(4096)
                except BlockingIOError:
                    ready_bytes = None
                if ready_bytes:
                    ready = any(
                        command == wire.CMD_DIAGNOSTICS_RESPONSE and len(payload) == 4
                        for command, payload in parser.consume(ready_bytes)
                    )
                    if ready:
                        break
                elif ready_bytes == b"":
                    raise RuntimeError("QEMU UART disconnected during readiness probe")
                else:
                    select.select([sock], [], [], 0.002)
            if not ready:
                raise TimeoutError("FreeRTOS ProtocolRX did not become ready")

            # 首个反馈主动发送：ProtocolTX 在未收到反馈前不会发布目标。
            sock.sendall(joint_feedback_frame(
                canonical_physical_feedback(angles_raw(p, arm, joints))))
            start_wall = time.monotonic()
            last_frame_wall = start_wall
            while True:
                now = time.monotonic()
                if not options.headless and not p.isConnected(client):
                    raise Stage5GuiDisconnectedError(
                        "PyBullet GUI window disconnected. For a measured HOLD "
                        "verdict keep the window open and press Q to exit; "
                        "closing X before the final sample is not a PASS.")
                if sequencer.done and hold_started is None:
                    hold_started = now
                    if avoidance_report is not None:
                        settled_ee_xyz_m = list(p.getLinkState(
                            arm, p.getNumJoints(arm) - 1,
                            computeForwardKinematics=True)[4])
                        expected_ee = avoidance_report["cartesian_goal_m"]
                        settled_ee_goal_error_mm = (
                            1000.0 * math.dist(settled_ee_xyz_m, expected_ee))
                        print(
                            "[APF PHYSICAL EE] actual_m="
                            f"{[round(v, 4) for v in settled_ee_xyz_m]}, "
                            "goal_m="
                            f"{[round(v, 4) for v in expected_ee]}, "
                            f"error_mm={settled_ee_goal_error_mm:.3f}",
                            flush=True)
                        hold_entry_actual_joint_deg = [
                            value / wire.JOINT_ANGLE_SCALE
                            for value in last_measured]
                        (hold_entry_clearance_m, hold_entry_closest_link) = (
                            enforce_runtime_obstacle_clearance(
                                p, arm, safety_obstacles, safety_clearance_m,
                                "interactive_hold", measurement_radius_m=0.15))
                        print(
                            "[APF HOLD GEOMETRY] planned_route_min_m="
                            f"{avoidance_report.get('minimum_planned_full_link_clearance_m')}; "
                            f"hold_entry_min_m={hold_entry_clearance_m}, "
                            f"closest_link={hold_entry_closest_link}; "
                            f"actual_joint_deg={hold_entry_actual_joint_deg}",
                            flush=True)
                        audit.write(json.dumps(dict(
                            event="apf_physical_ee_at_hold_entry",
                            actual_ee_xyz_m=settled_ee_xyz_m,
                            goal_ee_xyz_m=expected_ee,
                            physical_ee_goal_error_mm=settled_ee_goal_error_mm,
                            minimum_planned_full_link_clearance_m=avoidance_report.get(
                                "minimum_planned_full_link_clearance_m"),
                            hold_entry_minimum_clearance_m=hold_entry_clearance_m,
                            hold_entry_closest_link=hold_entry_closest_link,
                            hold_entry_actual_joint_deg=hold_entry_actual_joint_deg,
                            segments_settled=sequencer.completed)) + "\n")
                    if options.interactive_hold:
                        if native_picking_allowed:
                            print(
                                "[HOLD] Native mouse picking active; "
                                "holding the final goal", flush=True)
                        elif options.gui_safe_jog:
                            print(
                                "[HOLD] SAFE GUI JOG buttons armed; native mouse "
                                "dragging OFF. Click one +/-2deg button. "
                                "Guarded C APF/IK recovery follows. "
                        "Press SAFE JOG FINISH/EXIT to validate and exit after "
                        "recovery, or cancel safely without a jog. "
                        "Do not close the window with X.", flush=True)
                        else:
                            print(
                                "[HOLD] Static obstacle scene: mouse picking "
                                "disabled; unvalidated recovery rearm forbidden.",
                                flush=True)
                if (safe_gui_jog is not None and safe_gui_jog.cancel_requested
                        and not ending):
                    if (not sequencer.done or recovering or
                            recovery_sequencer is not None or
                            safe_gui_jog.applied != 0 or
                            safe_gui_jog.completed != 0):
                        raise RuntimeError(
                            "SAFE GUI JOG cancel attempted during an active recovery")
                    ending = True
                    audit.write(json.dumps(dict(
                        event="safe_gui_jog_cancel_accepted",
                        reason="no_guarded_jog_requested",
                        target_frames=target_frames)) + "\n")
                    print("[SAFE GUI JOG CANCEL] No jog attempted; "
                          "collecting final UART diagnostics. NOT PASS.",
                          flush=True)
                if (safe_gui_jog is not None and safe_gui_jog.finish_requested
                        and not ending):
                    if (not sequencer.done or recovering or
                            recovery_sequencer is not None or
                            safe_gui_jog.applied < 1 or
                            safe_gui_jog.completed != safe_gui_jog.applied or
                            hold_recoveries != safe_gui_jog.completed):
                        raise RuntimeError(
                            "SAFE GUI JOG finish attempted before validated C recovery")
                    ending = True
                    audit.write(json.dumps(dict(
                        event="safe_gui_jog_finish_accepted",
                        completed_recoveries=safe_gui_jog.completed,
                        target_frames=target_frames)) + "\n")
                    print("[SAFE GUI JOG FINISH] All requested C recoveries "
                          "settled; collecting final UART diagnostics", flush=True)
                if stop_requested is not None and stop_requested():
                    if not sequencer.done or not options.interactive_hold:
                        raise InterruptedError("Observer requested live control shutdown")
                    if (safe_gui_jog is not None and safe_gui_jog.applied == 0
                            and not safe_gui_jog.cancel_requested):
                        safe_gui_jog.cancel_requested = True
                        audit.write(json.dumps(dict(
                            event="safe_gui_jog_cancel_accepted",
                            source="external_stop",
                            reason="no_guarded_jog_requested",
                            target_frames=target_frames)) + "\n")
                    ending = True
                if options.interactive_hold and hold_started is not None:
                    if (options.hold_duration_s is not None and
                            now-hold_started >= options.hold_duration_s):
                        if safe_gui_jog is not None and safe_gui_jog.applied == 0:
                            if not safe_gui_jog.cancel_requested:
                                safe_gui_jog.cancel_requested = True
                                audit.write(json.dumps(dict(
                                    event="safe_gui_jog_cancel_accepted",
                                    source="hold_duration_limit",
                                    reason="no_guarded_jog_requested",
                                    target_frames=target_frames)) + "\n")
                            ending = True
                        elif (safe_gui_jog is not None and
                              safe_gui_jog.applied > safe_gui_jog.completed):
                            # Let an in-flight validated C recovery finish.
                            # The independent observer has a finite grace period.
                            pass
                        else:
                            ending = True
                elif now-start_wall >= options.timeout_s:
                    raise TimeoutError(f"Demo timed out; completed {sequencer.completed}/{len(sequencer.goals)} segments")
                if hold_request_sequence is not None and now-hold_request_at > 8.0:
                    raise TimeoutError("Hold recovery movement ACK timeout")
                if (recovery_sequencer is not None and
                        recovery_sequencer.ack_pending and
                        now - recovery_sequencer.requested_at > 8.0):
                    raise TimeoutError("Validated C recovery segment ACK timeout")
                if qemu.poll() is not None:
                    raise RuntimeError(f"QEMU terminated: {qemu.returncode}")
                if time.monotonic() - last_frame_wall > UART_SILENCE_TIMEOUT_S:
                    raise TimeoutError(
                        "No UART frames from QEMU for 8 seconds; "
                        f"targets={target_frames}, feedback_acks={feedback_acks}, "
                        f"accepted={sequencer.accepted}, "
                        f"completed={sequencer.completed}/{len(sequencer.goals)}. "
                        "See joint_trace.csv for the last measured motion."
                    )
                if sequencer.ack_pending and (
                    time.monotonic() - sequencer.requested_at > 8.0
                ):
                    raise TimeoutError("Movement request ACK timeout")
                if diagnostics_pending and (
                    time.monotonic() - diagnostics_pending_at > options.diagnostics_timeout_s
                ):
                    audit.write(json.dumps(dict(
                        event="diagnostics_timeout", selector=diagnostics_pending,
                        last_diagnostics=diagnostic_values, target_frames=target_frames,
                        feedback_acks=feedback_acks,
                        feedback_ack_lag=target_frames-feedback_acks,
                        bad_checksum=parser.bad_checksum,
                        elapsed_s=time.monotonic() - diagnostics_pending_at,
                    )) + "\n")
                    raise TimeoutError(f"Diagnostics timeout: {diagnostics_pending}; "
                                       f"UART targets={target_frames}, ACK={feedback_acks}")

                try:
                    data = sock.recv(8192)
                except BlockingIOError:
                    data = None
                if data == b"":
                    raise RuntimeError("QEMU UART disconnected")
                if data:
                    last_frame_wall = time.monotonic()
                    batch_started = last_frame_wall
                    parsed_frames = parser.consume(data)
                    # Stop guest RTOS time for the *entire* physical batch.
                    # It cannot schedule another 10 ms Control/TX period while
                    # PyBullet spends wall time integrating or rendering this one.
                    with clock_gate.paused_for_physics(any(
                            cmd == wire.CMD_SET_JOINT_TARGETS
                            for cmd, _ in parsed_frames)):
                        for command, payload in parsed_frames:
                            if (command == wire.CMD_SET_JOINT_TARGETS and
                                    len(payload) == wire.JOINT_PAYLOAD_LEN):
                                targets = struct.unpack(
                                    "<" + str(wire.JOINT_COUNT) + "h", payload)
                                # 仅在运动请求已被接受后执行固件位置目标；启动
                                # 时始终保持首次实测姿态，不能执行历史演示角度。
                                if (sequencer.accepted or
                                        (sequencer.done and options.interactive_hold
                                         and (recovery_sequencer is None or
                                              recovery_sequencer.accepted))):
                                    physical_motor_targets_rad = continuous_firmware_servo_targets(
                                        targets, last_measured,
                                        physical_motor_targets_rad, joint_limits_rad,
                                        rebase=motor_target_rebase_pending)
                                    motor_target_rebase_pending = False
                                    set_joint_position_motors(
                                        p, arm, joints, physical_motor_targets_rad,
                                        motor_forces_nm, gui_motor_velocity_caps)
                                # A DIRECT frame is ten 1ms physics steps.
                                # Force/torque fixtures must act at EACH step,
                                # not only the first; otherwise motor servoing
                                # conceals the applied disturbance.
                                advance_physics_step_batch(
                                    p, step_calls_per_feedback,
                                    target_frames * FEEDBACK_STEP_S,
                                    on_physics, arm, joints,
                                    sequencer.done, ending,
                                )
                                target_frames += 1
                                if not options.headless:
                                    joint_stop_contacts = enforce_gui_joint_hard_stops(
                                        p, arm, joints, joint_limits_rad)
                                    if joint_stop_contacts:
                                        last_gui_joint_stop_frame = target_frames
                                        gui_joint_stop_corrections += len(joint_stop_contacts)
                                        gui_joint_stop_last = joint_stop_contacts[-1]
                                        # Keep a full event on first encounter,
                                        # then rate-limit during sustained picking.
                                        if (time.monotonic()
                                                - gui_joint_stop_last_log_at >= 1.0):
                                            audit.write(json.dumps(dict(
                                                event="gui_urdf_hard_stop",
                                                contacts=joint_stop_contacts,
                                                corrections_total=gui_joint_stop_corrections,
                                                target_frames=target_frames,
                                            )) + "\n")
                                            gui_joint_stop_last_log_at = time.monotonic()
                                            print(
                                                "[JOINT STOP] "
                                                + ", ".join(
                                                    f"{c['joint']}: "
                                                    f"{c['before_deg']:.2f}deg "
                                                    f"-> {c['stop_deg']:.2f}deg"
                                                    for c in joint_stop_contacts
                                                ), flush=True)
                                measured = angles_raw(p, arm, joints)
                                validate_physical_feedback_step(
                                    measured, last_measured, joint_limits_rad,
                                    numerical_slop_deg=(
                                        GUI_JOINT_STOP_NUMERICAL_SLOP_DEG
                                        if not options.headless else 0.0))
                                last_measured = tuple(measured)
                                if safety_obstacles:
                                    # Motion and manual HOLD use the SAME strict
                                    # scene clearance. A fault stops QEMU and
                                    # freezes the GUI instead of claiming PASS.
                                    enforce_runtime_obstacle_clearance(
                                        p, arm, safety_obstacles,
                                        safety_clearance_m,
                                        "interactive_hold" if sequencer.done
                                        else "planned_motion")
                                    collision_scans += 1
                                sock.sendall(joint_feedback_frame(
                                    canonical_physical_feedback(measured)))
                                # Detect a real joint-space deviation, irrespective
                                # of whether it came from mouse picking or test
                                # forces. Never interpret UART motor commands as
                                # the final requested joint goal.
                                if sequencer.done and options.interactive_hold and not ending:
                                    deviation = max(abs(a-b)/wire.JOINT_ANGLE_SCALE
                                                    for a,b in zip(measured, hold_goal_raw))
                                    if (hold_recovery_requested(
                                            deviation, options.gui_safe_jog,
                                            safe_gui_jog) and not recovering):
                                        if safety_obstacles:
                                            # The arbitrary GUI picking case remains
                                            # blocked. Only a controlled DIRECT test
                                            # can authorize a new complete C route.
                                            if not (options.test_obstacle_hold_recovery
                                                    or options.gui_safe_jog):
                                                enforce_collision_aware_hold_policy(
                                                    True, measured, hold_goal_raw)
                                            if (options.gui_safe_jog and
                                                    not safe_gui_jog_recovery_pending(
                                                        safe_gui_jog)):
                                                # Guarded mode cannot be used as a
                                                # backdoor to replan arbitrary
                                                # unvalidated external motion.
                                                enforce_collision_aware_hold_policy(
                                                    True, measured, hold_goal_raw)
                                            if recovery_attempted:
                                                enforce_collision_aware_hold_policy(
                                                    True, measured, hold_goal_raw)
                                            recovery_attempted = True
                                            # Hold the displaced pose while C APF/
                                            # IK and all-link gates inspect this same
                                            # physical world with QMP clock paused.
                                            current_position_rad = tuple(
                                                math.radians(v / wire.JOINT_ANGLE_SCALE)
                                                for v in measured)
                                            set_joint_position_motors(
                                                p, arm, joints,
                                                current_position_rad,
                                                motor_forces_nm,
                                                gui_motor_velocity_caps)
                                            if not hasattr(prepare_motion, "plan_hold_recovery"):
                                                raise RuntimeError("C APF recovery planner unavailable")
                                            print(
                                                "[HOLD C APF PHASE] runtime_recovery_planning_started",
                                                flush=True)
                                            audit.write(json.dumps(dict(
                                                event="hold_C_apf_recovery_planning_started",
                                                target_frames=target_frames,
                                                measured_raw=list(measured))) + "\n")
                                            audit.flush()
                                            recovery_plan_started = time.monotonic()
                                            recovery_plan_report = (
                                                prepare_motion.plan_hold_recovery(
                                                    p, arm, joints, client, plan,
                                                    minimum_extra_clearance_m=(
                                                        safe_gui_jog.extra_clearance_m
                                                        if options.gui_safe_jog else 0.0)))
                                            print(
                                                "[HOLD C APF PHASE] runtime_recovery_planning_finished "
                                                f"wall_s={time.monotonic()-recovery_plan_started:.3f}",
                                                flush=True)
                                            recovery_goals = [
                                                tuple(round(math.degrees(v) *
                                                            wire.JOINT_ANGLE_SCALE)
                                                      for v in waypoint)
                                                for waypoint in recovery_plan_report[
                                                    "joint_waypoints_rad"][1:]]
                                            if recovery_sequence_next is None:
                                                recovery_sequence_next = (
                                                    sequencer.sequence + 1) & 0xff
                                            recovery_sequencer = MotionSequencer(
                                                recovery_goals,
                                                first_sequence=recovery_sequence_next)
                                            recovery_route_events.append(dict(
                                                first_sequence=recovery_sequence_next,
                                                planned_segments=len(recovery_goals),
                                                completed=False,
                                                minimum_planned_clearance_m=
                                                    recovery_plan_report[
                                                        "minimum_measured_full_link_clearance_m"]))
                                            recovery_attempts += 1
                                            audit.write(json.dumps(dict(
                                                event="hold_C_apf_recovery_preflight",
                                                passed=True,
                                                from_measured_raw=list(measured),
                                                planned_segments=len(recovery_goals),
                                                min_full_link_clearance_m=
                                                    recovery_plan_report[
                                                        "minimum_measured_full_link_clearance_m"],
                                            )) + "\n")
                                            sock.sendall(recovery_sequencer.next_request(
                                                time.monotonic()))
                                            print(
                                                "[HOLD C APF] Accepted new measured-pose "
                                                f"preflight: {len(recovery_goals)} "
                                                "collision-checked segments",
                                                flush=True)
                                        recovery_stalled_reported = False
                                        recovering = True
                                        recovery_attempts = 0
                                        recovery_replan_sent = False
                                        recovery_quiet = 0
                                        recovery_command_quiet = 0
                                        recovery_settled = 0
                                        recovery_previous = None
                                        recovery_previous_motor_targets = None
                                        audit.write(json.dumps(dict(event="hold_deviation",
                                            error_deg=deviation)) + "\n")
                                if recovering and recovery_sequencer is None and not ending:
                                    error = max(abs(a-b)/wire.JOINT_ANGLE_SCALE for a,b in zip(measured, hold_goal_raw))
                                    recovery_settled = recovery_settled+1 if error <= SETTLE_TOLERANCE_DEG else 0
                                    quiet = recovery_previous is not None and max(abs(a-b) for a,b in zip(measured, recovery_previous)) <= 1
                                    recovery_quiet = recovery_quiet+1 if quiet else 0
                                    # C motor targets are the authoritative
                                    # signal for whether planning still progresses.
                                    # A slow/temporarily stationary *arm* is not
                                    # evidence that the C planner has stopped.
                                    motor_quiet = (
                                        recovery_previous_motor_targets is not None
                                        and all(
                                            abs(current - prior) <= 1e-10
                                            for current, prior in zip(
                                                physical_motor_targets_rad,
                                                recovery_previous_motor_targets)
                                        )
                                    )
                                    recovery_command_quiet = (
                                        recovery_command_quiet + 1 if motor_quiet else 0)
                                    recovery_previous = tuple(measured)
                                    recovery_previous_motor_targets = tuple(
                                        physical_motor_targets_rad)
                                    if recovery_settled >= SETTLE_FRAME_COUNT:
                                        recovering = False
                                        recovery_replan_sent = False
                                        hold_recoveries += 1
                                        audit.write(json.dumps(dict(event="hold_recovered", error_deg=error,
                                            attempts=recovery_attempts)) + "\n")
                                    elif (recovery_quiet >= math.ceil(
                                            HOLD_STAGNANT_PERIOD_S / FEEDBACK_STEP_S)
                                          and recovery_command_quiet >= math.ceil(
                                            HOLD_STAGNANT_PERIOD_S / FEEDBACK_STEP_S)
                                          and hold_request_sequence is None):
                                        # Both the real motor command and the
                                        # measured arm have stopped away from
                                        # the chosen C goal. This differs from
                                        # legitimate slow unwinding or following.
                                        if options.hold_observe_only:
                                            if not recovery_stalled_reported:
                                                audit.write(json.dumps(dict(
                                                    event="hold_observe_stalled",
                                                    error_deg=error,
                                                    diagnostics=diagnostic_values.copy(),
                                                )) + "\n")
                                                recovery_stalled_reported = True
                                        elif hold_replan_allowed(
                                                recovery_quiet, recovery_command_quiet,
                                                recovery_attempts,
                                                target_frames - last_hold_replan_frame,
                                                target_frames - last_gui_joint_stop_frame):
                                            # Re-submit to the existing C motion
                                            # queue. ACK is queue admission, not
                                            # trajectory success. Retry only on
                                            # a confirmed stopped actuator target,
                                            # with cooldown and bounded count.
                                            hold_replan_needed = True
                                        elif (recovery_attempts >= HOLD_MAX_REPLAN_ATTEMPTS
                                              and not recovery_stalled_reported):
                                            audit.write(json.dumps(dict(
                                                event="hold_recovery_stalled",
                                                error_deg=error,
                                                physical_goal_raw=hold_goal_raw,
                                                attempts=recovery_attempts,
                                                diagnostics=diagnostic_values.copy(),
                                            )) + "\n")
                                            print(
                                                f"[HOLD STALLED] C motor commands stopped"
                                                f" {error:.2f}deg from the selected goal;"
                                                f" attempts={recovery_attempts};"
                                                f" diagnostics={diagnostic_values}. "
                                                "GUI/UART remain running, result is NOT PASS.",
                                                flush=True)
                                            recovery_stalled_reported = True
                                if hold_replan_needed and hold_request_sequence is None and not ending:
                                    # Belt-and-suspenders: never send C direct
                                    # joint rearm without a validated obstacle-
                                    # aware path even if the deviation check
                                    # or state flags change in later edits.
                                    enforce_collision_aware_hold_policy(
                                        bool(safety_obstacles), measured,
                                        hold_goal_raw)
                                    # Replan from measured pose using the original
                                    # C controller and identical final goal.
                                    hold_rearms += 1
                                    recovery_attempts += 1
                                    recovery_replan_sent = True
                                    last_hold_replan_frame = target_frames
                                    recovery_stalled_reported = False
                                    recovery_quiet = 0
                                    recovery_command_quiet = 0
                                    recovery_previous = None
                                    recovery_previous_motor_targets = None
                                    # The firmware chooses this nearest legal
                                    # continuous goal from its real feedback.
                                    # Keep the wire request canonical, but
                                    # evaluate settling in the same turn domain.
                                    hold_request_goal_raw = hold_continuous_goal_raw(
                                        sequencer.goals[-1], measured, joint_limits_rad)
                                    hold_request_sequence = (sequencer.sequence + hold_rearms) & 0xff
                                    hold_request_at = time.monotonic()
                                    payload_request = bytes((hold_request_sequence,)) + struct.pack("<6h", *sequencer.goals[-1])
                                    sock.sendall(integration.frame(wire.CMD_REQUEST_JOINT_MOVE, payload_request))
                                    hold_replan_needed = False
                                    audit.write(json.dumps(dict(event="hold_replan", sequence=hold_request_sequence,
                                        goal_raw=sequencer.goals[-1],
                                        continuous_goal_raw=hold_request_goal_raw,
                                        measured_raw=measured,
                                        attempt=recovery_attempts,
                                        diagnostics=diagnostic_values.copy())) + "\n")
                                tool_xyz = p.getLinkState(
                                    arm, p.getNumJoints(arm) - 1,
                                    computeForwardKinematics=True,
                                )[4]
                                # Render trail/text at 10 Hz, not at the
                                # 100 Hz UART rate. The C controller, PyBullet
                                # physics, feedback and CSV stay per target frame.
                                # With a 10 s lifetime this also bounds visible
                                # trail objects to roughly 100 instead of 1000.
                                if not options.headless and target_frames % 10 == 0:
                                    if last_tool_xyz is not None:
                                        p.addUserDebugLine(
                                            last_tool_xyz, tool_xyz,
                                            [0.1, 0.7, 1.0], lineWidth=2,
                                            lifeTime=HOLD_TRACE_LIFETIME_S,
                                        )
                                    last_tool_xyz = tuple(tool_xyz)
                                    debug_text_id = p.addUserDebugText(
                                        f"Segment {sequencer.index + 1}/"
                                        f"{len(sequencer.goals)} | "
                                        f"physics={target_frames * FEEDBACK_STEP_S:.2f}s",
                                        [-0.65, -0.65, 0.85],
                                        textColorRGB=[0.1, 0.9, 0.2],
                                        textSize=1.25,
                                        replaceItemUniqueId=debug_text_id,
                                    )
                                row = {
                                    "simulated_time_s": round(
                                        target_frames * FEEDBACK_STEP_S, 3),
                                    "segment": max(0, sequencer.index + 1),
                                    "tool_x_m": tool_xyz[0],
                                    "tool_y_m": tool_xyz[1],
                                    "tool_z_m": tool_xyz[2],
                                }
                                for name, target, servo_target_rad, actual in zip(
                                    JOINT_NAMES, targets, physical_motor_targets_rad,
                                    measured
                                ):
                                    # Raw UART command is canonical; actual position
                                    # and actuator target are continuous. Do not
                                    # subtract unlike coordinate representations.
                                    row[f"command_{name}_deg"] = target / wire.JOINT_ANGLE_SCALE
                                    row[f"servo_target_{name}_deg"] = math.degrees(servo_target_rad)
                                    row[f"actual_{name}_deg"] = actual / wire.JOINT_ANGLE_SCALE
                                    row[f"error_{name}_deg"] = (
                                        math.degrees(servo_target_rad)
                                        - actual / wire.JOINT_ANGLE_SCALE
                                    )
                                trace.append(row)
                                trace_writer.writerow(row)
                                # Optional nonblocking stream of actual PyBullet
                                # measurements. The receiver must never render or
                                # wait here: UART feedback has already been sent.
                                if on_measurement is not None:
                                    on_measurement(
                                        row["simulated_time_s"], row["segment"],
                                        tuple(measured),
                                    )

                                if on_sample is not None:
                                    on_sample(row["simulated_time_s"], row["segment"],
                                              tuple(measured), targets, feedback_acks,
                                              parser.bad_checksum)
                                wall = time.monotonic()
                                if wall - last_audit >= 1.0:
                                    audit.write(json.dumps(dict(wall_s=wall-start_wall,
                                        target_frames=target_frames, feedback_acks=feedback_acks,
                                        feedback_ack_lag=target_frames-feedback_acks,
                                        bad_checksum=parser.bad_checksum, **row)) + "\n")
                                    last_audit = wall
                                    live_trace.flush()

                                if recovery_sequencer is not None and recovery_sequencer.observe(measured):
                                    recovery_segments_settled += 1
                                    audit.write(json.dumps(dict(
                                        event="hold_C_apf_segment_settled",
                                        index=recovery_segments_settled,
                                        total=len(recovery_sequencer.goals))) + "\n")
                                    if recovery_sequencer.done:
                                        recovery_sequence_next = (
                                            recovery_sequencer.sequence + 1) & 0xff
                                        recovery_route_events[-1]["completed"] = True
                                        recovery_sequencer = None
                                        recovery_settled = SETTLE_FRAME_COUNT
                                        recovering = False
                                        recovery_attempted = False
                                        hold_recoveries += 1
                                        if options.gui_safe_jog:
                                            # Re-enable only after real UART ACK +
                                            # physical settled frames for every
                                            # verified recovery segment.
                                            safe_gui_jog.complete_recovery()
                                        print("[HOLD C APF] All validated recovery "
                                              "segments settled", flush=True)
                                    else:
                                        sock.sendall(recovery_sequencer.next_request(
                                            time.monotonic()))
                                if sequencer.observe(measured):
                                    print(f"[SETTLED] Segment {sequencer.completed}/"
                                          f"{len(sequencer.goals)}")
                                    if not sequencer.done:
                                        sock.sendall(sequencer.next_request(time.monotonic()))
                                if options.frame_delay_s > 0:
                                    time.sleep(options.frame_delay_s)
                            elif (command == wire.CMD_JOINT_STATE_ACK and
                                  len(payload) == wire.JOINT_PAYLOAD_LEN):
                                feedback_acks += 1
                                if sequencer.index < 0 and feedback_acks >= 3:
                                    sock.sendall(sequencer.next_request(time.monotonic()))
                            elif command == wire.CMD_JOINT_MOVE_ACK:
                                if len(payload) != wire.MOVE_ACK_PAYLOAD_LEN:
                                    bad_acks += 1
                                    continue
                                if (recovery_sequencer is not None and
                                        recovery_sequencer.ack_pending):
                                    recovery_sequencer.acknowledge(payload)
                                    motor_target_rebase_pending = True
                                    print(
                                        "[HOLD C APF ACCEPTED] Recovery segment "
                                        f"{recovery_sequencer.index+1}/"
                                        f"{len(recovery_sequencer.goals)}",
                                        flush=True)
                                    continue
                                if hold_request_sequence is not None:
                                    seq, status = struct.unpack("<Bb", payload)
                                    if seq != hold_request_sequence or status != 0:
                                        raise RuntimeError(f"Hold recovery rejected: seq={seq}, status={status}")
                                    hold_request_sequence = None
                                    hold_rearm_acks += 1
                                    motor_target_rebase_pending = True
                                    if hold_request_goal_raw is None:
                                        raise RuntimeError("HOLD replan ACK without a selected continuous goal")
                                    hold_goal_raw = hold_request_goal_raw
                                    hold_request_goal_raw = None
                                    recovery_quiet = 0
                                    recovery_command_quiet = 0
                                    recovery_previous = None
                                    recovery_previous_motor_targets = None
                                    recovery_settled = 0
                                    recovery_stalled_reported = False
                                    audit.write(json.dumps(dict(
                                        event="hold_replan_ack", sequence=seq,
                                        continuous_goal_raw=hold_goal_raw)) + "\n")
                                    continue
                                sequencer.acknowledge(payload)
                                motor_target_rebase_pending = True
                                print(f"[ACCEPTED] Segment {sequencer.index + 1}/"
                                      f"{len(sequencer.goals)}")
                            elif (command == wire.CMD_DIAGNOSTICS_RESPONSE and
                                  len(payload) == 4 and diagnostics_pending):
                                previous_diagnostic = diagnostic_values.get(
                                    diagnostics_pending)
                                diagnostic_values[diagnostics_pending] = (
                                    struct.unpack("<I", payload)[0])
                                audit.write(json.dumps(dict(diagnostic=diagnostics_pending,
                                    value=diagnostic_values[diagnostics_pending],
                                    wall_s=time.monotonic()-start_wall)) + "\n")
                                if (options.interactive_hold and recovering
                                        and diagnostics_pending in (
                                            "controller_faults", "planner_faults")
                                        and previous_diagnostic is not None
                                        and diagnostic_values[diagnostics_pending] > previous_diagnostic):
                                    # UART frames still being ACKed does NOT
                                    # imply C motion tasks remain active.
                                    audit.write(json.dumps(dict(
                                        event="hold_firmware_fault_counter_increased",
                                        metric=diagnostics_pending,
                                        previous=previous_diagnostic,
                                        current=diagnostic_values[diagnostics_pending],
                                        last_fault_raw_last_polled=diagnostic_values.get(
                                            "last_fault"),
                                        recovery_attempts=recovery_attempts,
                                        motor_target_raw=[
                                            round(math.degrees(v) *
                                                  wire.JOINT_ANGLE_SCALE)
                                            for v in physical_motor_targets_rad],
                                        measured_raw=last_measured,
                                    )) + "\n")
                                    print(
                                        f"[C MOTION FAULT] {diagnostics_pending}: "
                                        f"{previous_diagnostic} -> "
                                        f"{diagnostic_values[diagnostics_pending]}. "
                                        "UART is alive; inspect uart_live.jsonl "
                                        "last_fault and HOLD recovery.",
                                        flush=True,
                                    )
                                diagnostics_pending = None
                    # A large GUI physics batch can delay the next socket
                    # read and feedback ACK processing. Measure it without
                    # dropping any motor command or altering the control loop.
                    batch_duration_s = time.monotonic() - batch_started
                    if ((len(parsed_frames) >= UART_BATCH_WARNING_FRAMES or
                         batch_duration_s >= UART_BATCH_WARNING_S) and
                            time.monotonic() - last_uart_batch_warning >= 1.0):
                        audit.write(json.dumps(dict(
                            event="uart_batch_delay",
                            frames=len(parsed_frames), bytes_received=len(data),
                            processing_wall_s=round(batch_duration_s, 3),
                            target_frames=target_frames,
                            feedback_acks=feedback_acks,
                            feedback_ack_lag=target_frames-feedback_acks,
                        )) + "\n")
                        last_uart_batch_warning = time.monotonic()

                live_polling = not sequencer.done or (options.interactive_hold and not ending)
                if live_polling and not diagnostics_pending and time.monotonic() >= next_diagnostic:
                    diagnostics_pending = names[diagnostics_index % len(names)]
                    diagnostics_index += 1
                    diagnostics_pending_at = time.monotonic()
                    next_diagnostic = diagnostics_pending_at + 0.25
                    request_diagnostics(sock, diagnostics_pending)
                if sequencer.done and not live_polling and not diagnostics_pending:
                    if not final_diagnostics:
                        diagnostics_index = 0
                        final_diagnostics = True

                    if diagnostics_index < len(names):
                        diagnostics_pending = names[diagnostics_index]
                        diagnostics_index += 1
                        diagnostics_pending_at = time.monotonic()
                        request_diagnostics(sock, diagnostics_pending)
                    else:
                        break
                if not data:
                    select.select([sock], [], [], 0.002)
            # 校验多个请求确实经 C 任务执行，不能仅凭 GUI 运动判断成功。
            fault_names = (
                "controller_faults", "planner_faults", "feedback_stale_count",
            )
            faults = {name: diagnostic_values.get(name) for name in fault_names}
            trajectory_ratio, trajectory_aligned = assess_clock_alignment(
                diagnostic_values.get("trajectory_samples"), target_frames,
                trajectory_period_s)
            monitor_ratio, monitor_aligned = assess_clock_alignment(
                diagnostic_values.get("monitoring_ticks"), target_frames,
                monitor_period_s)
            # A short APF route has a larger fraction of idle/settle time;
            # Monitor ticks include it, while planner samples do not.
            clock_ratio, clock_aligned = (
                (trajectory_ratio, trajectory_aligned) if prepare_motion is None
                else (monitor_ratio, monitor_aligned))
            clock_reference = (
                "trajectory_samples" if prepare_motion is None else "monitoring_ticks")
            passed = (
                clock_aligned
                and (not options.gui_safe_jog or (
                    safe_gui_jog is not None and
                    safe_gui_jog.applied >= 1 and
                    safe_gui_jog.applied == safe_gui_jog.completed ==
                    hold_recoveries and
                    recovery_segments_settled >= safe_gui_jog.applied))
                and sequencer.done
                and feedback_acks >= 3
                and bad_acks == 0
                and hold_rearms == hold_rearm_acks
                and not recovering
                and recovery_sequencer is None
                and (not options.interactive_hold or max(abs(a-b)/wire.JOINT_ANGLE_SCALE
                    for a,b in zip(last_measured, hold_goal_raw)) <= SETTLE_TOLERANCE_DEG)
                and parser.bad_checksum == 0
                and diagnostic_values.get("requests_accepted", -1)
                    >= len(sequencer.goals)
                and diagnostic_values.get("trajectory_samples", 0) > 0
                and diagnostic_values.get("controller_steps", 0) > 0
                and all(value == 0 for value in faults.values())
            )
            result = {
                "scope": "QEMU_FreeRTOS_C_joint_trajectories_UART_PyBullet_GUI",
                "scenario": options.scenario,
                "headless": options.headless,
                "motor_commands": "virtual_serial_only",
                "avoidance_preflight": avoidance_report,
                "obstacle_recovery_preflight": recovery_plan_report,
                "obstacle_recovery_segments_settled": recovery_segments_settled,
                "obstacle_recovery_routes": recovery_route_events,
                "physical_ee_at_hold_entry_m": settled_ee_xyz_m,
                "physical_ee_goal_error_at_hold_entry_mm":
                    settled_ee_goal_error_mm,
                "hold_entry_minimum_full_link_clearance_m": hold_entry_clearance_m,
                "hold_entry_closest_link_index": hold_entry_closest_link,
                "hold_entry_actual_joint_deg": hold_entry_actual_joint_deg,
                "runtime_collision_scans": collision_scans,
                "runtime_full_link_collision_free": (
                    bool(collision_scans) if prepare_motion is not None else None),
                "segments_requested": len(sequencer.goals),
                "segments_settled": sequencer.completed,
                "segment_final_errors_deg": sequencer.segment_final_errors_deg,
                "max_final_segment_error_deg": max(
                    sequencer.segment_final_errors_deg, default=None
                ),
                "simulated_physics_s": round(target_frames * FEEDBACK_STEP_S, 3),
                "trajectory_task_nominal_elapsed_s": round(
                    diagnostic_values.get("trajectory_samples", 0) * trajectory_period_s, 3),
                "rtos_trajectory_to_physics_time_ratio": trajectory_ratio,
                "rtos_monitor_to_physics_time_ratio": monitor_ratio,
                "clock_alignment_reference": clock_reference,
                "clock_alignment_passed": clock_aligned,
                "clock_alignment_tolerance_fraction": CLOCK_ALIGNMENT_MAX_RELATIVE_ERROR,
                "clock_alignment_note": "Planning task sample count times configured period; "
                                        "not a hardware-tick precision measurement",
                "pybullet_step_calls_per_feedback": step_calls_per_feedback,
                "internal_substeps_per_feedback": SIMULATED_STEPS_PER_TARGET,
                "target_frames": target_frames,
                "feedback_acks": feedback_acks,
                "bad_ack_frames": bad_acks,
                "bad_checksum_frames": parser.bad_checksum,
                "uart_partial_writes": sock.partial_writes,
                "uart_would_block": sock.would_block,
                "qemu_clock": "adaptive_icount" if sys.platform == "win32" else "wall",
                "physics_clock_gate": "qmp_stop_cont",
                "qmp_physics_pauses": clock_gate.pauses,
                "wall_duration_s": time.monotonic()-start_wall,
                "interactive_hold": options.interactive_hold,
                "native_gui_picking": native_picking_allowed,
                "gui_safe_jog": options.gui_safe_jog,
                "cancelled": bool(safe_gui_jog is not None and
                                  safe_gui_jog.cancel_requested),
                "cancel_reason": (
                    "no_guarded_jog_requested"
                    if safe_gui_jog is not None and safe_gui_jog.cancel_requested
                    else None),
                "safe_gui_jog_events": (
                    safe_gui_jog.events if safe_gui_jog is not None else []),
                "safe_gui_jog_applied": (
                    safe_gui_jog.applied if safe_gui_jog is not None else 0),
                "safe_gui_jog_refused": (
                    safe_gui_jog.refused if safe_gui_jog is not None else 0),
                "safe_gui_jog_completed": (
                    safe_gui_jog.completed if safe_gui_jog is not None else 0),
                "avoidance_hold_recovery_policy": metadata[
                    "avoidance_hold_recovery_policy"],
                "hold_observe_only": options.hold_observe_only,
                "diagnostics_timeout_s": options.diagnostics_timeout_s,
                "hold_final_error_deg": max(abs(a-b)/wire.JOINT_ANGLE_SCALE
                    for a,b in zip(last_measured, hold_goal_raw)),
                "hold_recoveries": hold_recoveries,
                "hold_recovery_attempts": recovery_attempts,
                "hold_recovery_command_quiet_frames": recovery_command_quiet,
                "hold_continuous_goal_raw": hold_goal_raw,
                "gui_joint_stop_corrections": gui_joint_stop_corrections,
                "gui_joint_stop_last": gui_joint_stop_last,
                "gui_joint_stop_implementation": (
                    "post_physics_urdf_joint_state_guard"
                    if not options.headless else "disabled_in_direct"),
                "hold_rearms": hold_rearms,
                "hold_rearm_acks": hold_rearm_acks,
                "hold_wall_s": 0.0 if hold_started is None else time.monotonic()-hold_started,
                "diagnostics": diagnostic_values,
                "fault_counters": faults,
                "passed": passed,
                "not_tested": [
                    "Cartesian line/arc UART commands",
                    *(["full-link collision safety"] if prepare_motion is None else []),
                    "real servo torque control",
                    "long-term worst-case timing",
                ],
            }
            output_dir.mkdir(parents=True, exist_ok=True)
            live_trace.flush()
            result["plot_saved"] = write_plot(output_dir, trace)
            if not clock_aligned:
                audit.write(json.dumps(dict(event="clock_alignment_failed",
                    rtos_trajectory_to_physics_time_ratio=clock_ratio,
                    nominal_trajectory_s=result["trajectory_task_nominal_elapsed_s"],
                    simulated_physics_s=result["simulated_physics_s"])) + "\n")
            (output_dir / "result.json").write_text(
                json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
            )
            completed_successfully = bool(result["passed"])
            return result
    except BaseException as error:
        failure = dict(passed=False, error=repr(error),
            target_frames=target_frames, feedback_acks=feedback_acks,
            segments_settled=sequencer.completed, diagnostics=diagnostic_values,
            physical_ee_at_hold_entry_m=settled_ee_xyz_m,
            physical_ee_goal_error_at_hold_entry_mm=settled_ee_goal_error_mm,
            planned_route_minimum_full_link_clearance_m=(
                avoidance_report.get("minimum_planned_full_link_clearance_m")
                if avoidance_report is not None else None),
            hold_entry_minimum_full_link_clearance_m=hold_entry_clearance_m,
            hold_entry_closest_link_index=hold_entry_closest_link,
            hold_entry_actual_joint_deg=hold_entry_actual_joint_deg,
            gui_joint_stop_corrections=gui_joint_stop_corrections,
            gui_joint_stop_last=gui_joint_stop_last,
            segments_requested=len(sequencer.goals),
            obstacle_recovery_preflight=recovery_plan_report,
            obstacle_recovery_segments_settled=recovery_segments_settled,
            obstacle_recovery_routes=recovery_route_events,
            hold_recoveries=hold_recoveries,
            hold_rearms=hold_rearms,
            hold_rearm_acks=hold_rearm_acks,
            hold_final_error_deg=max(
                abs(a - b) / wire.JOINT_ANGLE_SCALE
                for a, b in zip(last_measured, hold_goal_raw)),
            native_gui_picking=native_picking_allowed,
            gui_safe_jog=options.gui_safe_jog,
            safe_gui_jog_events=(
                safe_gui_jog.events if safe_gui_jog is not None else []),
            safe_gui_jog_applied=(
                safe_gui_jog.applied if safe_gui_jog is not None else 0),
            safe_gui_jog_refused=(
                safe_gui_jog.refused if safe_gui_jog is not None else 0),
            safe_gui_jog_completed=(
                safe_gui_jog.completed if safe_gui_jog is not None else 0),
            avoidance_hold_recovery_policy=metadata[
                "avoidance_hold_recovery_policy"])
        if (not options.headless and client >= 0 and
                (isinstance(error, Stage5GuiDisconnectedError) or
                 (isinstance(error, p.error) and not p.isConnected(client)))):
            failure.update(
                fault_type=Stage5GuiDisconnectedError.fault_type,
                fault_phase="native_gui_disconnected_before_verdict",
                gui_finish_requested=bool(
                    safe_gui_jog is not None and safe_gui_jog.finish_requested),
                completed_recoveries=hold_recoveries,
                note="Motion completion is retained as evidence; "
                     "unexpected GUI disconnect is never PASS.")
            audit.write(json.dumps(dict(
                event=Stage5GuiDisconnectedError.fault_type,
                target_frames=target_frames,
                completed_recoveries=hold_recoveries)) + "\n")
        if isinstance(error, Stage5UnvalidatedHoldRecoveryError):
            failure.update(
                fault_type=error.fault_type,
                fault_phase="interactive_hold",
                unsafe_return_rearm_sent=False,
                deviation_deg=error.deviation_deg,
                measured_raw=list(error.measured_raw),
                goal_raw=list(error.goal_raw),
            )
            audit.write(json.dumps(dict(
                event="hold_unvalidated_recovery_blocked",
                error_deg=error.deviation_deg,
                measured_raw=list(error.measured_raw),
                goal_raw=list(error.goal_raw),
                target_frames=target_frames,
                unvalidated_rearm_sent=False,
            )) + "\n")
        if isinstance(error, Stage5RuntimeClearanceError):
            failure.update(
                fault_type=error.fault_type,
                fault_phase=error.phase,
                fault_link_index=error.link_index,
                fault_obstacle_id=error.obstacle_id,
                measured_clearance_m=error.distance_m,
                required_clearance_m=error.required_m,
                clearance_violation_m=error.required_m-error.distance_m,
                actual_joint_deg=[v / wire.JOINT_ANGLE_SCALE
                                  for v in last_measured],
                planned_final_C_IK_joint_deg=(
                    [math.degrees(v) for v in
                     avoidance_report["joint_waypoints_rad"][-1]]
                    if avoidance_report is not None else None),
                segments_requested=len(sequencer.goals),
            )
            audit.write(json.dumps(dict(
                event=error.fault_type, phase=error.phase,
                link=error.link_index, obstacle=error.obstacle_id,
                measured_clearance_m=error.distance_m,
                required_clearance_m=error.required_m,
                target_frames=target_frames,
                segments_settled=sequencer.completed,
            )) + "\n")
        if isinstance(error, (JointFeedbackRangeError, JointPhysicalSafetyError)):
            fault_type = error.fault_type
            failure.update(
                fault_type=fault_type,
                fault_joint=error.joint_name,
                fault_joint_index=error.joint_index,
                fault_raw_centideg=error.raw_centideg,
                fault_angle_deg=error.angle_deg,
                last_measured_raw=last_measured,
            )
            audit.write(json.dumps(dict(event=fault_type,
                joint=error.joint_name, raw_centideg=error.raw_centideg,
                actual_angle_deg=error.angle_deg, target_frames=target_frames,
                feedback_acks=feedback_acks)) + "\n")
        live_trace.flush()
        audit.flush()
        (output_dir / "result.json").write_text(
            json.dumps(failure, indent=2, ensure_ascii=False), encoding="utf-8")
        if (isinstance(error, (
                JointFeedbackRangeError, JointPhysicalSafetyError,
                Stage5RuntimeClearanceError,
                Stage5UnvalidatedHoldRecoveryError))
                and not options.headless and client >= 0):
            try:
                preserve_gui_after_physics_fault(
                    p, qemu, stop_requested, error)
            except Exception as presentation_error:
                # A renderer failure never replaces the original protocol fault.
                print(f"[SAFE STOP] GUI preservation unavailable: "
                      f"{presentation_error!r}", flush=True)
        raise
    finally:
        # QEMU 停止后才保持 GUI，避免固件在暂停反馈时误报失联。
        if qemu is not None:
            if qemu.poll() is None:
                qemu.terminate()
            try:
                qemu.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                qemu.kill()
                qemu.communicate()
        if client >= 0 and p.isConnected(client):
            if not options.headless and options.hold_window:
                try:
                    status = "成功完成" if completed_successfully else "中断或失败"
                    input(f"[GUI] 演示{status}，按 Enter 关闭窗口...")
                except EOFError:
                    pass
            # Native window X may already have disconnected Bullet. Do not
            # overwrite the original failure with a second disconnect error.
            if p.isConnected(client):
                p.disconnect(client)
        # 异常中途退出也留存已接收的轨迹，用于诊断。
        audit.close()
        live_trace.close()
        qemu_error.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path,
                        help="ARM FreeRTOS firmware ELF built for Stage5 QEMU UART")
    parser.add_argument("--qemu", default="qemu-system-arm")
    parser.add_argument("--scenario", choices=tuple(SCENARIOS),
                        default="six_axis")
    parser.add_argument("--stage5-c-apf-library", type=Path,
                        help="Use production C APF/IK with all-link preflight")
    parser.add_argument("--stage5-route-case", choices=("clear", "blocked"),
                        default="clear")
    parser.add_argument("--stage5-raw-route", action="store_true",
                        help="Keep all original C APF waypoints (no compaction)")
    parser.add_argument("--gui-joint-travel", choices=GUI_JOINT_TRAVEL_MODES,
                        default=DEFAULT_GUI_JOINT_TRAVEL_MODE,
                        help="GUI-only physical picking range; DIRECT always keeps URDF")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "build/stage5-visual-demo")
    parser.add_argument("--headless", action="store_true",
                        help="Use DIRECT for automated testing (no GUI)")
    parser.add_argument("--frame-delay-ms", type=float, default=50.0,
                        help="Additional wall-clock delay per feedback frame; GUI only")
    parser.add_argument("--headless-step-delay-ms", type=float, default=0.0,
                        help="CI stress test: simulated slow GUI frame handling in DIRECT, "
                             "without changing C firmware or physical time step")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--no-hold", action="store_true",
                        help="Close the PyBullet window immediately after finishing")
    args = parser.parse_args()
    if not math.isfinite(args.headless_step_delay_ms) or not (0 <= args.headless_step_delay_ms <= 50):
        parser.error("--headless-step-delay-ms must be between 0 and 50")
    if not args.headless and args.headless_step_delay_ms:
        parser.error("--headless-step-delay-ms requires --headless")
    if args.stage5_raw_route and args.stage5_c_apf_library is None:
        parser.error("--stage5-raw-route requires --stage5-c-apf-library")
    options = DemoOptions(
        scenario=args.scenario,
        headless=args.headless,
        frame_delay_s=(args.headless_step_delay_ms if args.headless
                       else args.frame_delay_ms) / 1000.0,
        timeout_s=args.timeout_s,
        hold_window=not args.no_hold,
        gui_joint_travel=args.gui_joint_travel,
    )
    prepare_motion = None
    if args.stage5_c_apf_library is not None:
        from stage5_avoidance_uart_gate import make_avoidance_preflight
        prepare_motion = make_avoidance_preflight(
            args.stage5_c_apf_library, args.stage5_route_case,
            shortcut_override=False if args.stage5_raw_route else None)
    report = run(args.elf, args.output_dir, args.qemu, options,
                 prepare_motion=prepare_motion)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())