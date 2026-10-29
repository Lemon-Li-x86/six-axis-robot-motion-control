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
HOLD_MAX_REPLAN_ATTEMPTS = 8
HOLD_TRACE_LIFETIME_S = 10.0
# Log costly UART batches at most once a second. Diagnostic-only, no pacing.
UART_BATCH_WARNING_FRAMES = 24
UART_BATCH_WARNING_S = 0.15
DIAGNOSTICS_TIMEOUT_S = 5.0  # Default; simulator observation can override.
# Acceptance criterion for physically meaningful temporal correspondence.
# Counts are diagnostic estimates, not proof of exact FreeRTOS tick alignment.
CLOCK_ALIGNMENT_MAX_RELATIVE_ERROR = 0.25
SERVO_MAX_TORQUE_NM = 150.0  # PyBullet revolute joint test fixture.
MAX_PAYLOAD_RAW = 32767
TRACE_HEADER = (
    "simulated_time_s", "segment", "tool_x_m", "tool_y_m", "tool_z_m",
    *(f"command_{name}_deg" for name in JOINT_NAMES),
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


class MotionSequencer:
    """按真实反馈到位情况依次发出关节请求，不覆盖未完成的请求。"""

    def __init__(self, goals: Sequence[Sequence[int]]) -> None:
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
        self.sequence = (41 + self.index) & 0xFF
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
    """校验 UART int16 请求格式；真正的关节限位仍由 C 固件判定。"""
    if not goals:
        raise ValueError("Demonstration needs at least one motion segment")
    normalized = []
    for goal in goals:
        if len(goal) != wire.JOINT_COUNT:
            raise ValueError("Each target must contain exactly six angles")
        if any(type(value) is not int or abs(value) > MAX_PAYLOAD_RAW
               for value in goal):
            raise ValueError("Each centidegree target must fit signed int16")
        normalized.append(tuple(goal))
    return tuple(normalized)


def joint_feedback_frame(raw: Sequence[int]) -> bytes:
    """按现有协议编码模拟关节测量值；不复制校验和计算。"""
    payload = struct.pack("<" + str(wire.JOINT_COUNT) + "h", *raw)
    return integration.frame(wire.CMD_JOINT_STATE, payload)


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
        ax.plot(time_s, [r[f"command_{name}_deg"] for r in rows],
                label="Firmware UART target", linewidth=1.5)
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
    task_periods = json.loads(DEFAULT_JSON.read_text(
        encoding="utf-8"))["task_period_ms"]
    trajectory_period_s = task_periods["trajectory"] / 1000.0
    monitor_period_s = task_periods["monitor"] / 1000.0
    step_calls_per_feedback = (SIMULATED_STEPS_PER_TARGET if options.headless else 1)
    safety_obstacles: tuple[int, ...] = ()
    safety_clearance_m = 0.0
    collision_scans = 0
    avoidance_report: dict | None = None
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
    ending = False
    hold_request_sequence = None
    hold_request_at = 0.0
    hold_rearms = 0
    hold_rearm_acks = 0
    hold_recoveries = 0
    hold_replan_needed = False
    recovering = False
    recovery_attempts = 0
    recovery_quiet = 0
    recovery_settled = 0
    recovery_previous = None
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
    metadata = dict(elf=str(elf.resolve()),
        elf_sha256=hashlib.sha256(elf.read_bytes()).hexdigest(),
        config_sha256=hashlib.sha256(DEFAULT_JSON.read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
        python=sys.version, platform=sys.platform, qemu_executable=qemu_executable,
        pybullet_api_version=p.getAPIVersion(),
        physics_mode="DIRECT" if options.headless else "GUI",
        native_gui_picking=not options.headless,
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
                p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
                # Keep native debug controls visible. Hiding them made the
                # sole physical GUI look like an empty rendering viewport.
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.setGravity(0, 0, -9.81)
            step_calls_per_feedback, warmup_calls = configure_physics_stepping(
                p, options.headless)
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            for idx, raw in zip(joints, START_RAW):
                p.resetJointState(arm, idx, math.radians(raw / wire.JOINT_ANGLE_SCALE))
            p.setJointMotorControlArray(
                arm, joints, p.POSITION_CONTROL,
                targetPositions=[math.radians(v / wire.JOINT_ANGLE_SCALE)
                                 for v in START_RAW],
                forces=[SERVO_MAX_TORQUE_NM] * wire.JOINT_COUNT,
            )
            for _ in range(warmup_calls):
                p.stepSimulation()
            if prepare_motion is not None:
                # Preflight actual physics state before UART actuation.
                plan = prepare_motion(p, arm, joints, client)
                targets = [tuple(
                    int(round(math.degrees(q) * wire.JOINT_ANGLE_SCALE))
                    for q in pose)
                    for pose in plan["joint_waypoints_rad"][1:]]
                sequencer = MotionSequencer(targets)
                safety_obstacles = tuple(plan["obstacle_ids"])
                safety_clearance_m = float(plan["static_collision_clearance_m"])
                avoidance_report = {
                    key: value for key, value in plan.items()
                    if key != "obstacle_ids"
                }
                (output_dir / "avoidance_preflight.json").write_text(
                    json.dumps(avoidance_report, indent=2),
                    encoding="utf-8")
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
            sock.sendall(joint_feedback_frame(angles_raw(p, arm, joints)))
            start_wall = time.monotonic()
            last_frame_wall = start_wall
            while True:
                now = time.monotonic()
                if sequencer.done and hold_started is None:
                    hold_started = now
                    if options.interactive_hold:
                        print("[HOLD] PyBullet native mouse picking is active; controller holds the final goal", flush=True)
                if stop_requested is not None and stop_requested():
                    if not sequencer.done or not options.interactive_hold:
                        raise InterruptedError("Observer requested live control shutdown")
                    ending = True
                if options.interactive_hold and hold_started is not None:
                    if options.hold_duration_s is not None and now-hold_started >= options.hold_duration_s:
                        ending = True
                elif now-start_wall >= options.timeout_s:
                    raise TimeoutError(f"Demo timed out; completed {sequencer.completed}/{len(sequencer.goals)} segments")
                if hold_request_sequence is not None and now-hold_request_at > 8.0:
                    raise TimeoutError("Hold recovery movement ACK timeout")
                if qemu.poll() is not None:
                    raise RuntimeError(f"QEMU terminated: {qemu.returncode}")
                if time.monotonic() - last_frame_wall > 8.0:
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
                                if sequencer.accepted or (sequencer.done and options.interactive_hold):
                                    p.setJointMotorControlArray(
                                        arm, joints, p.POSITION_CONTROL,
                                        targetPositions=[
                                            math.radians(v / wire.JOINT_ANGLE_SCALE)
                                            for v in targets
                                        ],
                                        forces=[SERVO_MAX_TORQUE_NM] * wire.JOINT_COUNT,
                                    )
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
                                measured = angles_raw(p, arm, joints)
                                last_measured = tuple(measured)
                                if safety_obstacles:
                                    # Every 10ms actual physical state: base,
                                    # movable and fixed links vs static boxes.
                                    p.performCollisionDetection()
                                    collision_scans += 1
                                    for link in range(-1, p.getNumJoints(arm)):
                                        for obstacle in safety_obstacles:
                                            contacts = p.getClosestPoints(
                                                arm, obstacle, safety_clearance_m,
                                                linkIndexA=link, linkIndexB=-1)
                                            if contacts:
                                                raise RuntimeError(
                                                    "Stage5 runtime full-arm collision "
                                                    f"link={link}, obstacle={obstacle}, "
                                                    f"distance_m={min(c[8] for c in contacts):.6f}")

                                sock.sendall(joint_feedback_frame(measured))
                                # Detect a real joint-space deviation, irrespective
                                # of whether it came from mouse picking or test
                                # forces. Never interpret UART motor commands as
                                # the final requested joint goal.
                                if sequencer.done and options.interactive_hold and not ending:
                                    deviation = max(abs(a-b)/wire.JOINT_ANGLE_SCALE
                                                    for a,b in zip(measured, sequencer.goals[-1]))
                                    if deviation > HOLD_DEVIATION_TRIGGER_DEG and not recovering:
                                        recovery_stalled_reported = False
                                        recovering = True
                                        recovery_attempts = 0
                                        recovery_quiet = 0
                                        recovery_settled = 0
                                        recovery_previous = None
                                        audit.write(json.dumps(dict(event="hold_deviation",
                                            error_deg=deviation)) + "\n")
                                if recovering and not ending:
                                    error = max(abs(a-b)/wire.JOINT_ANGLE_SCALE for a,b in zip(measured, sequencer.goals[-1]))
                                    recovery_settled = recovery_settled+1 if error <= SETTLE_TOLERANCE_DEG else 0
                                    quiet = recovery_previous is not None and max(abs(a-b) for a,b in zip(measured, recovery_previous)) <= 1
                                    recovery_quiet = recovery_quiet+1 if quiet else 0
                                    recovery_previous = tuple(measured)
                                    if recovery_settled >= SETTLE_FRAME_COUNT:
                                        recovering = False
                                        hold_recoveries += 1
                                        audit.write(json.dumps(dict(event="hold_recovered", error_deg=error,
                                            attempts=recovery_attempts)) + "\n")
                                    elif (recovery_quiet >= round(
                                            HOLD_STAGNANT_PERIOD_S / FEEDBACK_STEP_S)
                                          and hold_request_sequence is None):
                                        if options.hold_observe_only:
                                            # Passive experiment: keep the C PID active,
                                            # never add another motion request or stop
                                            # solely because the joint error persists.
                                            if not recovery_stalled_reported:
                                                audit.write(json.dumps(dict(
                                                    event="hold_observe_stalled",
                                                    error_deg=error,
                                                )) + "\n")
                                                recovery_stalled_reported = True
                                        else:
                                            # Normal mode: bounded replan from current
                                            # feedback using the SAME firmware goal.
                                            if recovery_attempts >= HOLD_MAX_REPLAN_ATTEMPTS:
                                                raise RuntimeError(
                                                    f"Hold recovery did not converge: {error:.2f}deg")
                                            hold_replan_needed = True
                                        recovery_quiet = 0
                                if hold_replan_needed and hold_request_sequence is None and not ending:
                                    # Replan from measured pose using the original
                                    # C controller and identical final goal.
                                    hold_rearms += 1
                                    recovery_attempts += 1
                                    hold_request_sequence = (sequencer.sequence + hold_rearms) & 0xff
                                    hold_request_at = time.monotonic()
                                    payload_request = bytes((hold_request_sequence,)) + struct.pack("<6h", *sequencer.goals[-1])
                                    sock.sendall(integration.frame(wire.CMD_REQUEST_JOINT_MOVE, payload_request))
                                    hold_replan_needed = False
                                    audit.write(json.dumps(dict(event="hold_replan", sequence=hold_request_sequence,
                                        goal_raw=sequencer.goals[-1], measured_raw=measured)) + "\n")
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
                                for name, target, actual in zip(
                                    JOINT_NAMES, targets, measured
                                ):
                                    row[f"command_{name}_deg"] = target / wire.JOINT_ANGLE_SCALE
                                    row[f"actual_{name}_deg"] = actual / wire.JOINT_ANGLE_SCALE
                                    row[f"error_{name}_deg"] = (
                                        target - actual) / wire.JOINT_ANGLE_SCALE
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
                                if hold_request_sequence is not None:
                                    seq, status = struct.unpack("<Bb", payload)
                                    if seq != hold_request_sequence or status != 0:
                                        raise RuntimeError(f"Hold recovery rejected: seq={seq}, status={status}")
                                    hold_request_sequence = None
                                    hold_rearm_acks += 1
                                    audit.write(json.dumps(dict(event="hold_replan_ack", sequence=seq)) + "\n")
                                    continue
                                sequencer.acknowledge(payload)
                                print(f"[ACCEPTED] Segment {sequencer.index + 1}/"
                                      f"{len(sequencer.goals)}")
                            elif (command == wire.CMD_DIAGNOSTICS_RESPONSE and
                                  len(payload) == 4 and diagnostics_pending):
                                diagnostic_values[diagnostics_pending] = (
                                    struct.unpack("<I", payload)[0])
                                audit.write(json.dumps(dict(diagnostic=diagnostics_pending,
                                    value=diagnostic_values[diagnostics_pending],
                                    wall_s=time.monotonic()-start_wall)) + "\n")
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
            # Active planner samples need not span physical settling and
            # final UART diagnostic polling after a short segment. Keep
            # both clocks visible; use always-running Monitor ticks for
            # overall experiment wall/physics alignment only in the
            # C-APF route execution mode.
            trajectory_ratio, trajectory_clock_aligned = assess_clock_alignment(
                diagnostic_values.get("trajectory_samples"), target_frames,
                trajectory_period_s)
            monitor_ratio, monitor_clock_aligned = assess_clock_alignment(
                diagnostic_values.get("monitoring_ticks"), target_frames,
                monitor_period_s)
            if prepare_motion is None:
                clock_ratio, clock_aligned = (
                    trajectory_ratio, trajectory_clock_aligned)
                clock_reference = "trajectory_samples"
            else:
                clock_ratio, clock_aligned = (
                    monitor_ratio, monitor_clock_aligned)
                clock_reference = "monitoring_ticks"
            passed = (
                clock_aligned
                and sequencer.done
                and feedback_acks >= 3
                and bad_acks == 0
                and hold_rearms == hold_rearm_acks
                and not recovering
                and (not options.interactive_hold or max(abs(a-b)/wire.JOINT_ANGLE_SCALE
                    for a,b in zip(last_measured, sequencer.goals[-1])) <= SETTLE_TOLERANCE_DEG)
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
                "clock_alignment_note": (
                    "Planner samples cover active motion references; "
                    "Monitor ticks cover full simulation including "
                    "settling and UART diagnostics; not a precision "
                    "Cortex-M hardware time measurement."),
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
                "hold_observe_only": options.hold_observe_only,
                "diagnostics_timeout_s": options.diagnostics_timeout_s,
                "hold_final_error_deg": max(abs(a-b)/wire.JOINT_ANGLE_SCALE
                    for a,b in zip(last_measured, sequencer.goals[-1])),
                "hold_recoveries": hold_recoveries,
                "hold_rearms": hold_rearms,
                "hold_rearm_acks": hold_rearm_acks,
                "hold_wall_s": 0.0 if hold_started is None else time.monotonic()-hold_started,
                "diagnostics": diagnostic_values,
                "fault_counters": faults,
                "passed": passed,
                "not_tested": [
                    "Cartesian line/arc UART commands",
                    "full-link collision safety",
                    "real servo torque control",
                    "long-term worst-case timing",
                ],
            }
            output_dir.mkdir(parents=True, exist_ok=True)
            live_trace.flush()
            result["plot_saved"] = write_plot(output_dir, trace)
            if not clock_aligned:
                audit.write(json.dumps(dict(event="clock_alignment_failed",
                    rtos_trajectory_to_physics_time_ratio=trajectory_ratio,
                    rtos_monitor_to_physics_time_ratio=monitor_ratio,
                    clock_alignment_reference=clock_reference,
                    nominal_trajectory_s=result["trajectory_task_nominal_elapsed_s"],
                    simulated_physics_s=result["simulated_physics_s"])) + "\n")
            (output_dir / "result.json").write_text(
                json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8"
            )
            completed_successfully = bool(result["passed"])
            return result
    except BaseException as error:
        (output_dir / "result.json").write_text(json.dumps(dict(passed=False,
            error=repr(error), target_frames=target_frames, feedback_acks=feedback_acks,
            segments_settled=sequencer.completed, diagnostics=diagnostic_values), indent=2), encoding="utf-8")
        raise
    finally:
        # QEMU 停止后才保持 GUI，避免固件在暂停反馈时误报失联。
        if qemu is not None:
            qemu.terminate()
            try:
                qemu.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                qemu.kill()
                qemu.communicate()
        if client >= 0:
            if not options.headless and options.hold_window:
                try:
                    status = "成功完成" if completed_successfully else "中断或失败"
                    input(f"[GUI] 演示{status}，按 Enter 关闭窗口...")
                except EOFError:
                    pass
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
                        help="Opt in: require production C APF, C IK, and all-link "
                             "preflight before virtual UART motion")
    parser.add_argument("--stage5-route-case",
                        choices=("clear", "blocked"), default="clear")
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
    options = DemoOptions(
        scenario=args.scenario,
        headless=args.headless,
        frame_delay_s=(args.headless_step_delay_ms if args.headless
                       else args.frame_delay_ms) / 1000.0,
        timeout_s=args.timeout_s,
        hold_window=not args.no_hold,
    )
    prepare_motion = None
    if args.stage5_c_apf_library is not None:
        from stage5_avoidance_uart_gate import make_avoidance_preflight
        prepare_motion = make_avoidance_preflight(
            args.stage5_c_apf_library, args.stage5_route_case)
    report = run(args.elf, args.output_dir, args.qemu, options,
                 prepare_motion=prepare_motion)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())