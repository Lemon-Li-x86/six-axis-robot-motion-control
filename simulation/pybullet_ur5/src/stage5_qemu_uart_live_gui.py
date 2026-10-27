"""Live QEMU/FreeRTOS/UART/PyBullet six-axis visualisation.

Two independent processes:
  worker: real ARM QEMU <-> UART <-> PyBullet DIRECT physical feedback loop
  parent: PyBullet GUI mirrors *current measured joint angles* as they arrive

The GUI sends leased mouse-grab intent to DIRECT; it does not calculate
trajectories, run PID, or fabricate UART feedback. Frames may be dropped if rendering falls behind; the authoritative
trace and verdict are always written by the worker. No prior recording is
required or played back. Windows spawn multiprocessing is supported.
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import queue
import sys
import time
import traceback
from pathlib import Path
from typing import Any

import qemu_uart_receiver as wire
from stage4_dynamic_closed_loop import JOINT_NAMES, ROOT, START_RAW, URDF, joint_indices
from stage5_qemu_uart_visual_demo import DemoOptions, SCENARIOS, run
from stage5_drag_interaction import DragMailbox, MouseDrag, PhysicalDrag

# Display-only policies. The serial/physics controller has no GUI delays.
REFRESH_HZ = 30.0
MIN_OBSERVED_MOTION_DEG = 2.0
RawFrame = tuple[float, int, tuple[int, ...]]


class LatestMailbox:
    """Single latest sample; no pipe, feeder thread, queue drain, or blocking lock.

    Producer/consumer acquire the shared lock with blocking=False. A stalled
    renderer can only lose display samples, never stall UART or child shutdown.
    """
    def __init__(self, context):
        self.data = context.Array("d", 19, lock=False)
        self.lock = context.Lock()
        self.skipped = context.Value("L", 0, lock=False)
        self.seen = 0

    def publish(self, stamp, segment, measured, targets, acks, bad):
        if not self.lock.acquire(False):
            self.skipped.value += 1
            return
        try:
            self.data[0] += 1
            self.data[1:] = (stamp, segment, *measured, *targets, acks, bad,
                             self.skipped.value, time.monotonic())
        finally:
            self.lock.release()

    def get_nowait(self):
        if not self.lock.acquire(False):
            raise queue.Empty
        try:
            values = tuple(self.data)
        finally:
            self.lock.release()
        if int(values[0]) == self.seen:
            raise queue.Empty
        self.seen = int(values[0])
        self.status = values
        return (values[1], int(values[2]), tuple(int(v) for v in values[3:9]))


def capture_worker(
    elf: str, output: str, qemu: str, scenario: str, timeout_s: float,
    channel: Any, stop: Any, drag_channel=None, ready=None,
    interactive=False, hold_seconds=None, test_drag=False, test_drag_cycles=1,
) -> None:
    """Run unchanged firmware-driven feedback physics in a spawned process."""
    options = DemoOptions(
        scenario=scenario,
        headless=True,
        frame_delay_s=0.0,
        timeout_s=timeout_s,
        hold_window=False,
        interactive_hold=interactive,
        hold_duration_s=hold_seconds,
    )
    physical_drag = PhysicalDrag(drag_channel) if drag_channel is not None else None
    hold_at = None
    test_anchor = None
    recovery = dict(peak_displacement_deg=0.0, recovered_frames=0,
                    max_uart_change_deg=0.0, released=False)
    initial_targets = None

    def physics(bullet, arm, joints, sim_time, done, ending):
        nonlocal hold_at, test_anchor
        if done and hold_at is None:
            hold_at = sim_time
            if ready is not None:
                ready.set()
        if test_drag and done:
            elapsed = sim_time-hold_at
            # Same input mailbox and DIRECT point constraint used by mouse drag.
            link = joints[2]
            if test_anchor is None:
                pose = bullet.getLinkState(arm, link, computeForwardKinematics=True)
                test_anchor = tuple(pose[0])
            cycle = int(elapsed // 12.0)
            phase = elapsed - cycle * 12.0
            active = cycle < test_drag_cycles and 1.0 <= phase < 1.8 and not ending
            target = tuple(v+d for v,d in zip(test_anchor, (0.08, 0.04, 0.06)))
            drag_channel.publish(active, link, (0.,0.,0.), target)
        if physical_drag is not None:
            if ending:
                drag_channel.publish(False, -1, (0.,0.,0.), (0.,0.,0.))
            before = (physical_drag.started, physical_drag.released)
            physical_drag.apply(bullet, arm)
            after = (physical_drag.started, physical_drag.released)
            if after[0] > before[0]:
                recovery["released"] = False
                recovery["recovered_frames"] = 0
            if after[1] > before[1]:
                recovery["released"] = True
            if before != after:
                with (Path(output) / "drag_events.jsonl").open("a", encoding="utf-8") as events:
                    events.write(json.dumps(dict(sim_time_s=sim_time, started=after[0],
                        released=after[1], active=physical_drag.active)) + "\n")
            return after[1] > before[1], physical_drag.active

    def push(stamp, segment, measured, targets, acks, bad):
        nonlocal initial_targets
        channel.publish(stamp, segment, measured, targets, acks, bad)
        if hold_at is not None:
            goal = SCENARIOS[scenario][-1]
            error = max(abs(a-b)/wire.JOINT_ANGLE_SCALE for a,b in zip(measured, goal))
            if initial_targets is None:
                initial_targets = targets
            change = max(abs(a-b)/wire.JOINT_ANGLE_SCALE for a,b in zip(targets, initial_targets))
            recovery["max_uart_change_deg"] = max(recovery["max_uart_change_deg"], change)
            if physical_drag is not None and physical_drag.active:
                recovery["peak_displacement_deg"] = max(recovery["peak_displacement_deg"], error)
            if recovery["released"]:
                recovery["recovered_frames"] = recovery["recovered_frames"]+1 if error <= 0.25 else 0
                recovery["final_error_deg"] = error

    Path(output).mkdir(parents=True, exist_ok=True)
    (Path(output) / "drag_events.jsonl").unlink(missing_ok=True)
    log = (Path(output) / "worker.log").open("w", encoding="utf-8", buffering=1)
    # Redirect native extension writes too (PyBullet uses C stdout/stderr).
    os.dup2(log.fileno(), 1)
    os.dup2(log.fileno(), 2)
    sys.stdout = log
    sys.stderr = log
    import faulthandler
    faulthandler.enable(log)
    # Crash-only diagnostics. Do not start dump_traceback_later's background
    # stack walker: the user's Windows Python 3.11.15 crashes in python311.dll
    # even with a pure-Python Event polling reproducer. UART JSONL supplies
    # cooperative progress diagnostics without asynchronously walking frames.
    try:
        report = run(
            Path(elf), Path(output), qemu, options, on_sample=push, stop_requested=stop.is_set,
            on_physics=physics if interactive else None
        )
        if interactive:
            recovery.update(drag_started=physical_drag.started, drag_released=physical_drag.released,
                            input_dropped=int(drag_channel.dropped.value), scripted_test=test_drag)
            if test_drag:
                recovery["passed"] = (recovery["peak_displacement_deg"] >= 2.0 and
                    recovery["recovered_frames"] >= 15 and recovery["max_uart_change_deg"] >= 0.2 and
                    physical_drag.started == test_drag_cycles and physical_drag.released == test_drag_cycles and
                    report.get("hold_recoveries", 0) >= test_drag_cycles)
                report["passed"] = bool(report["passed"] and recovery["passed"])
            report["drag_recovery"] = recovery
            (Path(output) / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        if report.get("passed") is not True:
            raise RuntimeError("QEMU/FreeRTOS/UART physics verdict failed")
        print(
            f"[LIVE/CAPTURE] PASS {report['segments_settled']}/"
            f"{report['segments_requested']} segments; "
            f"{report['target_frames']} physics target frames",
            flush=True,
        )
    except Exception:
        # Keep tracebacks in worker.log even if the terminal is paused.
        traceback.print_exc()
        raise
    finally:
        log.flush()


class LiveMeasurements:
    """Incremental diagnostics based only on sampled *actual* joint angles."""

    def __init__(self) -> None:
        self.frames_received = 0
        self.frames_while_worker_alive = 0
        self.segments: set[int] = set()
        self.minimum: list[int] | None = None
        self.maximum: list[int] | None = None
        self.latest: RawFrame | None = None

    def observe(self, frame: RawFrame, worker_alive: bool) -> None:
        stamp, segment, raw = frame
        if (not math.isfinite(stamp) or stamp < 0 or
                not 0 <= segment <= len(SCENARIOS["six_axis"]) or
                len(raw) != len(JOINT_NAMES) or
                any(type(v) is not int for v in raw)):
            raise ValueError("Invalid live measurement from worker")
        self.frames_received += 1
        if worker_alive:
            self.frames_while_worker_alive += 1
        self.segments.add(segment)
        if self.minimum is None:
            self.minimum = list(raw)
            self.maximum = list(raw)
        else:
            for index, value in enumerate(raw):
                self.minimum[index] = min(self.minimum[index], value)
                self.maximum[index] = max(self.maximum[index], value)
        self.latest = frame

    def ranges_deg(self) -> tuple[float, ...]:
        if self.minimum is None or self.maximum is None:
            return ()
        return tuple(
            (high - low) / wire.JOINT_ANGLE_SCALE
            for low, high in zip(self.minimum, self.maximum)
        )


def render_measurement(
    bullet: Any, arm: int, joints: list[int], frame: RawFrame,
    prior_tool_xyz: tuple[float, ...] | None, label_id: int,
) -> tuple[tuple[float, ...], int]:
    """Update GUI kinematics from DIRECT measurements, never recompute PID."""
    sim_time, segment, raw = frame
    for joint, position in zip(joints, raw):
        bullet.resetJointState(
            arm, joint, math.radians(position / wire.JOINT_ANGLE_SCALE)
        )
    bullet.stepSimulation()
    tool = tuple(bullet.getLinkState(
        arm, bullet.getNumJoints(arm) - 1,
        computeForwardKinematics=True,
    )[4])
    if prior_tool_xyz is not None:
        bullet.addUserDebugLine(
            prior_tool_xyz, tool, [0.1, 0.7, 1.0],
            lineWidth=2, lifeTime=10,
        )
    label_id = bullet.addUserDebugText(
        f"LIVE UART | segment {segment}/3 | physics {sim_time:.2f}s | left-drag link; Q exits",
        [-0.65, -0.65, 0.85],
        textColorRGB=[0.1, 0.9, 0.2],
        replaceItemUniqueId=label_id,
    )
    return tool, label_id


def run_live(
    elf: Path, output_dir: Path, qemu: str, scenario: str,
    timeout_s: float, refresh_hz: float, headless_observer: bool,
    hold_window: bool, interactive: bool | None = None,
    hold_seconds: float | None = None, test_drag: bool = False, opengl2: bool = False,
    test_drag_cycles: int = 1,
) -> dict:
    """Show latest physical measurements while the real QEMU loop runs."""
    if not elf.is_file():
        raise FileNotFoundError(f"ARM firmware ELF not found: {elf}")
    if not URDF.is_file():
        raise FileNotFoundError(f"PyBullet UR5 model not found: {URDF}")
    if not 10.0 <= refresh_hz <= 120.0 or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Invalid display refresh or simulation timeout")
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scene: {scenario}")

    if interactive is None:
        interactive = not headless_observer and hold_window
    if not 1 <= test_drag_cycles <= 8:
        raise ValueError("Drag test cycles must be between 1 and 8")
    if test_drag:
        interactive = True
    if hold_seconds is not None and (not math.isfinite(hold_seconds) or hold_seconds <= 0):
        raise ValueError("Hold seconds must be positive")
    if headless_observer and interactive and hold_seconds is None:
        raise ValueError("Headless interactive test needs bounded --hold-seconds")
    p = None
    client = -1
    arm = -1
    joints: list[int] = []
    last_tool: tuple[float, ...] | None = None
    text_id = -1
    measurements = LiveMeasurements()
    if not headless_observer:
        import pybullet as p
        client = p.connect(p.GUI, options="--opengl2" if opengl2 else "")
        if client < 0:
            raise RuntimeError("PyBullet GUI could not open")
        try:
            p.setRealTimeSimulation(0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.setGravity(0, 0, 0)
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            p.setJointMotorControlArray(
                arm, joints, p.VELOCITY_CONTROL, forces=[0.0] * len(joints)
            )
            for idx, raw in zip(joints, START_RAW):
                p.resetJointState(
                    arm, idx, math.radians(raw / wire.JOINT_ANGLE_SCALE)
                )
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5, cameraYaw=50, cameraPitch=-27,
                cameraTargetPosition=[0.0, 0.0, 0.35],
            )
            print("[GUI] LIVE window opened before starting the QEMU physics worker", flush=True)
        except Exception:
            p.disconnect(client)
            raise

    context = mp.get_context("spawn")
    channel = LatestMailbox(context)
    stop = context.Event()
    ready = context.Event()
    drag_channel = DragMailbox(context)
    mouse = MouseDrag(drag_channel, output_dir / "mouse_events.jsonl")
    worker = context.Process(
        target=capture_worker,
        args=(str(elf.resolve()), str(output_dir.resolve()),
              qemu, scenario, timeout_s, channel, stop, drag_channel, ready,
              interactive, hold_seconds, test_drag, test_drag_cycles),
        name="stage5-qemu-uart-pybullet-direct",
    )
    last_visible_update = 0.0
    last_segment = 0
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "observer_result.json").unlink(missing_ok=True)
        (output_dir / "worker_exit.json").unlink(missing_ok=True)
        (output_dir / "mouse_events.jsonl").unlink(missing_ok=True)
        worker.start()
        started = time.monotonic()
        console_at = 0.0
        rendered = 0
        rendered_sequence = 0
        hold_announced = False
        hold_wall = None
        audit = (output_dir / "observer.jsonl").open("w", encoding="utf-8", buffering=1)
        print("[LIVE] QEMU + FreeRTOS + UART + PyBullet DIRECT running now", flush=True)
        while True:
            frame = None
            # Bounded waiting keeps GUI interactive and does not throttle UART.
            try:
                frame = channel.get_nowait()
            except queue.Empty:
                pass
            if frame is not None:
                measurements.observe(frame, worker.is_alive())
                if measurements.latest is not None:
                    segment = measurements.latest[1]
                    if segment > last_segment:
                        print(f"[LIVE] Showing physical segment {segment}/"
                              f"{len(SCENARIOS[scenario])}", flush=True)
                        last_segment = segment
            now = time.monotonic()
            connected = p is not None and p.isConnected(client)
            if p is not None and not connected:
                stop.set()
            if connected:
                keys = p.getKeyboardEvents()
                if keys.get(ord("q"), 0) & p.KEY_WAS_TRIGGERED:
                    stop.set()
                if interactive and not test_drag:
                    mouse.poll(p, arm)
            if ready.is_set() and not hold_announced:
                print("[HOLD] Left-drag a robot link; release to recover. Q or close GUI to exit.", flush=True)
                hold_announced = True
                hold_wall = now
            if (connected and measurements.latest is not None and
                    channel.seen != rendered_sequence and
                    now - last_visible_update >= 1.0 / refresh_hz):
                last_tool, text_id = render_measurement(
                    p, arm, joints, measurements.latest, last_tool, text_id
                )
                last_visible_update = now
                rendered += 1
                rendered_sequence = channel.seen
            deadline = (hold_wall + hold_seconds + 25.0 if hold_wall is not None and hold_seconds is not None
                        else float("inf") if hold_wall is not None and interactive
                        else started + timeout_s + 25.0)
            if now > deadline:
                raise TimeoutError("Worker exceeded wall deadline; inspect worker.log and uart_live.jsonl")
            if measurements.latest is not None and now - console_at >= 1.0:
                status = channel.status
                snapshot = dict(wall_s=now-started, sequence=int(status[0]),
                    received=measurements.frames_received, rendered=rendered,
                    display_dropped=int(status[0])-measurements.frames_received+int(status[17]),
                    publish_busy=int(status[17]), age_s=now-status[18],
                    target_deg=[v / wire.JOINT_ANGLE_SCALE for v in status[9:15]],
                    actual_deg=[v / wire.JOINT_ANGLE_SCALE for v in status[3:9]],
                    feedback_acks=int(status[15]), bad_checksum=int(status[16]))
                audit.write(json.dumps(snapshot) + "\n")
                angles = " | ".join(
                    f"J{i+1} {target:.2f}/{actual:.2f}"
                    for i, (target, actual) in enumerate(zip(
                        snapshot["target_deg"], snapshot["actual_deg"])))
                print(f"[UART LIVE target/actual deg] {angles} | "
                      f"RX={snapshot['sequence']} ACK={snapshot['feedback_acks']} "
                      f"render={rendered} display_drop={snapshot['display_dropped']} "
                      f"checksum_bad={snapshot['bad_checksum']} age={snapshot['age_s']:.3f}s",
                      flush=True)
                console_at = now
            time.sleep(1.0 / refresh_hz)
            if not worker.is_alive():
                # Drain samples published at the end of the control sequence.
                worker.join()
                while True:
                    try:
                        remaining = channel.get_nowait()
                    except queue.Empty:
                        break
                    measurements.observe(remaining, False)
                break

        if worker.exitcode != 0:
            code = worker.exitcode
            native_code = f"0x{code & 0xffffffff:08X}" if code is not None else "unknown"
            (output_dir / "worker_exit.json").write_text(json.dumps(dict(
                passed=False, exit_code=code, exit_hex=native_code,
                last_sample=None if measurements.latest is None else measurements.latest), indent=2), encoding="utf-8")
            raise RuntimeError(
                f"QEMU physics worker failed (exit {code}, {native_code}); "
                "inspect worker.log, uart_live.jsonl and joint_trace.csv"
            )
        report_file = output_dir / "result.json"
        if not report_file.is_file():
            raise RuntimeError("Worker returned without physical validation report")
        result = json.loads(report_file.read_text("utf-8"))
        if (result.get("passed") is not True or
                result.get("segments_settled") != len(SCENARIOS[scenario])):
            raise RuntimeError("Physical QEMU closed-loop did not pass acceptance")
        motion = measurements.ranges_deg()
        print("[LIVE] Measured joint ranges: " + ", ".join(
            f"J{i+1}={angle:.2f}deg" for i, angle in enumerate(motion)
        ), flush=True)
        required_axes = [i for i in range(len(JOINT_NAMES))
                         if any(goal[i] != START_RAW[i] for goal in SCENARIOS[scenario])]
        if (not motion or any(motion[i] < MIN_OBSERVED_MOTION_DEG for i in required_axes) or
                measurements.frames_received < 10):
            raise RuntimeError("Insufficient live physical movement was observed")
        if measurements.frames_while_worker_alive == 0:
            raise RuntimeError("No frame was received while QEMU worker was running")
        print(
            f"[LIVE] PASS: {measurements.frames_received} measurement messages received, "
            f"{measurements.frames_while_worker_alive} received during control",
            flush=True,
        )
        summary = dict(received=measurements.frames_received, rendered=rendered,
            published=channel.seen, publish_busy=int(channel.skipped.value),
            display_dropped=channel.seen-measurements.frames_received+int(channel.skipped.value),
            frames_during_control=measurements.frames_while_worker_alive,
            joint_ranges_deg=motion, headless_observer=headless_observer)
        (output_dir / "observer_result.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return result
    finally:
        stop.set()
        if worker.is_alive():
            worker.join(timeout=5.0)
        if worker.is_alive():
            worker.terminate()
        if worker.pid is not None:
            worker.join(timeout=5.0)
        if "audit" in locals():
            audit.close()
        if p is not None and client >= 0:
            if hold_window and not interactive and worker.exitcode == 0:
                try:
                    input("[GUI] 实时运动结束，按 Enter 关闭窗口...")
                except EOFError:
                    pass
            if p.isConnected(client):
                p.disconnect(client)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--qemu", default="qemu-system-arm")
    parser.add_argument("--scenario", choices=tuple(SCENARIOS), default="six_axis")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "build/stage5-live-qemu-gui")
    parser.add_argument("--timeout-s", type=float, default=210.0)
    parser.add_argument("--refresh-hz", type=float, default=REFRESH_HZ)
    parser.add_argument("--headless-observer", action="store_true",
                        help="CI: receive live frames without an OpenGL window")
    parser.add_argument("--no-hold", action="store_true")
    parser.add_argument("--opengl2", action="store_true", help="Compatible Windows GUI renderer")
    parser.add_argument("--interactive", action="store_true", default=None,
                        help="Keep true UART control active after three segments; GUI defaults to this")
    parser.add_argument("--hold-seconds", type=float, default=None)
    parser.add_argument("--test-drag-cycles", type=int, default=1)
    parser.add_argument("--test-drag-recovery", action="store_true",
                        help="Bounded physical grab/release test through the mouse IPC")
    args = parser.parse_args()
    run_live(args.elf, args.output_dir, args.qemu, args.scenario,
             args.timeout_s, args.refresh_hz, args.headless_observer,
             not args.no_hold, args.interactive, args.hold_seconds, args.test_drag_recovery, args.opengl2, args.test_drag_cycles)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
