"""Live QEMU/FreeRTOS/UART/PyBullet six-axis control and telemetry.

The worker owns ONE PyBullet physics client: GUI (native picking) on desktop,
DIRECT in headless CI. Only measured joint states are fed back to FreeRTOS.
The parent monitors a nonblocking latest-sample mailbox; console output never
pauses simulation. No mirrored robot, mouse IPC or Python PID is required.
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
from stage5_qemu_uart_visual_demo import (
    DIAGNOSTICS_TIMEOUT_S, DemoOptions, SCENARIOS, run,
)

# Observer polling never changes the physics timestep or UART pacing.
REFRESH_HZ = 30.0
MIN_OBSERVED_MOTION_DEG = 2.0
# Test fixture only. Physical motor limits remain in firmware config.
TEST_TORQUE_NM = 300.0
# Stop external torque once actual joint displacement is measurable. Applying
# 300 N*m for the entire 0.8s fixture can spin the joint outside UART range.
TEST_MIN_DISPLACEMENT_DEG = 2.0
TEST_STOP_DISPLACEMENT_DEG = 3.0
TEST_PERIOD_S = 12.0
TEST_FORCE_START_S = 1.0
TEST_FORCE_END_S = 1.8
RawFrame = tuple[float, int, tuple[int, ...]]


class LatestMailbox:
    """Single latest sample; no pipe, feeder thread, queue drain, or blocking lock.

    Producer/consumer acquire the shared lock with blocking=False. A stalled
    observer can lose telemetry samples but never stalls UART or physics.
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
    channel: Any, stop: Any, ready=None, interactive=False,
    hold_seconds=None, test_disturbance=False, disturbance_cycles=1,
    headless=False, opengl2=False,
    diagnostics_timeout_s=DIAGNOSTICS_TIMEOUT_S, observe_hold=False,
) -> None:
    """Run actual ARM/FreeRTOS control and the only PyBullet physics client."""
    options = DemoOptions(
        scenario=scenario, headless=headless, frame_delay_s=0.0,
        timeout_s=timeout_s, hold_window=False,
        interactive_hold=interactive, hold_duration_s=hold_seconds,
        opengl2=opengl2, diagnostics_timeout_s=diagnostics_timeout_s,
        hold_observe_only=observe_hold,
    )
    hold_at = None
    peak_error_deg = 0.0
    injected_steps = [0] * disturbance_cycles
    cycle_peak_displacements_deg = [0.0] * disturbance_cycles
    completed_injections: set[int] = set()

    def physics(bullet, arm, joints, sim_time, done, ending):
        nonlocal hold_at
        if done and hold_at is None:
            hold_at = sim_time
            if ready is not None:
                ready.set()

        if not headless:
            # GUI picking and physical constraints are owned by PyBullet.
            # Only the optional Q-to-exit shortcut is handled here.
            try:
                if not bullet.isConnected():
                    stop.set()
                elif (bullet.getKeyboardEvents().get(ord("q"), 0)
                      & bullet.KEY_WAS_TRIGGERED):
                    stop.set()
            except bullet.error:
                stop.set()

        if test_disturbance and done and not ending:
            # Headless regression uses PyBullet's native external-torque
            # API, not a second implementation of mouse dragging.
            elapsed = sim_time - hold_at
            cycle = int(elapsed // TEST_PERIOD_S)
            phase = elapsed - TEST_PERIOD_S * cycle
            if (cycle < disturbance_cycles and
                    TEST_FORCE_START_S <= phase < TEST_FORCE_END_S and
                    cycle not in completed_injections):
                # Monitor the *same shoulder-lift joint* receiving the
                # external torque. Stop as soon as it leaves the final goal
                # by 3 degrees; don't keep accelerating it for 0.8s.
                # This is an external physics perturbation, not an edited
                # measured angle or a bypass of the production C controller.
                actual_deg = math.degrees(bullet.getJointState(arm, joints[1])[0])
                requested_deg = (SCENARIOS[scenario][-1][1] /
                                 wire.JOINT_ANGLE_SCALE)
                if abs(actual_deg - requested_deg) >= TEST_STOP_DISPLACEMENT_DEG:
                    completed_injections.add(cycle)
                else:
                    axis = bullet.getJointInfo(arm, joints[1])[13]
                    torque = [TEST_TORQUE_NM * component for component in axis]
                    bullet.applyExternalTorque(
                        arm, joints[1], torque, bullet.LINK_FRAME)
                    injected_steps[cycle] += 1

    def publish(stamp, segment, measured, targets, acks, bad):
        nonlocal peak_error_deg
        channel.publish(stamp, segment, measured, targets, acks, bad)
        if hold_at is not None:
            goal = SCENARIOS[scenario][-1]
            peak_error_deg = max(
                peak_error_deg,
                max(abs(a - b) / wire.JOINT_ANGLE_SCALE
                    for a, b in zip(measured, goal)),
            )
            if test_disturbance:
                cycle = int((stamp - hold_at) // TEST_PERIOD_S)
                if 0 <= cycle < disturbance_cycles:
                    joint_error_deg = abs(measured[1] - goal[1]) / wire.JOINT_ANGLE_SCALE
                    cycle_peak_displacements_deg[cycle] = max(
                        cycle_peak_displacements_deg[cycle], joint_error_deg)

    destination = Path(output)
    destination.mkdir(parents=True, exist_ok=True)
    log = (destination / "worker.log").open("w", encoding="utf-8", buffering=1)
    # Include native PyBullet stdout/stderr in the worker crash log.
    os.dup2(log.fileno(), 1)
    os.dup2(log.fileno(), 2)
    sys.stdout = log
    sys.stderr = log
    import faulthandler
    faulthandler.enable(log)
    # Deliberately no asynchronous stack walker: Python 3.11 Windows crash.
    try:
        report = run(
            Path(elf), destination, qemu, options, on_sample=publish,
            stop_requested=stop.is_set, on_physics=physics if interactive else None,
        )
        if test_disturbance:
            regression = {
                "peak_displacement_deg": peak_error_deg,
                "peak_displacement_per_cycle_deg": cycle_peak_displacements_deg,
                "external_torque_substeps_per_cycle": injected_steps,
                "expected_disturbances": disturbance_cycles,
                "hold_recoveries": report.get("hold_recoveries", 0),
            }
            regression["passed"] = (
                all(count > 0 for count in injected_steps)
                and all(peak >= TEST_MIN_DISPLACEMENT_DEG
                        for peak in cycle_peak_displacements_deg)
                and report.get("hold_recoveries", 0) >= disturbance_cycles
            )
            report["disturbance_regression"] = regression
            report["passed"] = bool(report["passed"] and regression["passed"])
            (destination / "result.json").write_text(
                json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        # A passive experiment may legitimately end outside tolerance.
        # Preserve passed=False in result.json, but allow an orderly exit
        # when the UART and firmware fault counters stayed healthy.
        clean_observation = (observe_hold
            and report.get("segments_settled") == report.get("segments_requested")
            and report.get("bad_checksum_frames") == 0
            and report.get("fault_counters")
            and all(value == 0 for value in report["fault_counters"].values()))
        if report.get("passed") is not True and not clean_observation:
            raise RuntimeError("QEMU/FreeRTOS/UART physics verdict failed")
        if clean_observation and report.get("passed") is not True:
            print("[OBSERVE] Finished outside target tolerance; result.json passed=false",
                  flush=True)
        verdict = "PASS" if report.get("passed") is True else "OBSERVED (NOT PASS)"
        print(f"[LIVE] {verdict} {report['segments_settled']}/"
              f"{report['segments_requested']} segments", flush=True)
    except Exception:
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


def run_live(
    elf: Path, output_dir: Path, qemu: str, scenario: str,
    timeout_s: float, refresh_hz: float, headless_observer: bool,
    hold_window: bool, interactive: bool | None = None,
    hold_seconds: float | None = None, test_disturbance: bool = False,
    opengl2: bool = False, disturbance_cycles: int = 1,
    diagnostics_timeout_s: float = DIAGNOSTICS_TIMEOUT_S,
    observe_hold: bool = False,
) -> dict:
    """Observe the physics worker; never create another PyBullet robot."""
    if not elf.is_file():
        raise FileNotFoundError(f"ARM firmware ELF not found: {elf}")
    if not URDF.is_file():
        raise FileNotFoundError(f"PyBullet UR5 model not found: {URDF}")
    if not 10.0 <= refresh_hz <= 120.0 or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Invalid observer frequency or timeout")
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scene: {scenario}")
    if not 1 <= disturbance_cycles <= 8:
        raise ValueError("Disturbance cycles must be between 1 and 8")
    if hold_seconds is not None and (
        not math.isfinite(hold_seconds) or hold_seconds <= 0
    ):
        raise ValueError("Hold seconds must be positive")

    if interactive is None:
        interactive = not headless_observer and hold_window
    if observe_hold and not interactive:
        raise ValueError("--observe-hold needs interactive HOLD; omit --no-hold")
    if test_disturbance:
        if not headless_observer:
            raise ValueError("Disturbance regression requires --headless-observer")
        interactive = True
    if headless_observer and interactive and hold_seconds is None:
        raise ValueError("Headless hold requires --hold-seconds")

    context = mp.get_context("spawn")
    channel = LatestMailbox(context)
    stop = context.Event()
    ready = context.Event()
    worker = context.Process(
        target=capture_worker,
        args=(
            str(elf.resolve()), str(output_dir.resolve()), qemu, scenario,
            timeout_s, channel, stop, ready, interactive, hold_seconds,
            test_disturbance, disturbance_cycles, headless_observer, opengl2,
            diagnostics_timeout_s, observe_hold,
        ),
        name="stage5-qemu-uart-pybullet-physics",
    )
    measurements = LiveMeasurements()
    last_segment = 0
    last_report_at = 0.0
    hold_announced = False
    hold_wall = None
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale in ("result.json", "observer_result.json", "worker_exit.json",
                  "mouse_events.jsonl", "drag_events.jsonl"):
        (output_dir / stale).unlink(missing_ok=True)
    audit = (output_dir / "observer.jsonl").open(
        "w", encoding="utf-8", buffering=1)
    try:
        worker.start()
        started = time.monotonic()
        print("[LIVE] QEMU + FreeRTOS + UART + "
              + ("PyBullet DIRECT" if headless_observer else "PyBullet native GUI")
              + " running", flush=True)
        while True:
            try:
                frame = channel.get_nowait()
            except queue.Empty:
                frame = None
            if frame is not None:
                measurements.observe(frame, worker.is_alive())
                segment = frame[1]
                if segment > last_segment:
                    print(f"[LIVE] Physical segment {segment}/"
                          f"{len(SCENARIOS[scenario])}", flush=True)
                    last_segment = segment

            now = time.monotonic()
            if ready.is_set() and not hold_announced:
                if not headless_observer:
                    print("[HOLD] Native PyBullet: left-drag a link and release; "
                          "press Q or Ctrl+C to exit", flush=True)
                else:
                    print("[HOLD] Headless disturbance observation started", flush=True)
                hold_announced = True
                hold_wall = now

            deadline = (hold_wall + hold_seconds + 25.0
                        if hold_wall is not None and hold_seconds is not None
                        else float("inf")
                        if hold_wall is not None and interactive
                        else started + timeout_s + 25.0)
            if now > deadline:
                raise TimeoutError("Physics worker exceeded deadline; see worker.log")

            if measurements.latest is not None and now - last_report_at >= 1.0:
                status = channel.status
                published = int(status[0])
                sample = dict(
                    wall_s=now - started, sequence=published,
                    received=measurements.frames_received,
                    observer_drop=published - measurements.frames_received
                        + int(status[17]),
                    publish_busy=int(status[17]), age_s=now - status[18],
                    target_deg=[v / wire.JOINT_ANGLE_SCALE for v in status[9:15]],
                    actual_deg=[v / wire.JOINT_ANGLE_SCALE for v in status[3:9]],
                    feedback_acks=int(status[15]), bad_checksum=int(status[16]),
                )
                audit.write(json.dumps(sample) + "\n")
                angles = " | ".join(
                    f"J{i+1} {target:.2f}/{actual:.2f}"
                    for i, (target, actual) in enumerate(
                        zip(sample["target_deg"], sample["actual_deg"]))
                )
                print(
                    f"[UART LIVE target/actual deg] {angles} | "
                    f"RX={published} ACK={sample['feedback_acks']} "
                    f"observer_drop={sample['observer_drop']} "
                    f"checksum_bad={sample['bad_checksum']} "
                    f"age={sample['age_s']:.3f}s",
                    flush=True,
                )
                last_report_at = now

            if not worker.is_alive():
                worker.join()
                try:
                    final_frame = channel.get_nowait()
                    measurements.observe(final_frame, False)
                except queue.Empty:
                    pass
                break
            time.sleep(1.0 / refresh_hz)

        if worker.exitcode != 0:
            code = worker.exitcode
            native_code = (f"0x{code & 0xffffffff:08X}"
                           if code is not None else "unknown")
            (output_dir / "worker_exit.json").write_text(json.dumps({
                "passed": False, "exit_code": code, "exit_hex": native_code,
                "last_sample": measurements.latest,
            }, indent=2), encoding="utf-8")
            raise RuntimeError(
                f"Physics worker exited {code} ({native_code}); "
                "see worker.log and joint_trace.csv"
            )

        report_file = output_dir / "result.json"
        if not report_file.is_file():
            raise RuntimeError("Physics worker returned without a validation report")
        result = json.loads(report_file.read_text(encoding="utf-8"))
        # An observation run is allowed to preserve passed=False without
        # claiming convergence. Real firmware/transport errors are still fatal.
        clean_observation = (observe_hold
            and result.get("bad_checksum_frames") == 0
            and result.get("fault_counters")
            and all(value == 0 for value in result["fault_counters"].values()))
        if (result.get("segments_settled") != len(SCENARIOS[scenario])
                or (result.get("passed") is not True and not clean_observation)):
            raise RuntimeError("QEMU/FreeRTOS closed-loop acceptance failed")

        motion = measurements.ranges_deg()
        required_axes = [
            i for i in range(len(JOINT_NAMES))
            if any(goal[i] != START_RAW[i] for goal in SCENARIOS[scenario])
        ]
        if (not motion
                or any(motion[i] < MIN_OBSERVED_MOTION_DEG for i in required_axes)
                or measurements.frames_received < 10
                or measurements.frames_while_worker_alive == 0):
            raise RuntimeError("Insufficient measured motion in observer stream")

        summary = dict(
            received=measurements.frames_received,
            published=channel.seen,
            publish_busy=int(channel.skipped.value),
            observer_dropped=channel.seen - measurements.frames_received
                + int(channel.skipped.value),
            frames_during_control=measurements.frames_while_worker_alive,
            joint_ranges_deg=motion, headless_observer=headless_observer,
            note="PyBullet GUI renders the authoritative physics directly",
        )
        (output_dir / "observer_result.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
        if observe_hold:
            print(f"[OBSERVE] Final error={result['hold_final_error_deg']:.2f}deg; "
                  f"recovered={result['hold_recoveries']}; "
                  f"control_passed={result['passed']}", flush=True)
        return result
    finally:
        stop.set()
        if worker.is_alive():
            worker.join(timeout=5.0)
        if worker.is_alive():
            worker.terminate()
        if worker.pid is not None:
            worker.join(timeout=5.0)
        audit.close()


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
    parser.add_argument("--diagnostics-timeout-s", type=float,
                        default=DIAGNOSTICS_TIMEOUT_S,
                        help="Finite UART diagnostic response timeout, 5..60 seconds")
    parser.add_argument("--observe-hold", action="store_true",
                        help="Keep native C PID running in HOLD without automatic replans; "
                             "still enforce UART and firmware errors")
    parser.add_argument("--disturbance-cycles", type=int, default=1)
    parser.add_argument("--test-disturbance-recovery", action="store_true",
                        help="Headless native PyBullet external-torque recovery test")
    args = parser.parse_args()
    run_live(args.elf, args.output_dir, args.qemu, args.scenario,
             args.timeout_s, args.refresh_hz, args.headless_observer,
             not args.no_hold, args.interactive, args.hold_seconds,
             args.test_disturbance_recovery, args.opengl2, args.disturbance_cycles,
             args.diagnostics_timeout_s, args.observe_hold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
