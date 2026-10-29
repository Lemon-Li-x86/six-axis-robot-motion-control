"""Stage5 GUI motion sequencer regression tests (no QEMU/GUI required)."""
from __future__ import annotations

import csv
import json
import socket
import threading
import struct
import sys
import tempfile
import unittest
from pathlib import Path

# The test file lives in tools/tests and imports the production demonstration
# module from its established location; PyBullet is imported only inside run().
ROOT = Path(__file__).resolve().parents[2]
SIMULATION = ROOT / "simulation" / "pybullet_ur5" / "src"
sys.path.insert(0, str(SIMULATION))

from stage5_qemu_uart_visual_demo import (  # noqa: E402
    MotionSequencer,
    SCENARIOS,
    SETTLE_FRAME_COUNT,
    SIMULATED_STEPS_PER_TARGET,
    PHYSICS_STEP_S,
    FEEDBACK_STEP_S,
    TRACE_HEADER,
    configure_physics_stepping,
    advance_physics_step_batch,
    assess_clock_alignment,
    reject_failed_motion_diagnostic,
    joint_feedback_frame,
    validate_goals,
    write_trace,
)
import qemu_uart_receiver as wire  # noqa: E402
from qemu_clock_gate import QemuClockGate  # noqa: E402



class QmpClockGateTests(unittest.TestCase):
    def run_fake_qmp(self, raise_during_physics: bool = False) -> None:
        client, server = socket.socketpair()
        server.settimeout(3.0)
        observed = []
        worker_errors = []

        def serve():
            state = "running"
            try:
                with server:
                    stream = server.makefile("rb")
                    with stream:
                        for raw in stream:
                            command = json.loads(raw)
                            name = command["execute"]
                            observed.append(name)
                            if name == "stop":
                                state = "paused"
                                server.sendall(b'{"event":"STOP"}\r\n')
                                payload = {}
                            elif name == "cont":
                                state = "running"
                                server.sendall(b'{"event":"RESUME"}\r\n')
                                payload = {}
                            elif name == "query-status":
                                payload = {"status": state, "running": state == "running"}
                            else:
                                raise AssertionError(f"Unexpected QMP command {name}")
                            reply = json.dumps({"return": payload, "id": command["id"]})
                            server.sendall((reply + "\r\n").encode("ascii"))
            except Exception as exc:
                worker_errors.append(exc)

        thread = threading.Thread(target=serve)
        thread.start()
        try:
            with QemuClockGate(client) as gate:
                with gate.paused_for_physics(False):
                    self.assertEqual(gate.pauses, 0)
                with gate.paused_for_physics(True):
                    self.assertEqual(gate.pauses, 1)
                    if raise_during_physics:
                        raise ValueError("PyBullet mock failure")
        finally:
            client.close()
            thread.join(timeout=3.0)
        self.assertFalse(thread.is_alive(), "QMP mock server did not exit")
        self.assertEqual(worker_errors, [])
        self.assertEqual(observed,
                         ["stop", "query-status", "cont", "query-status"])

    def test_clock_gate_pauses_guest_for_physics_only(self):
        self.run_fake_qmp()

    def test_clock_gate_resumes_guest_on_physics_error(self):
        with self.assertRaisesRegex(ValueError, "mock failure"):
            self.run_fake_qmp(raise_during_physics=True)


class FailFastFirmwareSafetyTests(unittest.TestCase):
    def test_good_diagnostics_do_not_abort(self):
        for name in ("planner_faults", "controller_faults",
                     "feedback_stale_count", "last_fault"):
            reject_failed_motion_diagnostic(name, 0)
        reject_failed_motion_diagnostic("trajectory_samples", 130)

    def test_fatal_diagnostics_abort_immediately(self):
        for name in ("planner_faults", "controller_faults",
                     "feedback_stale_count", "last_fault"):
            with self.subTest(name=name):
                with self.assertRaisesRegex(
                        RuntimeError, "Firmware motion safety fault"):
                    reject_failed_motion_diagnostic(name, 1)

    def test_signed_not_ready_is_reported_as_minus_seven(self):
        with self.assertRaisesRegex(RuntimeError, "last_fault=-7"):
            reject_failed_motion_diagnostic("last_fault", 4294967289)


class MotionSequencerTests(unittest.TestCase):
    def test_physics_time_must_match_firmware_trajectory_samples(self) -> None:
        # DIRECT reference: 319 * 20ms ~= 640 * 10ms physical integration.
        ratio, accepted = assess_clock_alignment(319, 640, 0.020)
        self.assertTrue(accepted)
        self.assertAlmostEqual(ratio, 0.996875)
        # Native GUI reference: a visually settled end pose must not
        # conceal ~7.24s of RTOS trajectories vs only ~1.96s of physics.
        ratio, accepted = assess_clock_alignment(362, 196, 0.020)
        self.assertFalse(accepted)
        self.assertGreater(ratio, 3.0)
        self.assertEqual(assess_clock_alignment(None, 100, 0.020),
                         (None, False))

    def test_short_apf_route_total_time_uses_monitor_tick_not_only_planning(self):
        # Real CI #40808180745: 3/3 joints settled, 192 physics frames,
        # 65 trajectory samples during motion and 20 Monitor periods spanning
        # motion + settling + UART diagnostics. No threshold is widened.
        active_ratio, active_aligned = assess_clock_alignment(
            65, 192, 0.020)
        self.assertFalse(active_aligned)
        self.assertAlmostEqual(active_ratio, 1.30/1.92)
        total_ratio, total_aligned = assess_clock_alignment(
            20, 192, 0.100)
        self.assertTrue(total_aligned)
        self.assertAlmostEqual(total_ratio, 2.00/1.92)
        # A broken Monitor timebase still fails.
        self.assertFalse(assess_clock_alignment(20, 80, 0.100)[1])

    def test_gui_native_substeps_preserve_physics_interval_and_reduce_api_calls(self) -> None:
        """Native GUI gets one API call per UART frame, with 1ms internal steps."""
        from unittest.mock import Mock
        bullet = Mock()
        calls, warmup = configure_physics_stepping(bullet, headless=False)
        bullet.setTimeStep.assert_called_once_with(FEEDBACK_STEP_S)
        bullet.setPhysicsEngineParameter.assert_called_once_with(
            numSubSteps=SIMULATED_STEPS_PER_TARGET)
        self.assertEqual(calls, 1)
        self.assertEqual(warmup, 100 // SIMULATED_STEPS_PER_TARGET)
        self.assertAlmostEqual(
            FEEDBACK_STEP_S / SIMULATED_STEPS_PER_TARGET, PHYSICS_STEP_S)

    def test_direct_still_uses_one_millisecond_api_calls(self) -> None:
        from unittest.mock import Mock
        bullet = Mock()
        calls, warmup = configure_physics_stepping(bullet, headless=True)
        bullet.setTimeStep.assert_called_once_with(PHYSICS_STEP_S)
        bullet.setPhysicsEngineParameter.assert_not_called()
        self.assertEqual(calls, SIMULATED_STEPS_PER_TARGET)
        self.assertEqual(warmup, 100)

    def test_direct_physical_disturbance_is_applied_before_every_substep(self) -> None:
        calls = []
        class Bullet:
            def stepSimulation(self):
                calls.append(("step", None))
        def callback(_bullet, _arm, _joints, physical_time, _done, _ending):
            calls.append(("force", physical_time))
        advance_physics_step_batch(
            Bullet(), SIMULATED_STEPS_PER_TARGET, 5.0, callback,
            object(), (0, 1, 2, 3, 4, 5), True, False,
        )
        self.assertEqual(len(calls), 2 * SIMULATED_STEPS_PER_TARGET)
        self.assertEqual([what for what, _ in calls],
                         ["force", "step"] * SIMULATED_STEPS_PER_TARGET)
        times = [stamp for what, stamp in calls if what == "force"]
        for index, stamp in enumerate(times):
            self.assertAlmostEqual(stamp, 5.0 + index * PHYSICS_STEP_S)

    def test_native_gui_keeps_one_callback_per_physics_feedback(self) -> None:
        calls = []
        class Bullet:
            def stepSimulation(self):
                calls.append("step")
        def callback(*_):
            calls.append("force")
        advance_physics_step_batch(
            Bullet(), 1, 2.0, callback, 0, (1,), True, False,
        )
        self.assertEqual(calls, ["force", "step"])


    def test_three_segments_advance_only_after_ack_and_feedback(self) -> None:
        goals = SCENARIOS["six_axis"]
        runner = MotionSequencer(goals)
        for index, goal in enumerate(goals):
            request = runner.next_request(now=float(index))
            self.assertEqual(request[2], wire.CMD_REQUEST_JOINT_MOVE)
            self.assertEqual(request[4], 41 + index)
            self.assertFalse(runner.observe(goal))  # ACK only = admission.
            runner.acknowledge(struct.pack("<Bb", 41 + index, 0))
            for _ in range(SETTLE_FRAME_COUNT - 1):
                self.assertFalse(runner.observe(goal))
            self.assertTrue(runner.observe(goal))
            self.assertEqual(runner.completed, index + 1)
        self.assertTrue(runner.done)
        self.assertEqual(len(runner.segment_final_errors_deg), 3)

    def test_pending_request_cannot_be_overwritten(self) -> None:
        runner = MotionSequencer(SCENARIOS["joint_sweep"])
        runner.next_request(now=0.0)
        with self.assertRaisesRegex(RuntimeError, "Previous motion"):
            runner.next_request(now=0.1)

    def test_wrong_ack_sequence_or_failure_is_rejected(self) -> None:
        runner = MotionSequencer(SCENARIOS["joint_sweep"])
        runner.next_request(now=0.0)
        with self.assertRaisesRegex(RuntimeError, "sequence"):
            runner.acknowledge(struct.pack("<Bb", 42, 0))
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            runner.acknowledge(struct.pack("<Bb", 41, -4))

    def test_outside_settle_tolerance_resets_count(self) -> None:
        runner = MotionSequencer((SCENARIOS["six_axis"][0],))
        runner.next_request(now=0.0)
        runner.acknowledge(struct.pack("<Bb", 41, 0))
        target = runner.goal
        assert target is not None
        for _ in range(SETTLE_FRAME_COUNT - 2):
            runner.observe(target)
        outside = (target[0] + 50,) + target[1:]
        self.assertFalse(runner.observe(outside))
        self.assertEqual(runner.settled_frames, 0)

    def test_rejects_non_six_joint_or_invalid_wire_values(self) -> None:
        for goal in ((0,) * 5, (0,) * 7, (32768,) + (0,) * 5,
                     (True,) + (0,) * 5):
            with self.subTest(goal=goal):
                with self.assertRaises(ValueError):
                    validate_goals((goal,))
        with self.assertRaises(ValueError):
            validate_goals(())

    def test_feedback_frame_round_trip(self) -> None:
        raw = (3000, -4500, 6000, 2000, 500, -1000)
        packet = joint_feedback_frame(raw)
        self.assertEqual(packet[:2], wire.FRAME_HEADER)
        self.assertEqual(packet[2], wire.CMD_JOINT_STATE)
        self.assertEqual(packet[3], wire.JOINT_PAYLOAD_LEN)
        self.assertEqual(struct.unpack("<6h", packet[4:-1]), raw)
        self.assertEqual(packet[-1], sum(packet[2:-1]) & 0xFF)

    def test_csv_has_explicit_units_and_all_six_axes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            write_trace(Path(temporary), [])
            with (Path(temporary) / "joint_trace.csv").open(
                encoding="utf-8", newline=""
            ) as file:
                self.assertEqual(next(csv.reader(file)), list(TRACE_HEADER))
            self.assertEqual(len(TRACE_HEADER), 5 + 3 * wire.JOINT_COUNT)


if __name__ == "__main__":
    unittest.main()