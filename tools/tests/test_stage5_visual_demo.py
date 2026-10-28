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
    joint_feedback_frame,
    canonical_physical_feedback,
    closest_physical_motor_targets,
    JointFeedbackRangeError,
    preserve_gui_after_feedback_range_fault,
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

    def test_actual_multi_turn_angles_use_canonical_wire_but_keep_physical_pose(self) -> None:
        physical = (35000, 36100, -19000, 18000, -54000, 0)
        canonical = canonical_physical_feedback(physical)
        self.assertEqual(canonical, (-1000, 100, 17000, -18000, -18000, 0))
        self.assertEqual(physical[0], 35000)  # no physical state mutation
        packet = joint_feedback_frame(canonical)
        self.assertEqual(struct.unpack("<6h", packet[4:-1]), canonical)
        for extreme in (-32769, 32768, 72000, -72000):
            wire_angle = canonical_physical_feedback(
                (extreme, 0, 0, 0, 0, 0))[0]
            self.assertGreaterEqual(wire_angle, -18000)
            self.assertLess(wire_angle, 18000)
            joint_feedback_frame((wire_angle,) + (0,) * 5)

    def test_multiturn_motor_commands_use_nearest_urdf_legal_pose(self) -> None:
        degrees = __import__("math").radians
        limits = (
            (degrees(-360), degrees(360)),
            (degrees(-360), degrees(360)),
            (degrees(-180), degrees(180)),
            (degrees(-360), degrees(360)),
            (degrees(-360), degrees(360)),
            (degrees(-360), degrees(360)),
        )
        command = (-1000, -18000, -17000, 3000, 0, 0)
        physical = (35000, 17900, 17000, 0, 0, 0)
        result = closest_physical_motor_targets(command, physical, limits)
        self.assertAlmostEqual(__import__("math").degrees(result[0]), 350.0)
        self.assertAlmostEqual(__import__("math").degrees(result[1]), 180.0)
        self.assertAlmostEqual(__import__("math").degrees(result[2]), -170.0)
        self.assertAlmostEqual(__import__("math").degrees(result[3]), 30.0)
        # A 390deg candidate is outside a +/-360deg model; never choose it.
        alternate = closest_physical_motor_targets(
            (3000, 0, 0, 0, 0, 0), (26000, 0, 0, 0, 0, 0), limits)
        self.assertAlmostEqual(__import__("math").degrees(alternate[0]), 30.0)
        with self.assertRaisesRegex(ValueError, "joint limits"):
            closest_physical_motor_targets(
                (0,) * 6, (0,) * 6, ((float("nan"), 1),) + limits[1:])

    def test_feedback_signed_int16_boundaries_are_not_changed(self) -> None:
        raw = (-32768, 32767, 0, 0, 0, 0)
        packet = joint_feedback_frame(raw)
        self.assertEqual(struct.unpack("<6h", packet[4:-1]), raw)

    def test_overflow_feedback_identifies_real_joint_without_silent_clamping(self) -> None:
        for raw_value in (-32769, 32768, 41000):
            raw = (0, raw_value, 0, 0, 0, 0)
            with self.subTest(raw_value=raw_value):
                with self.assertRaises(JointFeedbackRangeError) as raised:
                    joint_feedback_frame(raw)
                fault = raised.exception
                self.assertEqual(fault.joint_index, 1)
                self.assertEqual(fault.joint_name, "shoulder_lift_joint")
                self.assertEqual(fault.raw_centideg, raw_value)
                self.assertIn(str(raw_value), str(fault))
        with self.assertRaisesRegex(ValueError, "exactly six"):
            joint_feedback_frame((100,))
        with self.assertRaisesRegex(TypeError, "not an integer"):
            joint_feedback_frame((True, 0, 0, 0, 0, 0))

    def test_gui_fault_stop_terminates_qemu_preserves_physics_without_steps_until_q(self) -> None:
        class FakeProcess:
            def __init__(self):
                self.killed = False
            def poll(self):
                return None if not self.killed else 1
            def terminate(self):
                self.killed = True

        class FakeBullet:
            KEY_WAS_TRIGGERED = 1
            error = RuntimeError
            def __init__(self):
                self.labels = []
                self.polled = 0
            def addUserDebugText(self, text, *args, **kwargs):
                self.labels.append(text)
            def isConnected(self):
                return True
            def getKeyboardEvents(self):
                self.polled += 1
                return {ord("q"): self.KEY_WAS_TRIGGERED}
            def stepSimulation(self):
                raise AssertionError("Unsafe GUI fault state must not advance physics")

        qemu = FakeProcess()
        bullet = FakeBullet()
        fault = JointFeedbackRangeError(0, 35000)
        preserve_gui_after_feedback_range_fault(bullet, qemu, None, fault)
        self.assertTrue(qemu.killed)
        self.assertEqual(bullet.polled, 1)
        self.assertIn("FEEDBACK RANGE FAULT", bullet.labels[0])


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