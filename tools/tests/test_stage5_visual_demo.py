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
    configure_native_urdf_joint_stops,
    enforce_gui_joint_hard_stops,
    GUI_JOINT_STOP_NUMERICAL_SLOP_DEG,
    GUI_JOINT_STOP_FORCE_MULTIPLIER,
    advance_physics_step_batch,
    assess_clock_alignment,
    joint_feedback_frame,
    canonical_physical_feedback,
    closest_physical_motor_targets,
    continuous_firmware_servo_targets,
    joint_motor_efforts_from_urdf,
    joint_motor_velocity_caps,
    hold_continuous_goal_raw,
    hold_replan_allowed,
    HOLD_STAGNANT_PERIOD_S,
    HOLD_RETRY_COOLDOWN_S,
    HOLD_MAX_REPLAN_ATTEMPTS,
    JointFeedbackRangeError,
    preserve_gui_after_physics_fault,
    validate_physical_feedback_step,
    JointPhysicalSafetyError,
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

    def test_hold_clock_uses_continuous_monitor_ticks_not_planner_samples(self) -> None:
        # Actual failed QEMU log: HOLD includes physical time when no new
        # trajectory reference is produced. The planning-only clock cannot
        # serve as total elapsed time. Firmware NOT_READY remains a failure.
        planning_ratio, planning_passed = assess_clock_alignment(951, 3862, 0.020)
        self.assertFalse(planning_passed)
        self.assertAlmostEqual(planning_ratio, 19.02 / 38.62)
        monitor_ratio, monitor_passed = assess_clock_alignment(394, 3862, 0.100)
        self.assertTrue(monitor_passed)
        self.assertAlmostEqual(monitor_ratio, 39.4 / 38.62)
        # Even monitor ticks must be rejected when physical clock diverges.
        self.assertFalse(assess_clock_alignment(394, 1300, 0.100)[1])

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

    def test_continuous_servo_target_does_not_flip_with_unrelated_axis_crossing(self):
        import math
        rad = math.radians
        limits = (
            (rad(-360), rad(360)),
            (rad(-360), rad(360)),
            (rad(-180), rad(180)),
            (rad(-360), rad(360)),
            (rad(-360), rad(360)),
            (rad(-360), rad(360)),
        )
        previous = tuple(map(rad, (179, 20, 0, 10, 0, 0)))
        # Measured axis 1 crosses 180; an independent axis is mouse-dragged
        # near +350. The C command for that axis must NOT flip by 360.
        measured = (18100, 35000, 0, 1000, 0, 0)
        command = (-17900, 2000, 0, 1000, 0, 0)
        result = continuous_firmware_servo_targets(
            command, measured, previous, limits, rebase=False)
        self.assertAlmostEqual(math.degrees(result[0]), 181.0)
        self.assertAlmostEqual(math.degrees(result[1]), 20.0)
        self.assertAlmostEqual(math.degrees(result[3]), 10.0)
        # The same canonical target stays on the previous continuous turn
        # even if the physical arm is subsequently moved toward -350.
        result2 = continuous_firmware_servo_targets(
            command, (-35000, -35000, 0, 0, 0, 0),
            result, limits, rebase=False)
        self.assertEqual(tuple(round(math.degrees(v)) for v in result2),
                         (181, 20, 0, 10, 0, 0))

    def test_only_accepted_new_generation_rebases_servo_turn_to_feedback(self):
        import math
        rad = math.radians
        limits = ((rad(-360), rad(360)),) * 6
        previous = (rad(40),) + (0.0,) * 5
        command = (-1000,) + (0,) * 5
        measured = (35000,) + (0,) * 5
        old_generation = continuous_firmware_servo_targets(
            command, measured, previous, limits, rebase=False)
        new_generation = continuous_firmware_servo_targets(
            command, measured, previous, limits, rebase=True)
        self.assertAlmostEqual(math.degrees(old_generation[0]), -10.0)
        self.assertAlmostEqual(math.degrees(new_generation[0]), 350.0)
        # Subsequent commands unwind continuously from the newly rebased
        # physical turn rather than jumping to canonical -9deg.
        next_command = (-900,) + (0,) * 5
        next_target = continuous_firmware_servo_targets(
            next_command, measured, new_generation, limits, rebase=False)
        self.assertAlmostEqual(math.degrees(next_target[0]), 351.0)

    def test_motor_torque_caps_match_loaded_urdf_per_axis(self):
        class FakeBullet:
            JOINT_REVOLUTE = 0
            def getJointInfo(self, body, index):
                self.last_body = body
                info = [None] * 11
                info[2] = self.JOINT_REVOLUTE
                info[10] = (150, 150, 150, 28, 28, 28)[index]
                return info
        bullet = FakeBullet()
        self.assertEqual(joint_motor_efforts_from_urdf(
            bullet, 7, tuple(range(6))),
            (150, 150, 150, 28, 28, 28))
        self.assertEqual(bullet.last_body, 7)
        with self.assertRaises(ValueError):
            joint_motor_efforts_from_urdf(bullet, 7, tuple(range(5)))

    def test_hold_does_not_replan_while_C_motor_targets_are_still_moving(self):
        import math
        quiet = math.ceil(HOLD_STAGNANT_PERIOD_S / FEEDBACK_STEP_S)
        cooldown = math.ceil(HOLD_RETRY_COOLDOWN_S / FEEDBACK_STEP_S)
        # Physical arm stalled under native mouse picking while the C motor
        # continues sending new targets: it is not evidence of planner stop.
        self.assertFalse(hold_replan_allowed(
            measured_quiet_frames=quiet + 200,
            command_quiet_frames=0, attempts=1,
            frames_since_last_request=cooldown + 100))
        # A 15deg/s legal unwind can take tens of seconds; never interrupt it
        # because a single joint is steady.
        self.assertFalse(hold_replan_allowed(
            measured_quiet_frames=quiet + 200,
            command_quiet_frames=quiet - 1, attempts=0,
            frames_since_last_request=cooldown + 100))

    def test_hold_stopped_C_output_is_rearmed_with_cooldown_and_finite_budget(self):
        import math
        quiet = math.ceil(HOLD_STAGNANT_PERIOD_S / FEEDBACK_STEP_S)
        cooldown = math.ceil(HOLD_RETRY_COOLDOWN_S / FEEDBACK_STEP_S)
        self.assertTrue(hold_replan_allowed(quiet, quiet, 0, 0))
        # While the mouse is still pushing past the mechanical joint stop,
        # prevent consuming every retry before the user releases the arm.
        self.assertFalse(hold_replan_allowed(
            quiet, quiet, 0, 0, frames_since_last_gui_stop=0))
        self.assertFalse(hold_replan_allowed(
            quiet, quiet, 1, cooldown,
            frames_since_last_gui_stop=quiet - 1))
        self.assertTrue(hold_replan_allowed(
            quiet, quiet, 1, cooldown,
            frames_since_last_gui_stop=quiet))
        # ACK admitted the original recovery request but C may later cancel
        # the trajectory; do not leave the arm stuck forever at that command.
        self.assertFalse(hold_replan_allowed(quiet, quiet, 1, cooldown - 1))
        self.assertTrue(hold_replan_allowed(quiet, quiet, 1, cooldown))
        self.assertTrue(hold_replan_allowed(quiet, quiet, 2, cooldown))
        # Persistent fault remains observable in the GUI, but no unbounded
        # rapid replan loop and no false PASS.
        self.assertFalse(hold_replan_allowed(
            quiet * 20, quiet * 20, HOLD_MAX_REPLAN_ATTEMPTS,
            cooldown * 20))
        self.assertFalse(hold_replan_allowed(-1, quiet, 0, cooldown))

    def test_replan_hold_compares_to_continuous_c_goal_not_canonical_goal(self):
        import math
        rad = math.radians
        limits = ((rad(-360), rad(360)),) * 6
        # The raw 366.76-degree "error" reported by the old HOLD code:
        # +30deg request near -336.76deg physical is unwrapped to -330deg
        # by firmware motion_limits_unwrap_nearest.
        original = (3000,) + (0,) * 5
        physical = (-33676,) + (0,) * 5
        continuous = hold_continuous_goal_raw(original, physical, limits)
        self.assertEqual(continuous[0], -33000)
        self.assertAlmostEqual(abs(physical[0] - continuous[0]) / 100, 6.76)
        self.assertAlmostEqual(abs(physical[0] - original[0]) / 100, 366.76)
        # The opposite direction is not an arbitrary modulo-only equivalence:
        # +390deg is illegal for a [+/-360deg] model. C must unwind to +30.
        positive = hold_continuous_goal_raw(
            original, (35000,) + (0,) * 5, limits)
        self.assertEqual(positive[0], 3000)
        # For joints without a full revolution of travel there is only
        # the canonical pose, never an invented extra turn.
        narrow = ((rad(-180), rad(180)),) + limits[1:]
        self.assertEqual(
            hold_continuous_goal_raw(
                (-17000,) + (0,) * 5, (17000,) + (0,) * 5,
                narrow)[0], -17000)

    def test_motor_velocity_caps_use_firmware_settings_and_urdf_bounds(self):
        import math
        class Bullet:
            def getJointInfo(self, body, index):
                info = [None] * 12
                info[11] = [3.15, 3.15, 3.15, 3.2, 3.2, 3.2][index]
                return info
        caps = joint_motor_velocity_caps(
            Bullet(), 9, tuple(range(6)), [15] * 6)
        for cap in caps:
            self.assertAlmostEqual(cap, math.radians(15))
        smaller = joint_motor_velocity_caps(
            Bullet(), 9, tuple(range(6)), [15, 15, 15, 200, 200, 200])
        self.assertAlmostEqual(smaller[0], math.radians(15))
        self.assertAlmostEqual(smaller[-1], 3.2)
        with self.assertRaises(ValueError):
            joint_motor_velocity_caps(
                Bullet(), 9, tuple(range(6)), [0] * 6)

    def test_gui_stop_clamps_only_out_of_urdf_joint_with_velocity_zero(self):
        import math
        class Bullet:
            def __init__(self):
                self.positions = [
                    math.radians(361), math.radians(-30),
                    math.radians(181), 0, 0, math.radians(-361)]
                self.velocities = [2.0] * 6
                self.resets = []
            def getJointStates(self, arm, joints):
                self.assert_arm = arm
                self.joints = joints
                return [(q, v, (), 0) for q, v in
                        zip(self.positions, self.velocities)]
            def resetJointState(self, arm, idx, targetValue, targetVelocity):
                self.resets.append((arm, idx, targetValue, targetVelocity))
                self.positions[idx] = targetValue
                self.velocities[idx] = targetVelocity
        limits = tuple(
            (math.radians(-180 if i == 2 else -360),
             math.radians(180 if i == 2 else 360))
            for i in range(6))
        bullet = Bullet()
        contacts = enforce_gui_joint_hard_stops(
            bullet, 12, list(range(6)), limits)
        self.assertEqual([c["joint_index"] for c in contacts], [0, 2, 5])
        self.assertEqual(len(bullet.resets), 3)
        self.assertEqual([x[3] for x in bullet.resets], [0.0] * 3)
        self.assertAlmostEqual(math.degrees(bullet.positions[0]), 360)
        self.assertAlmostEqual(math.degrees(bullet.positions[2]), 180)
        self.assertAlmostEqual(math.degrees(bullet.positions[5]), -360)
        self.assertEqual(bullet.velocities[1], 2.0)
        self.assertEqual(enforce_gui_joint_hard_stops(
            bullet, 12, list(range(6)), limits), [])

    def test_gui_stop_guard_rejects_invalid_states_instead_of_encoding_them(self):
        import math
        class InvalidBullet:
            def getJointStates(self, _arm, _joints):
                return [(float("nan"), 0, (), 0)] + [(0, 0, (), 0)] * 5
            def resetJointState(self, *_args, **_kwargs):
                raise AssertionError("must not reset NaN to hide a fault")
        with self.assertRaises(JointPhysicalSafetyError) as error:
            enforce_gui_joint_hard_stops(
                InvalidBullet(), 0, tuple(range(6)),
                tuple((math.radians(-360), math.radians(360))
                      for _ in range(6)))
        self.assertEqual(error.exception.fault_type,
                         "nonfinite_physical_joint_state")

    def test_bullet_native_limit_configuration_uses_only_urdf_joint_bounds(self):
        import math
        class NativeBullet:
            JOINT_REVOLUTE = 0
            def __init__(self):
                self.configured = []
            def getJointInfo(self, body, index):
                assert body == 7
                info = [None] * 11
                info[2] = self.JOINT_REVOLUTE
                info[8] = math.radians(-180 if index == 2 else -360)
                info[9] = math.radians(180 if index == 2 else 360)
                info[10] = 28.0 if index >= 3 else 150.0
                return info
            def changeDynamics(self, body, index, **kwargs):
                self.configured.append((body, index, kwargs))
            def resetJointState(self, *args, **kwargs):
                raise AssertionError("No synthetic pose correction allowed")
        physics = NativeBullet()
        limits = configure_native_urdf_joint_stops(
            physics, 7, tuple(range(6)))
        self.assertEqual(len(limits), 6)
        self.assertEqual(len(physics.configured), 6)
        self.assertAlmostEqual(math.degrees(limits[2][1]), 180.0)
        self.assertAlmostEqual(math.degrees(limits[0][1]), 360.0)
        for _, i, values in physics.configured:
            self.assertEqual(values["jointLowerLimit"], limits[i][0])
            self.assertEqual(values["jointUpperLimit"], limits[i][1])
            self.assertAlmostEqual(
                values["jointLimitForce"],
                (28.0 if i >= 3 else 150.0) * GUI_JOINT_STOP_FORCE_MULTIPLIER)

    def test_gui_solver_tolerance_only_allows_tiny_penetration_not_real_overtravel(self):
        import math
        model = ((math.radians(-360), math.radians(360)),) * 6
        previous = (35995,) + (0,) * 5
        actual = (36005,) + (0,) * 5  # 0.05deg numerical solver penetration
        validate_physical_feedback_step(
            actual, previous, model,
            numerical_slop_deg=GUI_JOINT_STOP_NUMERICAL_SLOP_DEG)
        with self.assertRaises(JointPhysicalSafetyError):
            validate_physical_feedback_step(actual, previous, model)
        with self.assertRaises(JointPhysicalSafetyError):
            validate_physical_feedback_step(
                (36100,) + (0,) * 5, previous, model,
                numerical_slop_deg=GUI_JOINT_STOP_NUMERICAL_SLOP_DEG)
        with self.assertRaises(ValueError):
            validate_physical_feedback_step(
                actual, previous, model, numerical_slop_deg=1.0)

    def test_physical_continuity_allows_crossing_180_and_327_degree_boundaries(self):
        import math
        limits = (
            (math.radians(-360), math.radians(360)),
        ) * 6
        before = (17900, 32600, 35000, 0, 0, 0)
        after = (18100, 32900, 35500, 0, 0, 0)
        validate_physical_feedback_step(after, before, limits)
        self.assertEqual(
            canonical_physical_feedback(after)[:3],
            (-17900, -3100, -500),
        )

    def test_physical_limit_and_unresolvable_wrap_stop_without_fake_feedback(self):
        import math
        limits = (
            (math.radians(-360), math.radians(360)),
            (math.radians(-360), math.radians(360)),
            (math.radians(-180), math.radians(180)),
            (math.radians(-360), math.radians(360)),
            (math.radians(-360), math.radians(360)),
            (math.radians(-360), math.radians(360)),
        )
        with self.assertRaises(JointPhysicalSafetyError) as overtravel:
            validate_physical_feedback_step(
                (36100, 0, 0, 0, 0, 0), (35900, 0, 0, 0, 0, 0), limits)
        self.assertEqual(overtravel.exception.fault_type,
                         "urdf_joint_travel_exceeded")
        self.assertEqual(overtravel.exception.joint_name, "shoulder_pan_joint")
        with self.assertRaises(JointPhysicalSafetyError) as aliased:
            validate_physical_feedback_step(
                (20000, 0, 0, 0, 0, 0), (0,) * 6, limits)
        self.assertEqual(aliased.exception.fault_type,
                         "ambiguous_multi_turn_feedback")
        self.assertIn("one UART physical sample", str(aliased.exception))

    def test_feedback_follows_firmware_canonical_not_just_signed_int16(self) -> None:
        raw = (-18000, 17999, 0, 0, 0, 0)
        packet = joint_feedback_frame(raw)
        self.assertEqual(struct.unpack("<6h", packet[4:-1]), raw)
        # Signed-int16 alone would accept these, but C rejects them.
        for invalid in (-32768, -18001, 18000, 32767):
            with self.subTest(raw=invalid):
                with self.assertRaises(JointFeedbackRangeError) as raised:
                    joint_feedback_frame((invalid,) + (0,) * 5)
                self.assertEqual(raised.exception.fault_type,
                                 "noncanonical_uart_feedback")

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
        preserve_gui_after_physics_fault(bullet, qemu, None, fault)
        self.assertTrue(qemu.killed)
        self.assertEqual(bullet.polled, 1)
        self.assertIn("PHYSICAL FEEDBACK FAULT", bullet.labels[0])


    def test_csv_has_explicit_units_and_all_six_axes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            write_trace(Path(temporary), [])
            with (Path(temporary) / "joint_trace.csv").open(
                encoding="utf-8", newline=""
            ) as file:
                self.assertEqual(next(csv.reader(file)), list(TRACE_HEADER))
            self.assertEqual(len(TRACE_HEADER), 5 + 4 * wire.JOINT_COUNT)
            self.assertIn("servo_target_shoulder_pan_joint_deg", TRACE_HEADER)


if __name__ == "__main__":
    unittest.main()