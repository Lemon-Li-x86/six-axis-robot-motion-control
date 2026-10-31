"""Stage5 live-measurement transport tests, independent of GUI and QEMU."""
from __future__ import annotations

import queue
import math
import multiprocessing as mp
import time
import sys
import unittest
import io
import tempfile
from unittest.mock import Mock
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/pybullet_ur5/src"))

from stage5_qemu_uart_closed_loop import BufferedUART
from stage5_qemu_uart_live_gui import (  # noqa: E402
    LiveMeasurements, LatestMailbox, capture_worker, required_motion_axes,
    clean_safe_jog_cancellation,
    gui_safe_jog_worker_stalled,
)


def produce_without_reader(channel):
    for i in range(10000):
        channel.publish(i / 100, 1, (i,) * 6, (0,) * 6, i, 0)


class SafeGuiJogCancelVerdictTests(unittest.TestCase):
    def valid_cancel(self):
        return dict(
            passed=False, cancelled=True,
            cancel_reason="no_guarded_jog_requested",
            gui_safe_jog=True, safe_gui_jog_applied=0,
            safe_gui_jog_completed=0, hold_recoveries=0,
            segments_requested=4, segments_settled=4,
            bad_checksum_frames=0, clock_alignment_passed=True,
            fault_counters=dict(controller_faults=0, planner_faults=0,
                                feedback_stale_count=0),
        )

    def test_clean_cancel_is_orderly_but_not_pass(self):
        report = self.valid_cancel()
        self.assertTrue(clean_safe_jog_cancellation(report, True))
        self.assertFalse(report["passed"])
        self.assertFalse(clean_safe_jog_cancellation(report, False))

    def test_active_jog_or_fault_never_masquerades_as_cancel(self):
        for key, value in (
            ("safe_gui_jog_applied", 1),
            ("safe_gui_jog_completed", 1),
            ("segments_settled", 3),
            ("bad_checksum_frames", 1),
            ("clock_alignment_passed", False),
            ("passed", True),
        ):
            with self.subTest(key=key):
                report = self.valid_cancel()
                report[key] = value
                self.assertFalse(clean_safe_jog_cancellation(report, True))
        report = self.valid_cancel()
        report["fault_counters"]["controller_faults"] = 1
        self.assertFalse(clean_safe_jog_cancellation(report, True))


class GuiSafeJogWorkerWatchdogTests(unittest.TestCase):
    def test_interactive_idle_hold_does_not_timeout_while_frames_continue(self):
        self.assertFalse(gui_safe_jog_worker_stalled(
            102.0, 101.9, 30.0, ready=True, gui_safe_jog=True))

    def test_missing_samples_has_bounded_parent_failure(self):
        self.assertFalse(gui_safe_jog_worker_stalled(
            137.9, 100.0, 30.0, ready=True, gui_safe_jog=True))
        self.assertTrue(gui_safe_jog_worker_stalled(
            138.1, 100.0, 30.0, ready=True, gui_safe_jog=True))

    def test_non_jog_modes_and_startup_do_not_change(self):
        self.assertFalse(gui_safe_jog_worker_stalled(
            999.0, 0.0, 30.0, ready=True, gui_safe_jog=False))
        self.assertFalse(gui_safe_jog_worker_stalled(
            999.0, 0.0, 30.0, ready=False, gui_safe_jog=True))


class LiveTransportTests(unittest.TestCase):
    def test_worker_uses_crash_only_diagnostics_without_native_watchdog_thread(self):
        report = dict(passed=True, segments_settled=3, segments_requested=3, target_frames=600)
        with (tempfile.TemporaryDirectory() as output,
              patch("stage5_qemu_uart_live_gui.os.dup2"),
              patch("stage5_qemu_uart_live_gui.sys.stdout", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.sys.stderr", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.run", return_value=report),
              patch("faulthandler.enable") as enable,
              patch("faulthandler.dump_traceback_later") as timer,
              patch("faulthandler.cancel_dump_traceback_later") as cancel):
            capture_worker("unused.elf", output, "unused-qemu", "six_axis", 60, Mock(), Mock())
            enable.assert_called_once()
            timer.assert_not_called()
            cancel.assert_not_called()
            # The mocked diagnostic function retains the stream in call_args.
            enable.call_args.args[0].close()

    def test_latest_sample_overwrites_without_consumer(self):
        channel = LatestMailbox(mp.get_context("spawn"))
        for i in range(10000):
            channel.publish(i / 100, 1, (i,) * 6, (123,) * 6, i, 0)
        self.assertEqual(channel.get_nowait(), (99.99, 1, (9999,) * 6))
        self.assertEqual(channel.status[0], 10000)
        self.assertEqual(channel.status[9:15], (123,) * 6)
        with self.assertRaises(queue.Empty):
            channel.get_nowait()

    def test_stalled_reader_cannot_block_producer(self):
        channel = LatestMailbox(mp.get_context("spawn"))
        channel.lock.acquire()
        try:
            started = time.perf_counter()
            channel.publish(1, 1, (100,) * 6, (200,) * 6, 1, 0)
            self.assertLess(time.perf_counter() - started, 0.1)
            self.assertEqual(channel.skipped.value, 1)
            with self.assertRaises(queue.Empty):
                channel.get_nowait()
        finally:
            channel.lock.release()
        channel.publish(2, 1, (300,) * 6, (400,) * 6, 2, 0)
        self.assertEqual(channel.get_nowait()[2], (300,) * 6)

    def test_spawned_producer_exits_without_reader_or_queue_feeder(self):
        context = mp.get_context("spawn")
        channel = LatestMailbox(context)
        worker = context.Process(target=produce_without_reader, args=(channel,))
        worker.start()
        worker.join(timeout=10)
        try:
            self.assertFalse(worker.is_alive())
            self.assertEqual(worker.exitcode, 0)
            self.assertEqual(channel.get_nowait()[2], (9999,) * 6)
        finally:
            if worker.is_alive():
                worker.terminate()
                worker.join()

    def test_worker_chooses_native_gui_physics_and_preserves_options(self):
        """The GUI belongs to the worker, not a mirrored observer robot."""
        captured = {}
        def fake_run(elf, output, qemu, options, **kwargs):
            captured["options"] = options
            self.assertIsNotNone(kwargs["on_physics"])
            return dict(passed=True, segments_settled=3,
                        segments_requested=3, target_frames=600)

        with (tempfile.TemporaryDirectory() as output,
              patch("stage5_qemu_uart_live_gui.os.dup2"),
              patch("stage5_qemu_uart_live_gui.sys.stdout", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.sys.stderr", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.run", side_effect=fake_run),
              patch("faulthandler.enable") as enable):
            capture_worker("test.elf", output, "qemu", "six_axis", 60,
                           Mock(), Mock(), interactive=True, opengl2=True)
            enable.call_args.args[0].close()
        self.assertFalse(captured["options"].headless)
        self.assertTrue(captured["options"].opengl2)
        self.assertTrue(captured["options"].interactive_hold)

    def test_passive_hold_preserves_failed_convergence_without_aborting(self):
        """Observation completion is distinct from physical PASS."""
        report = dict(passed=False, segments_settled=3, segments_requested=3,
                      bad_checksum_frames=0, hold_final_error_deg=17.0,
                      fault_counters=dict(controller_faults=0, planner_faults=0,
                                          feedback_stale_count=0))
        observed = []
        def fake_run(elf, output, qemu, options, **kwargs):
            observed.append(options)
            return report

        with (tempfile.TemporaryDirectory() as output,
              patch("stage5_qemu_uart_live_gui.os.dup2"),
              patch("stage5_qemu_uart_live_gui.sys.stdout", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.sys.stderr", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.run", side_effect=fake_run),
              patch("faulthandler.enable") as enable):
            capture_worker("unused.elf", output, "qemu", "six_axis", 60,
                           Mock(), Mock(), interactive=True,
                           observe_hold=True, diagnostics_timeout_s=30.0)
            enable.call_args.args[0].close()
        self.assertEqual(observed[0].diagnostics_timeout_s, 30.0)
        self.assertTrue(observed[0].hold_observe_only)
        self.assertFalse(report["passed"])

    def test_passive_hold_never_ignores_firmware_faults(self):
        """Observation mode must not swallow real RTOS control failures."""
        report = dict(passed=False, segments_settled=3, segments_requested=3,
                      bad_checksum_frames=0,
                      fault_counters=dict(controller_faults=1, planner_faults=0))
        with (tempfile.TemporaryDirectory() as output,
              patch("stage5_qemu_uart_live_gui.os.dup2"),
              patch("stage5_qemu_uart_live_gui.sys.stdout", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.sys.stderr", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.run", return_value=report),
              patch("faulthandler.enable") as enable):
            with self.assertRaisesRegex(RuntimeError, "physics verdict failed"):
                capture_worker("unused.elf", output, "qemu", "six_axis", 60,
                               Mock(), Mock(), interactive=True,
                               observe_hold=True, diagnostics_timeout_s=30.0)
            enable.call_args.args[0].close()

    def test_headless_disturbance_uses_pybullet_torque_not_mouse_ipc(self):
        """A repeatable software force tests recovery without synthetic picking."""
        class Bullet:
            LINK_FRAME = 2
            def __init__(self):
                self.applied = []
                self.joint_angle_deg = -45.0
            def getJointState(self, body, joint):
                return (math.radians(self.joint_angle_deg), 0.0)
            def getJointInfo(self, body, joint):
                return (None,) * 13 + ((0.0, 1.0, 0.0),)
            def applyExternalTorque(self, *args):
                self.applied.append(args)

        bullet = Bullet()
        def fake_run(elf, output, qemu, options, **kwargs):
            self.assertTrue(options.headless)
            kwargs["on_physics"](bullet, 0, [0, 1, 2, 3, 4, 5], 10.0, True, False)
            kwargs["on_physics"](bullet, 0, [0, 1, 2, 3, 4, 5], 11.1, True, False)
            bullet.joint_angle_deg = -41.0
            # The measured shoulder-lift is already 4deg off goal.
            # Further torque must stop for the remainder of this cycle.
            kwargs["on_physics"](bullet, 0, [0, 1, 2, 3, 4, 5], 11.2, True, False)
            kwargs["on_physics"](bullet, 0, [0, 1, 2, 3, 4, 5], 11.3, True, False)
            kwargs["on_sample"](11.2, 3,
                (3000, -4100, 6000, 2000, 500, -1000),
                (3000, -4500, 6000, 2000, 500, -1000), 50, 0)
            return dict(passed=True, segments_settled=3,
                        segments_requested=3, target_frames=600,
                        hold_recoveries=1)

        with (tempfile.TemporaryDirectory() as output,
              patch("stage5_qemu_uart_live_gui.os.dup2"),
              patch("stage5_qemu_uart_live_gui.sys.stdout", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.sys.stderr", io.StringIO()),
              patch("stage5_qemu_uart_live_gui.run", side_effect=fake_run),
              patch("faulthandler.enable") as enable):
            capture_worker("test.elf", output, "qemu", "six_axis", 60,
                           Mock(), Mock(), interactive=True, headless=True,
                           test_disturbance=True)
            enable.call_args.args[0].close()
            report = __import__("json").loads(
                (Path(output) / "result.json").read_text(encoding="utf-8"))
        self.assertEqual(len(bullet.applied), 1)
        self.assertEqual(bullet.applied[0][-1], Bullet.LINK_FRAME)
        self.assertEqual(report["disturbance_regression"]["peak_displacement_deg"], 4.0)
        self.assertEqual(
            report["disturbance_regression"]["peak_displacement_per_cycle_deg"],
            [4.0])
        self.assertEqual(
            report["disturbance_regression"]["external_torque_substeps_per_cycle"],
            [1])
        self.assertTrue(report["disturbance_regression"]["passed"])

    def test_uart_retains_partial_send_and_retries_without_duplicate_bytes(self):
        class Socket:
            def __init__(self):
                self.calls = 0
                self.written = bytearray()
            def send(self, data):
                self.calls += 1
                if self.calls == 2:
                    raise BlockingIOError
                self.written.extend(data[:2])
                return min(2, len(data))
        raw = Socket()
        uart = BufferedUART(raw)
        uart.sendall(b"abcde")
        uart.flush()
        uart.sendall(b"fgh")
        while uart.pending:
            uart.flush()
        self.assertEqual(raw.written, b"abcdefgh")
        self.assertEqual(uart.would_block, 1)
        self.assertGreater(uart.partial_writes, 0)
        with self.assertRaises(BufferError):
            uart.sendall(b"x" * 65537)

    def test_live_state_uses_actual_raw_joint_displacements(self) -> None:
        tracker = LiveMeasurements()
        tracker.observe((0.01, 1, (3000, -4500, 6000, 2000, 500, -1000)), True)
        tracker.observe((0.04, 2, (4000, -3500, 7000, 3000, 1500, 0)), True)
        tracker.observe((0.08, 3, (3000, -4500, 6000, 2000, 500, -1000)), False)
        self.assertEqual(tracker.frames_received, 3)
        self.assertEqual(tracker.frames_while_worker_alive, 2)
        self.assertEqual(tracker.segments, {1, 2, 3})
        self.assertEqual(tracker.ranges_deg(), (10.0,) * 6)

    def test_blocked_C_APF_four_segments_are_accepted_without_relaxing_validation(self):
        tracker = LiveMeasurements(max_segments=28)
        for segment in (1, 2, 3, 4):
            tracker.observe((segment * 0.1, segment, (100 * segment,) * 6), True)
        self.assertEqual(tracker.segments, {1, 2, 3, 4})
        self.assertEqual(tracker.frames_received, 4)
        with self.assertRaisesRegex(ValueError, "segment=29"):
            tracker.observe((1.0, 29, (0,) * 6), True)
        with self.assertRaises(ValueError):
            tracker.observe((float("nan"), 5, (0,) * 6), True)

    def test_raw_C_APF_28_segments_follow_scene_capacity(self):
        tracker = LiveMeasurements(max_segments=28)
        for segment in range(1, 29):
            tracker.observe((segment * 0.01, segment, (segment * 100,) * 6), True)
        self.assertEqual(max(tracker.segments), 28)
        self.assertEqual(tracker.frames_received, 28)
        self.assertEqual(tracker.ranges_deg(), (27.0,) * 6)

    def test_real_C_APF_waypoints_define_required_axes_not_scripted_six_axes(self):
        # One joint is physically commanded to move 8deg; the other five
        # have <=0.5deg planning motion and must not be artificially failed.
        waypoints = [
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (math.radians(8), math.radians(0.5), 0.0, 0.0, 0.0, 0.0),
        ]
        self.assertEqual(
            required_motion_axes("six_axis", {"joint_waypoints_rad": waypoints}),
            [0],
        )
        self.assertEqual(len(required_motion_axes("six_axis", None)), 6)
        with self.assertRaisesRegex(ValueError, "Malformed C APF"):
            required_motion_axes("six_axis", {"joint_waypoints_rad": [(0.0,)]})

    def test_invalid_live_frame_is_rejected(self) -> None:
        tracker = LiveMeasurements()
        for frame in (
            (float("nan"), 1, (3000,) * 6),
            (0.0, -1, (3000,) * 6),
            (0.0, 1, (3000,) * 5),
            (0.0, 1, (True,) * 6),
        ):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                tracker.observe(frame, True)


if __name__ == "__main__":
    unittest.main()
