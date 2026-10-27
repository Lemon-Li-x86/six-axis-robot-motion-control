"""Stage5 live-measurement transport tests, independent of GUI and QEMU."""
from __future__ import annotations

import queue
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
from stage5_qemu_uart_live_gui import LiveMeasurements, LatestMailbox, capture_worker  # noqa: E402


def produce_without_reader(channel):
    for i in range(10000):
        channel.publish(i / 100, 1, (i,) * 6, (0,) * 6, i, 0)


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

    def test_headless_disturbance_uses_pybullet_torque_not_mouse_ipc(self):
        """A repeatable software force tests recovery without synthetic picking."""
        class Bullet:
            LINK_FRAME = 2
            def __init__(self):
                self.applied = []
            def getJointInfo(self, body, joint):
                return (None,) * 13 + ((0.0, 1.0, 0.0),)
            def applyExternalTorque(self, *args):
                self.applied.append(args)

        bullet = Bullet()
        def fake_run(elf, output, qemu, options, **kwargs):
            self.assertTrue(options.headless)
            kwargs["on_physics"](bullet, 0, [0, 1, 2, 3, 4, 5], 10.0, True, False)
            kwargs["on_physics"](bullet, 0, [0, 1, 2, 3, 4, 5], 11.1, True, False)
            kwargs["on_sample"](11.1, 3,
                (3400, -4500, 6000, 2000, 500, -1000),
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
