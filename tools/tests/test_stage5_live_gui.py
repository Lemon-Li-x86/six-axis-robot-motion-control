"""Stage5 live-measurement transport tests, independent of GUI and QEMU."""
from __future__ import annotations

import queue
import multiprocessing as mp
import time
import sys
import unittest
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/pybullet_ur5/src"))

from stage5_qemu_uart_closed_loop import BufferedUART
from stage5_drag_interaction import DragMailbox, PhysicalDrag, MouseDrag, plane_target, DRAG_LEASE_S
from stage5_qemu_uart_live_gui import LiveMeasurements, LatestMailbox  # noqa: E402


def produce_without_reader(channel):
    for i in range(10000):
        channel.publish(i / 100, 1, (i,) * 6, (0,) * 6, i, 0)


class LiveTransportTests(unittest.TestCase):
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

    def test_drag_transport_expires_and_does_not_block(self):
        box = DragMailbox(mp.get_context("spawn"))
        box.publish(True, 2, (0.,0.,0.), (1.,2.,3.), now=10.)
        self.assertTrue(box.read(now=10.1)[0])
        self.assertFalse(box.read(now=10.+DRAG_LEASE_S+0.01)[0])
        box.lock.acquire()
        try:
            self.assertFalse(box.publish(False, -1, (0.,)*3, (0.,)*3))
            self.assertIsNone(box.read())
        finally:
            box.lock.release()
        self.assertEqual(box.dropped.value, 1)
        with self.assertRaises(ValueError):
            box.publish(True, 2, (float("nan"),0.,0.), (0.,)*3)

    def test_physical_drag_creates_updates_and_releases_constraint(self):
        class Bullet:
            JOINT_POINT2POINT = 5
            def __init__(self):
                self.created = []; self.updated = []; self.removed = []
            def getNumJoints(self, arm): return 6
            def createConstraint(self, *args):
                self.created.append(args); return 42
            def changeConstraint(self, *args, **kwargs): self.updated.append((args,kwargs))
            def removeConstraint(self, constraint): self.removed.append(constraint)
        box = DragMailbox(mp.get_context("spawn")); physics = PhysicalDrag(box); bullet = Bullet()
        box.publish(True, 2, (0.,)*3, (0.1,0.2,0.3))
        physics.apply(bullet, 7)
        self.assertEqual(bullet.created[0][:4], (7,2,-1,-1))
        self.assertEqual(physics.started, 1)
        physics.apply(bullet, 7)
        self.assertEqual(len(bullet.created), 1)
        # A stale GUI must release the actual physical grab, too.
        with patch("stage5_drag_interaction.time.monotonic", return_value=time.monotonic()+1):
            physics.apply(bullet, 7)
        self.assertEqual(bullet.removed, [42])
        self.assertEqual(physics.released, 1)

    def test_drag_plane_preserves_depth_and_caps_virtual_reach(self):
        target = plane_target((0,0,0), (2,0,10), (0,0,1), (0,0,1))
        self.assertEqual(target, (0.2,0.0,1.0))
        far = plane_target((0,0,0), (100,0,1), (0,0,1), (0,0,1))
        self.assertAlmostEqual(far[0], 0.35)
        self.assertAlmostEqual(far[2], 1)

    def test_mouse_click_motion_release_publish_same_link_local_anchor(self):
        class Bullet:
            KEY_WAS_TRIGGERED=1; KEY_WAS_RELEASED=2
            def __init__(self): self.events=[]
            def getMouseEvents(self): return self.events
            def getDebugVisualizerCamera(self): return (100,100,None,None,None,(0,0,1))
            def rayTest(self, *args): return [(7,2,0.5,(0.,0.,1.))]
            def getLinkState(self, *args, **kwargs): return ((0,0,1),(0,0,0,1))
            def invertTransform(self, *args): return ((0,0,-1),(0,0,0,1))
            def multiplyTransforms(self, *args): return ((0,0,0),(0,0,0,1))
        bullet=Bullet(); box=DragMailbox(mp.get_context("spawn")); mouse=MouseDrag(box)
        with patch("stage5_drag_interaction.screen_ray", return_value=((0,0,0),(1,0,10))):
            bullet.events=[(2,50,50,0,1)]; mouse.poll(bullet,7)
            self.assertTrue(box.read()[0]); self.assertEqual(box.read()[1],2)
            bullet.events=[(1,60,50,-1,0)]; mouse.poll(bullet,7)
            self.assertAlmostEqual(box.read()[3][0],0.1)
            bullet.events=[(2,60,50,0,2)]; mouse.poll(bullet,7)
            self.assertFalse(box.read()[0])
            # Rapid real input may contain both pressed and released bits.
            bullet.events=[(2,50,50,0,3),(1,60,50,-1,0)]; mouse.poll(bullet,7)
            self.assertTrue(box.read()[0])
            bullet.events=[]
            with patch("stage5_drag_interaction.time.monotonic", return_value=time.monotonic()+1):
                mouse.poll(bullet,7)
            self.assertFalse(box.read()[0])

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
