"""Stage5 live-measurement transport tests, independent of GUI and QEMU."""
from __future__ import annotations

import queue
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/pybullet_ur5/src"))

from stage5_qemu_uart_live_gui import LiveMeasurements, publish_latest  # noqa: E402


class LiveTransportTests(unittest.TestCase):
    def test_full_gui_channel_never_blocks_control(self) -> None:
        channel: queue.Queue = queue.Queue(maxsize=2)
        publish_latest(channel, (0.01, 1, (3000,) * 6))
        publish_latest(channel, (0.02, 1, (3100,) * 6))
        publish_latest(channel, (0.03, 2, (3200,) * 6))
        self.assertEqual(channel.qsize(), 2)
        frames = [channel.get_nowait(), channel.get_nowait()]
        self.assertEqual(frames[-1][0], 0.03)
        self.assertEqual(frames[0][0], 0.02)

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
