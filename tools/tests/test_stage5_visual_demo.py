"""Stage5 GUI motion sequencer regression tests (no QEMU/GUI required)."""
from __future__ import annotations

import csv
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
    TRACE_HEADER,
    joint_feedback_frame,
    validate_goals,
    write_trace,
)
import qemu_uart_receiver as wire  # noqa: E402


class MotionSequencerTests(unittest.TestCase):
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