"""检查实测轨迹回放输入，不依赖 QEMU、GUI 或 PyBullet 动力学。"""
from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/pybullet_ur5/src"))

from stage4_dynamic_closed_loop import JOINT_NAMES  # noqa: E402
from stage5_qemu_uart_trace_replay import load_trace, measured_ranges  # noqa: E402


class TraceReplayTests(unittest.TestCase):
    """只接受完整六轴、真实数值、严格递增时间的采样。"""

    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "joint_trace.csv"

    def write(self, rows) -> None:
        columns = ["simulated_time_s", "segment"]
        columns.extend(f"actual_{name}_deg" for name in JOINT_NAMES)
        with self.path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, columns)
            writer.writeheader()
            for timestamp, segment, angles in rows:
                values = {"simulated_time_s": timestamp, "segment": segment}
                values.update(
                    {f"actual_{name}_deg": angle
                     for name, angle in zip(JOINT_NAMES, angles)}
                )
                writer.writerow(values)

    def test_valid_trace_reports_actual_six_joint_motion(self) -> None:
        self.write([
            (0.01, 1, (30, -45, 60, 20, 5, -10)),
            (0.02, 1, (38, -37, 68, 28, 13, -2)),
        ])
        frames = load_trace(self.path)
        self.assertEqual(len(frames), 2)
        self.assertEqual(measured_ranges(frames), (8, 8, 8, 8, 8, 8))

    def test_rejects_duplicate_time(self) -> None:
        self.write([(0.01, 1, (0,) * 6), (0.01, 1, (1,) * 6)])
        with self.assertRaisesRegex(ValueError, "Non-monotonic"):
            load_trace(self.path)

    def test_rejects_nonfinite_joint_angles(self) -> None:
        self.write([(0.01, 1, (0,) * 6), (0.02, 1, (float("nan"),) + (1,) * 5)])
        with self.assertRaisesRegex(ValueError, "invalid angles"):
            load_trace(self.path)

    def test_rejects_incomplete_recording(self) -> None:
        self.write([(0.01, 1, (0,) * 6)])
        with self.assertRaisesRegex(ValueError, "fewer than two"):
            load_trace(self.path)


if __name__ == "__main__":
    unittest.main()
