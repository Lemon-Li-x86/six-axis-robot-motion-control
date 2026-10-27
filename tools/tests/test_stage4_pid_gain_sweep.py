"""Test Stage4 gain-sweep evidence logic without a C compiler or PyBullet."""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import stage4_pid_gain_sweep as sweep  # noqa: E402


class Stage4GainSweepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.defaults = json.loads(sweep.simulation.DEFAULT_JSON.read_text("utf-8"))

    def test_candidates_are_unique_and_deterministic(self) -> None:
        options = sweep.candidates()
        self.assertEqual(
            len(options),
            len(sweep.KP_MULTIPLIERS) * len(sweep.KI_TRIALS)
            * len(sweep.KD_TRIALS),
        )
        self.assertEqual(len({c.name for c in options}), len(options))
        self.assertEqual(options, sweep.candidates())

    def test_default_gain_source_is_config_json(self) -> None:
        candidate = sweep.Candidate(0.75, 0.005, 0.004)
        rows = sweep.gains_for(candidate, self.defaults)
        self.assertEqual(len(rows), sweep.simulation.COUNT)
        for new, config in zip(rows, self.defaults["stage4_motion"]["pid"]):
            self.assertAlmostEqual(new[0], config["kp"] * 0.75)
            self.assertEqual(new[1:], (0.005, 0.004))
        self.assertEqual(self.defaults["stage4_motion"]["pid"][0]["ki"], 0)

    def test_violation_ratio_uses_stable_policy_without_adjustment(self) -> None:
        valid = {
            "trajectory_tracking_rmse_j2_deg": 0.1,
            "overshoot_j2_deg": 0.0,
            "max_final_error_deg": 0.02,
        }
        self.assertAlmostEqual(sweep.violation_ratio(valid), 0.4)
        invalid = dict(valid, overshoot_j2_deg=0.45)
        self.assertAlmostEqual(sweep.violation_ratio(invalid), 3.0)
        with self.assertRaises(ValueError):
            sweep.violation_ratio(dict(valid, max_final_error_deg=float("nan")))

    def test_csv_preserves_individual_case_evidence(self) -> None:
        row = {
            "candidate": "example",
            "scenario": "joint2",
            "violation_ratio": 1.5,
            "within_comparison_policy": False,
        }
        with tempfile.TemporaryDirectory() as dirname:
            target = Path(dirname) / "matrix.csv"
            sweep.write_matrix(target, [row])
            data = target.read_text("utf-8")
            self.assertIn("candidate,scenario", data)
            self.assertIn("example,joint2,1.5,False", data)


if __name__ == "__main__":
    unittest.main()
