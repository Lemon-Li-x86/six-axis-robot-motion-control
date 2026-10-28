"""Pure-Python checks for the Stage4 PID study harness, without PyBullet."""
from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import stage4_pid_study as study  # noqa: E402


class PidMatrixStudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.defaults = json.loads(study.simulation.DEFAULT_JSON.read_text("utf-8"))

    def test_study_uses_json_baseline_gains(self) -> None:
        source = self.defaults["stage4_motion"]["pid"]
        profiles = study.gain_profiles(self.defaults)
        self.assertIsNone(profiles["baseline"])
        for profile_name in ("pi_experimental", "pid_experimental"):
            rows = profiles[profile_name]
            self.assertEqual(len(rows), study.simulation.COUNT)
            for measured, authoritative in zip(rows, source):
                self.assertEqual(measured[0], authoritative["kp"])
        self.assertTrue(all(row[2] == 0 for row in profiles["pi_experimental"]))
        self.assertTrue(all(row[1] > 0 and row[2] > 0
                            for row in profiles["pid_experimental"]))

    def test_all_cases_are_distinct_and_complete(self) -> None:
        cases = study.study_cases(self.defaults)
        expected = (len(study.SCENARIOS) * len(study.ACTUATOR_FORCES)
                    * len(study.gain_profiles(self.defaults)))
        self.assertEqual(len(cases), expected)
        identifiers = {
            (case.scenario, case.motor_force_n_m, case.profile) for case in cases
        }
        self.assertEqual(len(identifiers), expected)
        self.assertIn(study.NOMINAL_SCENARIO, study.SCENARIOS)

    def test_invalid_configuration_cannot_generate_gains(self) -> None:
        corrupted = json.loads(json.dumps(self.defaults))
        corrupted["stage4_motion"]["pid"] = []
        with self.assertRaisesRegex(ValueError, "Expected one PID"):
            study.gain_profiles(corrupted)
        corrupted = json.loads(json.dumps(self.defaults))
        corrupted["stage4_motion"]["pid"][0]["kp"] = math.nan
        with self.assertRaisesRegex(ValueError, "Invalid authoritative"):
            study.gain_profiles(corrupted)

    def test_summary_distinguishes_execution_from_acceptance(self) -> None:
        case = study.study_cases(self.defaults)[0]
        measured = {
            "passed": False,
            "trajectory_tracking_rmse_deg_per_joint": [0.1] * 6,
            "max_final_error_deg": 0.3,
            "trajectory_tracking_rmse_j2_deg": 0.12,
            "overshoot_j2_deg": 0.1,
            "sample_count": 70,
        }
        row = study.report_row(case, measured, None)
        self.assertTrue(row["integration_executed"])
        self.assertFalse(row["baseline_acceptance_passed"])
        self.assertEqual(row["worst_axis_rmse_deg"], 0.1)
        failed = study.report_row(case, None, "runtime failure")
        self.assertFalse(failed["integration_executed"])
        self.assertIsNone(failed["baseline_acceptance_passed"])
        self.assertEqual(failed["error"], "runtime failure")


if __name__ == "__main__":
    unittest.main()
