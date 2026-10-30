"""Fail-closed checks for real GDB/FreeRTOS task timing evidence."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_stage5_task_timing import analyze, TASKS


def fixture():
    lines = ["timing_frequency_hz=25000000"]
    for i in range(3):
        vals = {
            "interval_sample_count": 120 if i != 2 else 40,
            "execution_sample_count": 120 if i != 2 else 40,
            "expected_period_counter_ticks": (250000, 500000, 2500000)[i],
            "maximum_abs_period_jitter_ticks": 40000,
            "p99_abs_period_jitter_ticks": 25000,
            "maximum_processing_ticks": 60000,
            "p99_processing_ticks": 30000,
        }
        lines.extend(f"timing_{i}_{key}={value}" for key, value in vals.items())
    lines.extend(f"stack_{name.lower()}_free_words=120" for name in TASKS)
    return "\n".join(lines) + "\n"


class TimingEvidenceTest(unittest.TestCase):
    def test_valid_gdb_snapshot(self):
        report = analyze(fixture())
        self.assertEqual(len(report["tasks"]), 3)
        self.assertEqual(report["tasks"][0]["nominal_period_ms"], 10)
        self.assertEqual(report["tasks"][1]["nominal_period_ms"], 20)
        self.assertEqual(report["tasks"][2]["nominal_period_ms"], 100)
        self.assertEqual(report["tasks"][0]["p99_abs_period_jitter_ms"], 1)
        self.assertEqual(report["tasks"][0]["stack_minimum_free_words_observed"], 120)

    def test_missing_stack_watermark_rejected(self):
        with self.assertRaisesRegex(ValueError, "GDB"):
            analyze(fixture().replace("stack_control_free_words=120", ""))

    def test_no_samples_rejected(self):
        with self.assertRaisesRegex(ValueError, "insufficient"):
            analyze(fixture().replace("timing_0_interval_sample_count=120",
                                      "timing_0_interval_sample_count=0"))

    def test_p99_greater_than_max_rejected(self):
        with self.assertRaisesRegex(ValueError, "p99"):
            analyze(fixture().replace("timing_2_p99_processing_ticks=30000",
                                      "timing_2_p99_processing_ticks=90000"))


if __name__ == "__main__":
    unittest.main()
