"""Host-only tests for strict GDB timing report parsing. No ARM needed."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_stage4_timing import parse_report  # noqa: E402


def fixture() -> str:
    values = {"finished": 1, "checks": 29, "failed": 0,
              "healthy_duration_ms": 1200,
              "timing_frequency_hz": 25000000}
    for name, ms in (("control", 10), ("trajectory", 20),
                     ("monitor", 100)):
        values.update({
            f"{name}_interval_samples": 60,
            f"{name}_processing_samples": 61,
            f"{name}_expected_period_ticks": 25000 * ms,
            f"{name}_jitter_p99_ticks": 250,
            f"{name}_jitter_max_ticks": 700,
            f"{name}_processing_p99_ticks": 10000,
            f"{name}_processing_max_ticks": 15000,
            f"stack_words_{name}": 100,
        })
    return "\n".join(f"{key}={value}" for key, value in values.items())


class TimingReportTest(unittest.TestCase):
    def test_positive_sample_and_exact_unit_conversion(self):
        report = parse_report(fixture())
        control = report["tasks"]["control"]
        self.assertEqual(control["nominal_period_ms"], 10.0)
        self.assertEqual(control["p99_abs_period_jitter_us"], 10.0)
        self.assertEqual(control["max_elapsed_processing_us"], 600.0)
        self.assertEqual(control["interval_sample_count"], 60)
        self.assertEqual(report["stacks"]["control"]["allocated_words"], 768)
        self.assertEqual(report["stacks"]["control"]["peak_used_words"], 668)

    def test_rejects_invalid_qemu_health_window(self):
        damaged = fixture().replace(
            "healthy_duration_ms=1200", "healthy_duration_ms=0")
        with self.assertRaisesRegex(ValueError, "healthy run duration"):
            parse_report(damaged)

    def test_requires_numeric_p99_fields(self):
        damaged = fixture().replace("control_jitter_p99_ticks=250", "")
        with self.assertRaisesRegex(ValueError, "control_jitter_p99_ticks"):
            parse_report(damaged)

    def test_rejects_failed_qemu_selftest(self):
        damaged = fixture().replace("failed=0", "failed=1")
        with self.assertRaises(ValueError):
            parse_report(damaged)

    def test_rejects_inconsistent_percentile(self):
        damaged = fixture().replace(
            "control_jitter_p99_ticks=250",
            "control_jitter_p99_ticks=1000")
        with self.assertRaises(ValueError):
            parse_report(damaged)

    def test_rejects_excessive_qemu_p99_jitter(self):
        damaged = fixture().replace(
            "control_jitter_p99_ticks=250",
            "control_jitter_p99_ticks=26000").replace(
            "control_jitter_max_ticks=700",
            "control_jitter_max_ticks=26000")
        with self.assertRaisesRegex(ValueError, "P99 interval jitter"):
            parse_report(damaged)

    def test_rejects_activation_longer_than_control_period(self):
        damaged = fixture().replace(
            "control_processing_max_ticks=15000",
            "control_processing_max_ticks=251000")
        with self.assertRaisesRegex(ValueError, "exceeds QEMU test period"):
            parse_report(damaged)

    def test_rejects_impossible_stack_watermark(self):
        damaged = fixture().replace(
            "stack_words_monitor=100",
            "stack_words_monitor=999999")
        with self.assertRaises(ValueError):
            parse_report(damaged)


if __name__ == "__main__":
    unittest.main()
