"""Guard the Stage5 multi-run QEMU A/B analyzer against fabricated PASS."""
from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aggregate_stage5_perf_repeats import summarize

NAMES = ("Control", "Trajectory", "Monitor")


def evidence(jitter=0.07, monitor_cpu=0.06, elapsed=4.72):
    cpu = {
        "frequency_hz": 25000000,
        "overflow_count": 0,
        "elapsed_seconds": elapsed,
        "observed_tasks": 3,
        "tasks": [{"name": name, "cpu_share_percent": monitor_cpu if name == "Monitor" else 1.1}
                  for name in NAMES],
    }
    timing = {
        "timer_frequency_hz": 25000000,
        "tasks": [
            {
                "task": name,
                "nominal_period_ms": {"Control": 10.0, "Trajectory": 20.0, "Monitor": 100.0}[name],
                "interval_samples": 42 if name == "Monitor" else 128,
                "execution_samples": 42 if name == "Monitor" else 128,
                "p99_abs_period_jitter_ms": jitter,
                "max_abs_period_jitter_ms": jitter * 2,
                "p99_processing_ms": 0.05,
                "max_processing_ms": 0.1,
                "stack_minimum_free_words_observed": 100,
            } for name in NAMES
        ],
    }
    return cpu, timing


def pairs(count=6, reverse_jitter=False):
    result = []
    for i in range(count):
        bc, bt = evidence(jitter=0.03 + i * 0.01, monitor_cpu=0.06)
        oc, ot = evidence(
            jitter=(0.05 + i * 0.01 if reverse_jitter else 0.02 + i * 0.01),
            monitor_cpu=0.04,
        )
        result.append((bc, oc, bt, ot))
    return result


class RepeatTests(unittest.TestCase):
    def test_paired_measured_deltas_not_averaged_over_unpaired_samples(self):
        result = summarize(pairs())
        self.assertEqual(result["pairs"], 6)
        self.assertTrue(result["interpretation"]["monitor_cpu_lower_in_all_pairs"])
        self.assertEqual(result["tasks"]["Monitor"]["cpu_share_percent"]["pairs_lower"], 6)
        self.assertEqual(result["tasks"]["Control"]["p99_abs_period_jitter_ms"]["pairs_lower"], 6)
        self.assertFalse(result["interpretation"]["jitter_optimization_proven"])
        self.assertEqual(len(result["trials"]), 6)

    def test_worse_jitter_must_be_reported_as_regression(self):
        result = summarize(pairs(reverse_jitter=True))
        v = result["tasks"]["Control"]["p99_abs_period_jitter_ms"]
        self.assertEqual(v["pairs_higher"], 6)
        self.assertGreater(v["paired_delta_median"], 0)
        self.assertFalse(result["interpretation"]["control_jitter_lower_in_all_pairs"])

    def test_requires_multiple_pairs(self):
        with self.assertRaisesRegex(ValueError, "four complete"):
            summarize(pairs(3))

    def test_rejects_missing_monitor_or_incomplete_samples(self):
        p = pairs()
        p[0][2]["tasks"][2]["interval_samples"] = 0
        with self.assertRaisesRegex(ValueError, "insufficient"):
            summarize(p)

    def test_rejects_mismatched_clock_and_elapsed(self):
        p = pairs()
        p[0][3]["timer_frequency_hz"] = 24000000
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            summarize(p)
        p = pairs()
        # Both windows remain valid 5s runs, but differ by more than 10%.
        p[0][1]["elapsed_seconds"] = 5.4
        with self.assertRaisesRegex(ValueError, "windows differ"):
            summarize(p)

    def test_rejects_short_or_mismatched_task_periods(self):
        p = pairs()
        p[0][0]["elapsed_seconds"] = 3.2
        with self.assertRaisesRegex(ValueError, "healthy-window duration"):
            summarize(p)
        p = pairs()
        p[0][3]["tasks"][0]["nominal_period_ms"] = 20.0
        with self.assertRaisesRegex(ValueError, "mismatched nominal task periods"):
            summarize(p)

    def test_rejects_nan_and_invalid_percentile(self):
        p = pairs()
        p[0][2]["tasks"][0]["p99_abs_period_jitter_ms"] = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            summarize(p)
        p = pairs()
        p[0][2]["tasks"][0]["p99_abs_period_jitter_ms"] = 9
        with self.assertRaisesRegex(ValueError, "percentile"):
            summarize(p)

    def test_rejects_overflow_and_missing_task(self):
        p = pairs()
        p[0][0]["overflow_count"] = 1
        with self.assertRaisesRegex(ValueError, "overflow"):
            summarize(p)
        p = pairs()
        p[0][2]["tasks"].pop()
        with self.assertRaisesRegex(ValueError, "missing one or more"):
            summarize(p)


if __name__ == "__main__":
    unittest.main()
