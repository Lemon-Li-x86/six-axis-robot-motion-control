#!/usr/bin/env python3
"""Fail-closed, evidence-only paired A/B analysis of QEMU Stage5 RTOS traces.

Both ELFs must be independently built at the same commit/toolchain/config,
differing only in ROBOT_ENABLE_STAGE5_MONITOR_STACK_OPT. No CI pass criterion
may be based on claiming jitter improvement: variance is reported, not hidden.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

TASKS = ("Control", "Trajectory", "Monitor")
FIELDS = (
    "p99_abs_period_jitter_ms",
    "max_abs_period_jitter_ms",
    "p99_processing_ms",
    "max_processing_ms",
    "stack_minimum_free_words_observed",
)


def _finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: expected a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name}: non-finite number")
    return result


def _validated(cpu: dict, timing: dict, role: str) -> dict:
    if cpu.get("overflow_count") != 0:
        raise ValueError(f"{role}: CPU trace overflow")
    hz = _finite_number(cpu.get("frequency_hz"), f"{role}: CPU frequency")
    timer_hz = _finite_number(timing.get("timer_frequency_hz"), f"{role}: timing frequency")
    if hz <= 0 or timer_hz != hz:
        raise ValueError(f"{role}: inconsistent Timer0/CPU frequency")
    elapsed = _finite_number(cpu.get("elapsed_seconds"), f"{role}: elapsed")
    # The A/B workflow fixes the healthy test duration at 5000ms; reject
    # accidentally shortened runs even if they contain enough samples.
    if not 4.5 <= elapsed <= 5.5:
        raise ValueError(f"{role}: unexpected healthy-window duration (expected about 5s)")
    if cpu.get("observed_tasks") != len(cpu.get("tasks", [])):
        raise ValueError(f"{role}: incomplete CPU task report")
    cpu_tasks = {t["name"]: t for t in cpu["tasks"]}
    timing_tasks = {t["task"]: t for t in timing["tasks"]}
    if any(name not in cpu_tasks or name not in timing_tasks for name in TASKS):
        raise ValueError(f"{role}: missing one or more RTOS tasks")
    result = {"elapsed_seconds": elapsed, "timer_frequency_hz": hz, "tasks": {}}
    for name in TASKS:
        c = cpu_tasks[name]
        t = timing_tasks[name]
        share = _finite_number(c.get("cpu_share_percent"), f"{role}/{name}: CPU share")
        if not 0 <= share <= 100:
            raise ValueError(f"{role}/{name}: invalid CPU share")
        # Fixed QEMU healthy period: 4-5s -> >=100 Control/Trajectory
        # and >=20 Monitor intervals. Fewer samples are not comparable.
        if int(t.get("interval_samples", 0)) < (20 if name == "Monitor" else 100):
            raise ValueError(f"{role}/{name}: insufficient healthy interval samples")
        if int(t.get("execution_samples", 0)) < (20 if name == "Monitor" else 100):
            raise ValueError(f"{role}/{name}: insufficient execution samples")
        if _finite_number(t.get("stack_minimum_free_words_observed"),
                          f"{role}/{name}: stack") <= 0:
            raise ValueError(f"{role}/{name}: zero/invalid stack margin")
        nominal_ms = _finite_number(t.get("nominal_period_ms"),
                                    f"{role}/{name}: nominal period")
        if nominal_ms <= 0:
            raise ValueError(f"{role}/{name}: invalid nominal task period")
        values = {key: _finite_number(t.get(key), f"{role}/{name}/{key}")
                  for key in FIELDS}
        if any(v < 0 for v in values.values()):
            raise ValueError(f"{role}/{name}: negative measurements")
        if (values["p99_abs_period_jitter_ms"] > values["max_abs_period_jitter_ms"]
                or values["p99_processing_ms"] > values["max_processing_ms"]):
            raise ValueError(f"{role}/{name}: percentile exceeds maximum")
        result["tasks"][name] = {"cpu_share_percent": share,
                                  "nominal_period_ms": nominal_ms,
                                  "interval_samples": t["interval_samples"],
                                  "execution_samples": t["execution_samples"],
                                  **values}
    return result


def summarize(pairs: list[tuple[dict, dict, dict, dict]]) -> dict:
    if len(pairs) < 4:
        raise ValueError("At least four complete A/B pairs are required")
    samples = []
    for i, (b_cpu, o_cpu, b_timing, o_timing) in enumerate(pairs, start=1):
        b = _validated(b_cpu, b_timing, f"pair {i} baseline")
        o = _validated(o_cpu, o_timing, f"pair {i} optimized")
        if b["timer_frequency_hz"] != o["timer_frequency_hz"]:
            raise ValueError(f"pair {i}: mismatched timer clocks")
        if abs(b["elapsed_seconds"] - o["elapsed_seconds"]) > 0.10 * b["elapsed_seconds"]:
            raise ValueError(f"pair {i}: healthy observation windows differ over 10%")
        for name in TASKS:
            if b["tasks"][name]["nominal_period_ms"] != o["tasks"][name]["nominal_period_ms"]:
                raise ValueError(f"pair {i}/{name}: mismatched nominal task periods")
        samples.append({"pair": i, "baseline": b, "optimized": o})
    overview = {}
    for name in TASKS:
        metrics = {}
        for key in ("cpu_share_percent",) + FIELDS[:4]:
            before = [x["baseline"]["tasks"][name][key] for x in samples]
            after = [x["optimized"]["tasks"][name][key] for x in samples]
            deltas = [a - b for a, b in zip(after, before)]
            metrics[key] = {
                "baseline_median": statistics.median(before),
                "optimized_median": statistics.median(after),
                "paired_delta_median": statistics.median(deltas),
                "paired_delta_min": min(deltas),
                "paired_delta_max": max(deltas),
                "pairs_lower": sum(d < 0 for d in deltas),
                "pairs_equal": sum(d == 0 for d in deltas),
                "pairs_higher": sum(d > 0 for d in deltas),
                "per_pair_deltas": deltas,
            }
        overview[name] = metrics
    n = len(pairs)
    return {
        "measurement": "interleaved same-commit QEMU RTOS monitor stack-scan A/B",
        "pairs": n,
        "order": "A-B for odd pairs, B-A for even pairs",
        "scope": "immobile-feedback Stage4 synthetic healthy RTOS workload",
        "tasks": overview,
        "trials": samples,
        "interpretation": {
            "monitor_cpu_lower_in_all_pairs":
                overview["Monitor"]["cpu_share_percent"]["pairs_lower"] == n,
            "control_jitter_lower_in_all_pairs":
                overview["Control"]["p99_abs_period_jitter_ms"]["pairs_lower"] == n,
            "trajectory_jitter_lower_in_all_pairs":
                overview["Trajectory"]["p99_abs_period_jitter_ms"]["pairs_lower"] == n,
            "jitter_optimization_proven": False,
            "explanation": (
                "No QEMU virtual-time paired trial can prove hardware WCET "
                "or robust full-PyBullet control-period jitter improvement. "
                "Review signed per-pair deltas and independent workloads."
            ),
        },
        "limitations": [
            "Final 128 Control/Trajectory intervals and about 40 Monitor samples per trial",
            "QEMU virtual timers and GitHub runner scheduling; not real-hardware WCET",
            "Fixed immobile-feedback selftest, not UART/PyBullet long-duration load",
            "Multiple paired runs assess variability but do not prove universal jitter reduction",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("--pairs", type=int, default=6)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 4 <= args.pairs <= 20:
        parser.error("--pairs must be between 4 and 20")
    rows = []
    for i in range(1, args.pairs + 1):
        n = f"pair-{i:02d}-"
        records = []
        for suffix in ("baseline.json", "optimized.json",
                       "baseline-timing.json", "optimized-timing.json"):
            path = args.directory / (n + suffix)
            records.append(json.loads(path.read_text(encoding="utf-8")))
        rows.append(tuple(records))
    result = summarize(rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items()
                      if key != "trials"}, indent=2))


if __name__ == "__main__":
    main()
