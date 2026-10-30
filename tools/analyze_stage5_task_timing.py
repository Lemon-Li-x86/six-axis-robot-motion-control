#!/usr/bin/env python3
"""Turn real QEMU/GDB Timer0 task timing and FreeRTOS stack samples into evidence.

The firmware's existing stage4_timing_probe owns all capture/percentile math;
this script only validates and converts GDB output. QEMU samples are not
real-hardware WCET and the self-test is not a loaded 30-minute benchmark.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

TASKS = ("Control", "Trajectory", "Monitor")


def analyze(text: str) -> dict:
    raw = dict(re.findall(r"^([a-zA-Z_0-9]+)=([0-9]+)$", text, re.MULTILINE))
    try:
        hz = int(raw["timing_frequency_hz"])
        entries = []
        for i, name in enumerate(TASKS):
            prefix = f"timing_{i}_"
            values = {field: int(raw[prefix + field]) for field in (
                "interval_sample_count", "execution_sample_count",
                "expected_period_counter_ticks",
                "maximum_abs_period_jitter_ticks", "p99_abs_period_jitter_ticks",
                "maximum_processing_ticks", "p99_processing_ticks")}
            entries.append((name, values))
        stacks = {name: int(raw[f"stack_{name.lower()}_free_words"])
                  for name in TASKS}
    except (KeyError, ValueError) as exc:
        raise ValueError("Missing or invalid actual GDB task timing fields") from exc

    if hz <= 0 or not all(x > 0 for x in stacks.values()):
        raise ValueError("Invalid Timer0 frequency or missing live stack samples")

    report_tasks = []
    for name, values in entries:
        count = values["interval_sample_count"]
        executed = values["execution_sample_count"]
        if not (2 <= count <= 128 and 2 <= executed <= 128):
            raise ValueError(f"{name}: insufficient/out-of-range real periodic samples")
        if values["expected_period_counter_ticks"] <= 0:
            raise ValueError(f"{name}: invalid nominal period")
        if not (0 <= values["p99_abs_period_jitter_ticks"] <=
                values["maximum_abs_period_jitter_ticks"] and
                0 <= values["p99_processing_ticks"] <=
                values["maximum_processing_ticks"]):
            raise ValueError(f"{name}: p99 must not exceed measured maximum")
        report_tasks.append({
            "task": name, "interval_samples": count,
            "execution_samples": executed,
            "nominal_period_ms": values["expected_period_counter_ticks"] * 1000 / hz,
            "max_abs_period_jitter_ms": values["maximum_abs_period_jitter_ticks"] * 1000 / hz,
            "p99_abs_period_jitter_ms": values["p99_abs_period_jitter_ticks"] * 1000 / hz,
            "max_processing_ms": values["maximum_processing_ticks"] * 1000 / hz,
            "p99_processing_ms": values["p99_processing_ticks"] * 1000 / hz,
            "stack_minimum_free_words_observed": stacks[name],
        })
    return {
        "measurement": "real QEMU Timer0 FreeRTOS task scheduling/stack GDB snapshot",
        "scope": "Stage4 RTOS self-test task mix, bounded latest-128-sample window",
        "timer_frequency_hz": hz,
        "tasks": report_tasks,
        "not_measured": [
            "real hardware WCET", "30-minute sustained motion",
            "IRQ latency percentile", "full-load CPU/jitter optimization A/B",
        ],
    }


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: analyze_stage5_task_timing.py GDB_LOG OUT_JSON")
    result = analyze(Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace"))
    Path(sys.argv[2]).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
