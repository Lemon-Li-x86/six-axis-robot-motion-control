#!/usr/bin/env python3
"""Turn the QEMU-only FreeRTOS Timer0 GDB report into auditable metrics.

Input is the REAL GDB transcript, not synthesized benchmark figures.
Timer0 counts simulated 25MHz peripheral cycles. P99 is nearest-rank over
at most 128 most-recent samples collected during the healthy 1.2s interval.
This is NOT a hardware WCET, CPU utilization, or host wall-clock latency.
"""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path


def parse_report(log: str) -> dict:
    fields = {}
    for line in log.splitlines():
        match = re.search(r"(?<![A-Za-z0-9_])([a-z][a-z0-9_]*)=(\d+)\s*$", line)
        if match:
            fields[match.group(1)] = int(match.group(2))

    def require(name: str) -> int:
        if name not in fields:
            raise ValueError(f"Missing GDB result: {name}")
        return fields[name]

    if require("finished") != 1 or require("checks") != 29 or require("failed") != 0:
        raise ValueError("The 29-item ARM/FreeRTOS self-test did not pass")
    frequency = require("timing_frequency_hz")
    if not 1_000_000 <= frequency <= 100_000_000:
        raise ValueError(f"Unsupported Timer0 frequency: {frequency}")

    metrics = {}
    for kind in ("control", "trajectory", "monitor"):
        interval_samples = require(f"{kind}_interval_samples")
        processing_samples = require(f"{kind}_processing_samples")
        expected = require(f"{kind}_expected_period_ticks")
        p99_jitter = require(f"{kind}_jitter_p99_ticks")
        max_jitter = require(f"{kind}_jitter_max_ticks")
        p99_processing = require(f"{kind}_processing_p99_ticks")
        max_processing = require(f"{kind}_processing_max_ticks")
        if not (2 <= interval_samples <= 128 and
                2 <= processing_samples <= 128):
            raise ValueError(f"{kind} has insufficient QEMU task samples")
        if expected <= 0 or p99_jitter > max_jitter or (
                p99_processing > max_processing):
            raise ValueError(f"{kind} timing distribution is inconsistent")
        usec_per_tick = 1e6 / frequency
        metrics[kind] = {
            "interval_sample_count": interval_samples,
            "processing_sample_count": processing_samples,
            "nominal_period_ms": expected * 1e3 / frequency,
            "p99_abs_period_jitter_us": p99_jitter * usec_per_tick,
            "max_abs_period_jitter_us": max_jitter * usec_per_tick,
            "p99_elapsed_processing_us": p99_processing * usec_per_tick,
            "max_elapsed_processing_us": max_processing * usec_per_tick,
        }
        if not all(math.isfinite(v) for v in metrics[kind].values()):
            raise ValueError(f"{kind} produced nonfinite timing metrics")

    # Use the same JSON task-stack default source as the ARM firmware.
    # Stack high-water mark denotes the MINIMUM remaining stack in words.
    defaults_path = (
        Path(__file__).resolve().parents[1]
        / "firmware/freertos_demo/config/robot_defaults.json"
    )
    defaults = json.loads(defaults_path.read_text(encoding="utf-8"))
    stacks = {}
    for kind in ("control", "trajectory", "monitor"):
        remaining = require(f"stack_words_{kind}")
        total = int(defaults["task_stack_depth_words"][kind])
        if remaining <= 0 or remaining > total:
            raise ValueError(f"{kind} stack high-water mark is invalid")
        stacks[kind] = {
            "allocated_words": total,
            "minimum_free_words": remaining,
            "peak_used_words": total - remaining,
        }

    return {
        "measurement": "QEMU_MPS2_AN386_CMSDK_TIMER0_VIRTUAL_CYCLES",
        "not_measured": [
            "physical Cortex-M4 hardware WCET",
            "real-time operating system CPU utilization",
            "UART-to-PyBullet end-to-end latency",
            "30-minute full-load stability",
        ],
        "timing_frequency_hz": frequency,
        "percentile_definition": "nearest-rank 99th percentile",
        "jitter_definition": "absolute difference between observed consecutive wake-start intervals and nominal period",
        "processing_definition": "wall elapsed simulated Timer0 ticks from after vTaskDelayUntil to activation end, including preemption",
        "samples": "first up to 128 healthy activations per task, no heap or artificial busy-work",
        "tasks": metrics,
        "stacks": stacks,
    }


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: analyze_stage4_timing.py GDB_TRANSCRIPT OUTPUT_JSON", file=sys.stderr)
        return 2
    log = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
    report = parse_report(log)
    dest = Path(sys.argv[2])
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
