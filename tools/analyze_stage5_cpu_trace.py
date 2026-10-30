#!/usr/bin/env python3
"""Validate GDB-sampled FreeRTOS per-task runtime, without inventing CPU %. """
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

EXPECTED_TASKS = {"Control", "Trajectory", "Monitor"}


def analyze(text: str) -> dict:
    values = dict(re.findall(r"^(\w+)=(.*)$", text, re.MULTILINE))
    required = ("elapsed_ticks", "timer_frequency_hz", "observed_tasks", "overflow_count")
    if any(key not in values for key in required):
        raise ValueError("Missing GDB CPU trace fields")
    elapsed = int(values["elapsed_ticks"])
    freq = int(values["timer_frequency_hz"])
    count = int(values["observed_tasks"])
    overflow = int(values["overflow_count"])
    if elapsed <= 0 or freq <= 0 or count <= 0 or count > 16 or overflow != 0:
        raise ValueError("Invalid CPU trace timebase/capacity/overflow")
    tasks = []
    for i in range(count):
        name = values[f"task_{i}_name"].strip()
        active = int(values[f"task_{i}_active_ticks"])
        bps = int(values[f"task_{i}_cpu_bp"])
        if active < 0 or active > elapsed or not (0 <= bps <= 10000):
            raise ValueError("Task active time exceeds observation interval")
        if abs(bps - (active * 10000 // elapsed)) > 1:
            raise ValueError("CPU proportion does not match runtime counters")
        tasks.append({"name": name, "active_timer_ticks": active, "cpu_share_percent": bps / 100.0})
    observed = {x["name"] for x in tasks}
    if not EXPECTED_TASKS.issubset(observed):
        raise ValueError(f"Missing required real RTOS tasks: {EXPECTED_TASKS-observed}")
    if sum(x["cpu_share_percent"] for x in tasks) > 100.1:
        raise ValueError("Per-task utilization exceeds measured elapsed time")
    return {"measurement": "QEMU FreeRTOS switch/tick sampled task execution",
            "elapsed_seconds": elapsed/freq, "frequency_hz": freq,
            "observed_tasks": count, "overflow_count": overflow,
            "tasks": tasks}


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: analyze_stage5_cpu_trace.py GDB_LOG OUT_JSON")
    result = analyze(Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace"))
    Path(sys.argv[2]).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
