"""Fail-closed operator/CI acceptance check for Stage5 native PyBullet GUI.

Accepts only measured QEMU/FreeRTOS/C APF/UART/physics evidence. This is a
virtual-motor simulation gate, NOT real industrial robot hardware certification.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))

from stage5_safe_gui_jog import JOG_MIN_DEG, JOG_MAX_DEG  # noqa: E402
from stage5_qemu_uart_visual_demo import SETTLE_TOLERANCE_DEG  # noqa: E402


def evaluate(report: dict, *, two_step: bool = False) -> list[str]:
    """Return reasons for NOT PASS, without weakening the production verdict."""
    failures: list[str] = []
    def need(ok: bool, message: str) -> None:
        if not ok:
            failures.append(message)

    need(report.get("passed") is True, "production C/UART/physics verdict != PASS")
    need(report.get("headless") is False, "not native PyBullet GUI")
    need(report.get("gui_safe_jog") is True, "guarded GUI jog was not enabled")
    need(report.get("cancelled") is False, "test was cancelled, not completed")
    need(report.get("motor_commands") == "virtual_serial_only",
         "unexpected actuator protocol scope")
    need(report.get("segments_requested", 0) > 0 and
         report.get("segments_requested") == report.get("segments_settled"),
         "original C APF segments were not all settled")
    need(isinstance(report.get("avoidance_preflight"), dict) and
         report["avoidance_preflight"].get("passed") is True,
         "original production C APF/IK full-link preflight missing")
    need(isinstance(report.get("obstacle_recovery_preflight"), dict) and
         report["obstacle_recovery_preflight"].get("passed") is True,
         "guarded C recovery preflight missing")
    need(report.get("runtime_full_link_collision_free") is True,
         "physical per-frame full-link collision validation missing")
    # A PASS must not hide a later operator click that was refused. This
    # previously let "+0.5deg PASS then +2deg REFUSED" appear successful.
    need(report.get("safe_gui_jog_refused", 0) == 0,
         "one or more operator GUI jog requests were refused")
    applied = report.get("safe_gui_jog_applied")
    completed = report.get("safe_gui_jog_completed")
    recovered = report.get("hold_recoveries")
    expected = 2 if two_step else 1
    need(type(applied) is int and applied == expected and
         applied == completed == recovered,
         "guarded GUI jog, C UART recovery and physical settlement counts differ")
    routes = report.get("obstacle_recovery_routes")
    need(isinstance(routes, list) and len(routes) == applied and
         all(isinstance(route, dict) and route.get("completed") is True
             for route in routes),
         "not every C recovery route completed")
    events = report.get("safe_gui_jog_events")
    if two_step:
        need(isinstance(events, list) and any(
            isinstance(event, dict)
            and event.get("event") == "safe_gui_jog_finish_requested"
            for event in events),
            "operator FINISH after second physical return missing")
    actual_steps = [
        event.get("displacement_deg")
        for event in (events if isinstance(events, list) else [])
        if isinstance(event, dict)
        and event.get("event") == "safe_gui_jog_applied"
        and event.get("joint") == 6
    ]
    required_steps = (JOG_MIN_DEG, JOG_MAX_DEG) if two_step else (JOG_MIN_DEG,)
    need(len(actual_steps) == len(required_steps) and all(
        isinstance(actual, (int, float)) and
        math.isfinite(actual) and
        math.isclose(actual, expected_step, rel_tol=0, abs_tol=1e-8)
        for actual, expected_step in zip(actual_steps, required_steps)),
        "J6 requested steps differ from independently required sequence")
    need(report.get("bad_checksum_frames") == 0 and
         report.get("bad_ack_frames") == 0,
         "UART checksum/ACK fault")
    need(report.get("clock_alignment_passed") is True,
         "C FreeRTOS / PyBullet clock alignment failed")
    faults = report.get("fault_counters")
    need(isinstance(faults, dict) and
         all(faults.get(k) == 0 for k in (
             "controller_faults", "planner_faults", "feedback_stale_count")),
         "firmware fault counter is nonzero or missing")
    need(isinstance(report.get("hold_final_error_deg"), (int, float))
         and math.isfinite(report["hold_final_error_deg"])
         and report["hold_final_error_deg"] <= SETTLE_TOLERANCE_DEG,
         "physical six-axis final HOLD error above existing settle tolerance")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path,
                        help="Stage5 output_dir/result.json")
    parser.add_argument("--two-step", action="store_true",
                        help="Require two physically recovered J6 moves: +0.5, +2deg")
    args = parser.parse_args()
    if not args.result.is_file():
        print(f"[STAGE5 GUI] NOT PASS: result.json missing: {args.result}")
        return 1
    try:
        data = json.loads(args.result.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("result must be an object")
    except (OSError, ValueError) as exc:
        print(f"[STAGE5 GUI] NOT PASS: invalid JSON: {exc}")
        return 1
    failures = evaluate(data, two_step=args.two_step)
    if failures:
        print("[STAGE5 GUI] NOT PASS: " + "; ".join(failures))
        print("Keep worker.log, uart_live.jsonl, observer.jsonl and result.json.")
        return 1
    print("[STAGE5 GUI] PASS: native virtual GUI, "
          + ("J6 +0.5deg then +2deg" if args.two_step else "J6 +0.5deg")
          + ", C APF/IK, "
          "full-link clearance, UART ACK and physical settled recovery.")
    print("This does not certify physical six-axis robot hardware.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
