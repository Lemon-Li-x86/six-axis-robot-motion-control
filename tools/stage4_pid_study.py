"""Reproducible Stage4 native-C/PyBullet PID sensitivity study.

This is an EXPERIMENT, not production PID tuning. Every simulated trial uses
the existing Stage4 C trajectory, PID and limiter from stage4_sim_bridge.
The Python host supplies initial/goal states and synthetic actuator loads;
it does not implement a second controller or modify robot_defaults.json.

Run after building the native C bridge:
    python tools/stage4_pid_study.py --library build/stage4-simulation/libstage4_sim_bridge.so
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))
import stage4_dynamic_closed_loop as simulation  # noqa: E402

# Test fixture only: varied movement direction/amplitude and simulated
# POSITION_CONTROL force. These are NOT industrial robot load ratings.
SCENARIOS = ("joint2", "joint2_reverse", "six_axis_medium")
ACTUATOR_FORCES = (150.0, 60.0)

# Exploratory NONZERO I and D gains. The baseline Kp is read from the same
# JSON used to build C defaults; none of these values changes production.
TRIAL_KI = 0.01
TRIAL_KD = 0.001

# Acceptance applies only to the original nominal trial. Other observations
# are measurements, even if their metrics exceed the baseline's thresholds.
NOMINAL_SCENARIO = "joint2"
NOMINAL_FORCE = ACTUATOR_FORCES[0]


@dataclass(frozen=True)
class StudyCase:
    """Independent trial parameters with explicit units and gain source."""

    scenario: str
    motor_force_n_m: float
    profile: str
    gains: tuple[tuple[float, float, float], ...] | None


def gain_profiles(defaults: dict) -> dict[str, tuple[tuple[float, float, float], ...] | None]:
    """Construct test gains from one authoritative JSON configuration."""
    rows = defaults["stage4_motion"]["pid"]
    if len(rows) != simulation.COUNT:
        raise ValueError("Expected one PID configuration for each UR5 joint")
    base = tuple((float(v["kp"]), float(v["ki"]), float(v["kd"])) for v in rows)
    if any(not all(math.isfinite(x) and x >= 0 for x in row) for row in base):
        raise ValueError("Invalid authoritative PID defaults")
    return {
        "baseline": None,
        "pi_experimental": tuple((kp, TRIAL_KI, 0.0) for kp, _, _ in base),
        "pid_experimental": tuple((kp, TRIAL_KI, TRIAL_KD) for kp, _, _ in base),
    }


def study_cases(defaults: dict) -> tuple[StudyCase, ...]:
    """Produce a stable cross-product for equivalent repeated runs."""
    profiles = gain_profiles(defaults)
    return tuple(
        StudyCase(scenario, motor_force, profile, gains)
        for scenario in SCENARIOS
        for motor_force in ACTUATOR_FORCES
        for profile, gains in profiles.items()
    )


def report_row(case: StudyCase, measured: dict | None, failure: str | None) -> dict:
    """Translate native-C loop metrics into one rectangular CSV row."""
    measured = measured or {}
    per_joint = measured.get("trajectory_tracking_rmse_deg_per_joint", [])
    return {
        "scenario": case.scenario,
        "motor_force_n_m": case.motor_force_n_m,
        "profile": case.profile,
        "integration_executed": failure is None,
        "baseline_acceptance_passed": (
            measured.get("passed") if failure is None else None
        ),
        "max_final_error_deg": measured.get("max_final_error_deg"),
        "j2_rmse_deg": measured.get("trajectory_tracking_rmse_j2_deg"),
        "j2_overshoot_deg": measured.get("overshoot_j2_deg"),
        "worst_axis_rmse_deg": max(per_joint) if per_joint else None,
        "sample_count": measured.get("sample_count"),
        "error": failure or "",
    }


def run_matrix(library: Path, output_dir: Path) -> dict:
    """Execute each independent test; retain evidence even on trial failure."""
    defaults = json.loads(simulation.DEFAULT_JSON.read_text(encoding="utf-8"))
    cases = study_cases(defaults)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    failures = []
    for index, case in enumerate(cases, start=1):
        print(
            f"[{index}/{len(cases)}] {case.scenario}, "
            f"force={case.motor_force_n_m}Nm, gains={case.profile}",
            flush=True,
        )
        trial_dir = (
            output_dir / case.scenario /
            f"force_{int(case.motor_force_n_m)}" / case.profile
        )
        try:
            metrics = simulation.run(
                library=library,
                output_dir=trial_dir,
                max_final_error_deg=simulation.DEFAULT_FINAL_ERROR_DEG,
                physics_dt_s=simulation.DEFAULT_PHYSICS_DT_S,
                force_newton_m=case.motor_force_n_m,
                scenario=case.scenario,
                pid_gains=case.gains,
            )
            rows.append(report_row(case, metrics, None))
        except (OSError, RuntimeError, ValueError) as exc:
            failures.append(f"{case.scenario}/{case.profile}: {exc}")
            rows.append(report_row(case, None, str(exc)))

    report_path = output_dir / "study_matrix.csv"
    with report_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    # Emit the measured reason a trial missed the ORIGINAL small-step policy.
    # These exploratory observations are not automatically production failures.
    for row in rows:
        if row["integration_executed"] and row["baseline_acceptance_passed"] is False:
            print(
                "[OUTSIDE_BASELINE_POLICY] "
                f"{row['scenario']} force={row['motor_force_n_m']} "
                f"gains={row['profile']} "
                f"final={row['max_final_error_deg']:.4f}deg "
                f"J2_RMSE={row['j2_rmse_deg']:.4f}deg "
                f"J2_overshoot={row['j2_overshoot_deg']:.4f}deg",
                flush=True,
            )

    nominal = next(
        r for r in rows
        if r["scenario"] == NOMINAL_SCENARIO
        and r["motor_force_n_m"] == NOMINAL_FORCE
        and r["profile"] == "baseline"
    )
    report = {
        "scope": "host_native_C_stage4_PID_PyBullet_DIRECT_sensitivity",
        "python": platform.python_version(),
        "config_source": str(simulation.DEFAULT_JSON.relative_to(ROOT)),
        "production_pid_config_changed": False,
        "case_count": len(rows),
        "completed_without_exception": sum(r["integration_executed"] for r in rows),
        "nominal_reference_passed": nominal["baseline_acceptance_passed"] is True,
        "exploratory_baseline_threshold_passes": sum(
            r["baseline_acceptance_passed"] is True for r in rows
        ),
        "failures": failures,
        "candidate_ki": TRIAL_KI,
        "candidate_kd": TRIAL_KD,
        "simulated_force_n_m": list(ACTUATOR_FORCES),
        "scenarios": list(SCENARIOS),
        "not_measured": [
            "FreeRTOS task scheduling and CPU utilization",
            "QEMU UART transport timing",
            "industrial robot joint torque",
            "verified full-workspace tuning",
        ],
    }
    (output_dir / "study_summary.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "build" / "stage4-pid-study"
    )
    args = parser.parse_args()
    report = run_matrix(args.library, args.output_dir)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if not report["failures"] and report["nominal_reference_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
