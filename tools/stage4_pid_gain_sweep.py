"""Stage4 C/PyBullet gain sweep with separate calibration and validation cases.

The actual controller, trajectory planner and command limiter are the
production C sources in stage4_sim_bridge. No PID logic is reimplemented here.
The simulator actuator-force setting is a test fixture, not an industrial
load or calibrated motor torque. This script NEVER changes robot_defaults.json.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import stage4_pid_study as study

simulation = study.simulation
ROOT = study.ROOT

# Search values belong to this experimental study, not production config.
# Hold out J2 forward/reverse from the medium-amplitude calibration cases.
KP_MULTIPLIERS = (0.75, 1.0, 1.25)
KD_TRIALS = (0.0, 0.001, 0.004, 0.012, 0.025)
KI_TRIALS = (0.0, 0.01)
CALIBRATION_SCENARIO = "six_axis_medium"
VALIDATION_SCENARIOS = ("joint2", "joint2_reverse")
FINALISTS = 5

# Reuse the unchanged Stage4 small-step validation policy as a conservative,
# explicitly *experimental* comparison across all six simulated conditions.
RMSE_LIMIT_DEG = 0.25
OVERSHOOT_LIMIT_DEG = 0.15
FINAL_LIMIT_DEG = simulation.DEFAULT_FINAL_ERROR_DEG


@dataclass(frozen=True)
class Candidate:
    """Uniform trial multiplier and I/D gains applied to each JSON P value."""

    multiplier: float
    ki: float
    kd: float

    @property
    def name(self) -> str:
        return f"kp_{self.multiplier:g}_ki_{self.ki:g}_kd_{self.kd:g}"


def candidates() -> tuple[Candidate, ...]:
    """Stable search order for reproducible CI and local trials."""
    return tuple(
        Candidate(scale, ki, kd)
        for scale in KP_MULTIPLIERS
        for ki in KI_TRIALS
        for kd in KD_TRIALS
    )


def gains_for(candidate: Candidate, defaults: dict):
    """Scale only the C-default proportional gains; do not duplicate them."""
    original = defaults["stage4_motion"]["pid"]
    if len(original) != simulation.COUNT:
        raise ValueError("PID configuration must contain six joints")
    gains = tuple(
        (float(g["kp"]) * candidate.multiplier, candidate.ki, candidate.kd)
        for g in original
    )
    if not all(math.isfinite(v) and v >= 0 for triple in gains for v in triple):
        raise ValueError("Non-finite or negative PID test gain")
    return gains


def violation_ratio(metrics: dict) -> float:
    """Worst ratio against unchanged test-only numeric comparison limits."""
    values = (
        metrics["trajectory_tracking_rmse_j2_deg"] / RMSE_LIMIT_DEG,
        metrics["overshoot_j2_deg"] / OVERSHOOT_LIMIT_DEG,
        metrics["max_final_error_deg"] / FINAL_LIMIT_DEG,
    )
    if any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("Invalid simulation metric")
    return max(values)


def run_trial(library: Path, root: Path, candidate: Candidate, defaults: dict,
              scenario: str, force: float) -> dict:
    """Execute one native-C + PyBullet physical-feedback trial."""
    output = root / candidate.name / scenario / f"force_{int(force)}"
    metrics = simulation.run(
        library=library,
        output_dir=output,
        max_final_error_deg=FINAL_LIMIT_DEG,
        max_tracking_rmse_j2_deg=RMSE_LIMIT_DEG,
        max_overshoot_j2_deg=OVERSHOOT_LIMIT_DEG,
        physics_dt_s=simulation.DEFAULT_PHYSICS_DT_S,
        force_newton_m=force,
        scenario=scenario,
        pid_gains=gains_for(candidate, defaults),
    )
    return {
        "candidate": candidate.name,
        "scenario": scenario,
        "force_n_m": force,
        "j2_rmse_deg": metrics["trajectory_tracking_rmse_j2_deg"],
        "j2_overshoot_deg": metrics["overshoot_j2_deg"],
        "final_error_deg": metrics["max_final_error_deg"],
        "violation_ratio": violation_ratio(metrics),
        "within_comparison_policy": metrics["passed"],
    }


def write_matrix(path: Path, rows: list[dict]) -> None:
    """Keep trial evidence even if no candidate satisfies the policy."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run(library: Path, output_dir: Path) -> dict:
    """Calibrate on medium movement, then validate finalists on held-out moves."""
    defaults = json.loads(simulation.DEFAULT_JSON.read_text(encoding="utf-8"))
    trials = candidates()
    calibration_rows = []
    calibration_scores = []
    for index, candidate in enumerate(trials, start=1):
        print(f"[CALIBRATE {index}/{len(trials)}] {candidate.name}", flush=True)
        points = [
            run_trial(library, output_dir / "calibration", candidate, defaults,
                      CALIBRATION_SCENARIO, force)
            for force in study.ACTUATOR_FORCES
        ]
        calibration_rows.extend(points)
        calibration_scores.append(
            (max(p["violation_ratio"] for p in points), candidate)
        )

    # Stable tie-break: retain deterministic candidate order in close runs.
    calibration_scores.sort(key=lambda value: value[0])
    shortlisted = [c for _, c in calibration_scores[:FINALISTS]]
    validation_rows = []
    for index, candidate in enumerate(shortlisted, start=1):
        print(f"[VALIDATE {index}/{len(shortlisted)}] {candidate.name}", flush=True)
        for scenario in VALIDATION_SCENARIOS:
            for force in study.ACTUATOR_FORCES:
                validation_rows.append(
                    run_trial(library, output_dir / "validation", candidate,
                              defaults, scenario, force)
                )

    write_matrix(output_dir / "calibration.csv", calibration_rows)
    write_matrix(output_dir / "validation.csv", validation_rows)
    assessment = []
    for candidate in shortlisted:
        scoped_rows = [
            r for r in calibration_rows + validation_rows
            if r["candidate"] == candidate.name
        ]
        assessment.append({
            "candidate": candidate.name,
            "max_violation_ratio": max(r["violation_ratio"] for r in scoped_rows),
            "conditions_within_policy": sum(
                r["within_comparison_policy"] for r in scoped_rows
            ),
            "conditions_tested": len(scoped_rows),
        })
    assessment.sort(key=lambda r: r["max_violation_ratio"])
    report = {
        "scope": "Stage4_native_C_PID_to_PyBullet_DIRECT_gain_sensitivity",
        "defaults_modified": False,
        "calibration_trials": len(calibration_rows),
        "held_out_validation_trials": len(validation_rows),
        "best_evaluated": assessment[0],
        "shortlist": assessment,
        "all_six_conditions_within_test_policy": (
            assessment[0]["max_violation_ratio"] <= 1.0
        ),
        "notes": [
            "These comparisons are only valid for tested URDF and motor-force fixtures.",
            "A favorable candidate is not automatically an approved firmware gain.",
            "No QEMU, hardware torque or real-time scheduling is measured.",
        ],
    }
    (output_dir / "sweep_summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "build/stage4-pid-sweep"
    )
    args = parser.parse_args()
    result = run(args.library, args.output_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    # Sensitivity studies should not fail CI solely because no candidate
    # satisfies an exploratory cross-condition comparison threshold.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
