"""
Stage 4 dynamic simulation using the actual C firmware algorithms.

Requires: pybullet, cmake native build of stage4_sim_bridge.
Run from repository root:
    python simulation/pybullet_ur5/src/stage4_dynamic_closed_loop.py \
      --library build/stage4-simulation/Release/stage4_sim_bridge.dll

This deliberately does NOT pretend to exercise FreeRTOS/UART. It uses the
production joint trapezoid, incremental PID, command acceleration limiter,
UR5 model and default control parameters through the host shared library.

No separate Python PID constants, kinematic constants or firmware timing
constants. The test fixture and explicit acceptance threshold belong here.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_JSON = ROOT / "firmware/freertos_demo/config/robot_defaults.json"
URDF = ROOT / "simulation/pybullet_ur5/models/ur5/urdf/ur5.urdf"
JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
# Test FIXTURE ONLY, not the source of robot model safety limits.
START_RAW = (3000, -4500, 6000, 2000, 500, -1000)
GOAL_RAW = (3000, -4300, 6000, 2000, 500, -1000)
COUNT = len(JOINT_NAMES)
IntJoints = ctypes.c_int32 * COUNT


def load_bridge(path: Path):
    if not path.is_file():
        raise FileNotFoundError(f"Native Stage4 C bridge not found: {path}")
    lib = ctypes.CDLL(str(path.resolve()))
    lib.stage4_sim_create.argtypes = [ctypes.POINTER(ctypes.c_int32)] * 2
    lib.stage4_sim_create.restype = ctypes.c_void_p
    lib.stage4_sim_duration.argtypes = [ctypes.c_void_p]
    lib.stage4_sim_duration.restype = ctypes.c_double
    lib.stage4_sim_step.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.c_int32),
        ctypes.c_double, ctypes.c_double,
        ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32),
    ]
    lib.stage4_sim_step.restype = ctypes.c_int
    lib.stage4_sim_destroy.argtypes = [ctypes.c_void_p]
    lib.stage4_sim_destroy.restype = None
    return lib


def joint_indices(bullet, body: int) -> list[int]:
    mapping = {
        bullet.getJointInfo(body, i)[1].decode("utf-8"): i
        for i in range(bullet.getNumJoints(body))
    }
    missing = set(JOINT_NAMES) - set(mapping)
    if missing:
        raise RuntimeError(f"URDF is missing joints: {sorted(missing)}")
    return [mapping[name] for name in JOINT_NAMES]


def angles_raw(bullet, body: int, indices: list[int]) -> list[int]:
    return [
        round(math.degrees(s[0]) * 100.0)
        for s in bullet.getJointStates(body, indices)
    ]


def run(library: Path, output_dir: Path, max_final_error_deg: float,
        physics_dt_s: float, force_newton_m: float) -> dict:
    import pybullet as p

    config = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))
    control_dt_s = config["task_period_ms"]["control"] / 1000.0
    ratio = control_dt_s / physics_dt_s
    steps_per_control = round(ratio)
    if (steps_per_control <= 0 or
            not math.isclose(ratio, steps_per_control, rel_tol=0.0,
                             abs_tol=1e-9)):
        raise ValueError("Physics timestep must exactly divide firmware control period")

    bridge = load_bridge(library)
    start = IntJoints(*START_RAW)
    goal = IntJoints(*GOAL_RAW)
    handle = bridge.stage4_sim_create(start, goal)
    if not handle:
        raise RuntimeError("Native trajectory/controller initialization failed")
    try:
        duration = bridge.stage4_sim_duration(handle)
        if duration < 0 or not math.isfinite(duration):
            raise RuntimeError("Invalid C trajectory duration")

        client = p.connect(p.DIRECT)
        if client < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        try:
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(physics_dt_s)
            body = p.loadURDF(str(URDF), useFixedBase=True)
            indices = joint_indices(p, body)

            # Initialise both measured and desired poses at the same fixture.
            for index, raw in zip(indices, START_RAW):
                p.resetJointState(body, index, math.radians(raw / 100.0))
            positions = [math.radians(raw / 100.0) for raw in START_RAW]
            p.setJointMotorControlArray(
                body, indices, p.POSITION_CONTROL,
                targetPositions=positions, forces=[force_newton_m] * COUNT)

            # Warm up physics at the initial joint pose; no test target yet.
            for _ in range(100):
                p.stepSimulation()

            samples = []
            total_steps = math.ceil((duration + 2.0) / control_dt_s)
            for i in range(total_steps + 1):
                elapsed = i * control_dt_s
                feedback = IntJoints(*angles_raw(p, body, indices))
                command = IntJoints()
                trajectory = IntJoints()
                status = bridge.stage4_sim_step(
                    handle, feedback, elapsed, control_dt_s,
                    command, trajectory)
                if status != 0:
                    raise RuntimeError(
                        f"Production C Stage4 returned status {status} at t={elapsed:.3f}s"
                    )
                commanded_rads = [math.radians(raw / 100.0) for raw in command]
                p.setJointMotorControlArray(
                    body, indices, p.POSITION_CONTROL,
                    targetPositions=commanded_rads,
                    forces=[force_newton_m] * COUNT)
                for _ in range(steps_per_control):
                    p.stepSimulation()

                feedback_after = angles_raw(p, body, indices)
                samples.append({
                    "time_s": round(elapsed + control_dt_s, 6),
                    "reference_j2_deg": trajectory[1] / 100.0,
                    "command_j2_deg": command[1] / 100.0,
                    "actual_j2_deg": feedback_after[1] / 100.0,
                    "tracking_error_j2_deg": (
                        trajectory[1] - feedback_after[1]) / 100.0,
                })

            last = angles_raw(p, body, indices)
            goal_errors = [
                abs(last[j] - GOAL_RAW[j]) / 100.0
                for j in range(COUNT)
            ]
            # Measurements on the sampled trajectory, NOT claims about
            # hardware. A tolerance/settling window is a test-only policy.
            tracking_errors = [
                row["tracking_error_j2_deg"] for row in samples]
            tracking_mae = sum(abs(x) for x in tracking_errors) / len(samples)
            tracking_rmse = math.sqrt(
                sum(x * x for x in tracking_errors) / len(samples))
            initial_j2_deg = START_RAW[1] / 100.0
            goal_j2_deg = GOAL_RAW[1] / 100.0
            direction = 1.0 if goal_j2_deg >= initial_j2_deg else -1.0
            overshoot_deg = max(
                0.0, max(direction * (row["actual_j2_deg"] - goal_j2_deg)
                         for row in samples))
            settling_s = None
            for k, row in enumerate(samples):
                if (row["time_s"] >= duration and
                    all(abs(rest["actual_j2_deg"] - goal_j2_deg) <=
                        max_final_error_deg for rest in samples[k:])):
                    settling_s = row["time_s"]
                    break

            summary = {
                "kind": "native_C_stage4_algorithms_to_PyBullet_DIRECT",
                "not_tested": [
                    "Cortex-M4 FreeRTOS scheduling",
                    "UART round-trip",
                    "real hardware",
                    "collision-free full workspace",
                    "optimized PID gains",
                ],
                "control_period_s": control_dt_s,
                "physics_period_s": physics_dt_s,
                "path_duration_s": duration,
                "simulation_elapsed_s": samples[-1]["time_s"],
                "goal_error_deg_per_joint": goal_errors,
                "max_final_error_deg": max(goal_errors),
                "max_tracking_error_j2_deg": max(
                    abs(row["tracking_error_j2_deg"]) for row in samples),
                "trajectory_tracking_mae_j2_deg": tracking_mae,
                "trajectory_tracking_rmse_j2_deg": tracking_rmse,
                "overshoot_j2_deg": overshoot_deg,
                "settled_within_threshold_after_s": settling_s,
                "sample_count": len(samples),
                "acceptance_error_deg": max_final_error_deg,
                "passed": max(goal_errors) <= max_final_error_deg,
            }
        finally:
            p.disconnect()
    finally:
        bridge.stage4_sim_destroy(handle)

    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "stage4_pybullet_samples.csv").open(
            "w", newline="", encoding="utf-8") as out:
        writer = csv.DictWriter(out, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    (output_dir / "stage4_pybullet_metrics.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True,
                        help="Path to freshly built Stage4 C shared library")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "build/stage4-closed-loop")
    parser.add_argument("--max-final-error-deg", type=float, default=1.0,
                        help="Simulation acceptance criterion, not firmware config")
    parser.add_argument("--physics-dt-s", type=float, default=0.001)
    parser.add_argument("--motor-force", type=float, default=150.0,
                        help="PyBullet trial fixture, not hardware torque rating")
    args = parser.parse_args()
    if (not 0 < args.max_final_error_deg <= 10 or
            not 0 < args.physics_dt_s <= 0.01 or
            not 0 < args.motor_force <= 1000):
        parser.error("Invalid simulation trial parameters")
    summary = run(
        args.library, args.output_dir, args.max_final_error_deg,
        args.physics_dt_s, args.motor_force)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
