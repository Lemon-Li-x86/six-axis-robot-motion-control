"""Controlled DIRECT/QEMU six-axis disturbance and real C APF HOLD recovery.

No GUI mouse picking and no physical robot. This is a deterministic integration
harness, NOT proof that unbounded manual dragging is collision safe.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))

from stage5_qemu_uart_visual_demo import DemoOptions, run
from stage5_avoidance_uart_gate import make_avoidance_preflight, load_scene
from stage5_collision_validation import validate_joint_path, InvalidPath


class OneSafeJointDisplacement:
    def __init__(self):
        self.injected = False
        self.displacement_deg = 2.0

    def __call__(self, bullet, arm, joints, time_s, done, ending):
        if done and not ending and not self.injected:
            joint = joints[0]
            angle = bullet.getJointState(arm, joint)[0]
            # A programmed position reset is not a real push. However, even
            # this deterministic fixture must not skip the actual full-link
            # geometric gate BEFORE moving the simulated robot.
            scene, policy = load_scene("blocked")
            measured = [bullet.getJointState(arm, j)[0] for j in joints]
            displaced = list(measured)
            displaced[0] = angle + math.radians(self.displacement_deg)
            others = [
                bullet.getBodyUniqueId(i)
                for i in range(bullet.getNumBodies())
                if bullet.getBodyUniqueId(i) != arm
            ]
            if len(others) != 1:
                raise RuntimeError(
                    "Controlled displacement requires exactly one "
                    "authoritative PyBullet obstacle body")
            gate = validate_joint_path(
                bullet, arm, others, joints, [measured, displaced],
                maximum_joint_step_rad=policy["max_joint_step_rad"],
                clearance_m=scene["minimum_full_link_clearance_mm"] / 1000.0,
                maximum_samples=policy["max_collision_samples"],
                physics_client_id=0)
            if not gate["passed"]:
                raise RuntimeError(
                    "Controlled displacement blocked before actuation: "
                    + str(gate["collision"]))
            bullet.resetJointState(
                arm, joint, targetValue=displaced[0], targetVelocity=0.0)
            self.injected = True
            print(
                "[TEST ONLY] Synthetic two-degree shoulder-pan displacement "
                f"at simulated_t={time_s:.3f}s",
                flush=True)


def main():
    cli = argparse.ArgumentParser()
    cli.add_argument("--elf", required=True, type=Path)
    cli.add_argument("--library", required=True, type=Path)
    cli.add_argument("--output-dir", required=True, type=Path)
    cli.add_argument("--qemu", default="qemu-system-arm")
    args = cli.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    verdict_path = args.output_dir / "controlled_recovery_acceptance.json"
    verdict_path.unlink(missing_ok=True)
    impulse = OneSafeJointDisplacement()
    options = DemoOptions(
        scenario="six_axis",
        headless=True,
        frame_delay_s=0.0,
        timeout_s=180,
        hold_window=True,
        interactive_hold=True,
        hold_duration_s=60.0,
        diagnostics_timeout_s=30,
        test_obstacle_hold_recovery=True,
    )
    report = None
    try:
        report = run(
            args.elf, args.output_dir, args.qemu, options,
            on_physics=impulse,
            prepare_motion=make_avoidance_preflight(args.library, "blocked"))
        recovery = report.get("obstacle_recovery_preflight") or {}
        checks = {
            "controlled_displacement_injected": impulse.injected,
            "QEMU_UART_passed": report.get("passed") is True,
            "original_C_APF_segments_settled": (
                report.get("segments_settled") ==
                (report.get("avoidance_preflight") or {}).get(
                    "cartesian_waypoints", 0)),
            "new_C_APF_path_collision_checked": recovery.get("passed") is True,
            "all_return_segments_settled": (
                report.get("obstacle_recovery_segments_settled") ==
                len(recovery.get("joint_waypoints_rad", [])) - 1
                and report.get("obstacle_recovery_segments_settled", 0) > 0),
            "hold_recovered": report.get("hold_recoveries", 0) >= 1,
            "hold_final_error_within_0p15deg": (
                report.get("hold_final_error_deg", 100) <= 0.15),
            "full_link_runtime_clearance_ok": (
                report.get("runtime_full_link_collision_free") is True),
            "GUI_mouse_picking_off": report.get("native_gui_picking") is False,
            "zero_C_faults": report.get("fault_counters") == {
                "controller_faults": 0,
                "planner_faults": 0,
                "feedback_stale_count": 0,
            },
        }
        verdict = {
            "scope": "Windows/Linux PC QEMU FreeRTOS UART PyBullet DIRECT; "
                     "no physical motors and no manual mouse picking",
            "passed": all(checks.values()),
            "checks": checks,
            "original_segments": report.get("segments_settled"),
            "recovery_segments": report.get("obstacle_recovery_segments_settled"),
            "recovery_planned_minimum_clearance_m": recovery.get(
                "minimum_measured_full_link_clearance_m"),
            "required_clearance_m": recovery.get(
                "minimum_full_link_clearance_m"),
            "hold_final_error_deg": report.get("hold_final_error_deg"),
            "fault_type": report.get("fault_type"),
            "raw_result": "result.json",
        }
    except Exception as exc:
        verdict = {
            "scope": "controlled C APF simulated return via real UART",
            "passed": False,
            "checks": {"controlled_displacement_injected": impulse.injected},
            "error": repr(exc),
            "raw_result": "result.json",
        }
    verdict_path.write_text(
        json.dumps(verdict, ensure_ascii=False, indent=2),
        encoding="utf-8")
    print(json.dumps(verdict, ensure_ascii=False, indent=2), flush=True)
    if not verdict["passed"]:
        raise SystemExit("CONTROLLED C APF HOLD RECOVERY FAIL: see "
                         + str(verdict_path))
    print("CONTROLLED C APF HOLD RECOVERY ACTUAL QEMU UART PASS")



if __name__ == "__main__":
    main()
