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
from stage5_avoidance_uart_gate import make_avoidance_preflight


class OneSafeJointDisplacement:
    def __init__(self):
        self.injected = False
        self.displacement_deg = 2.0

    def __call__(self, bullet, arm, joints, time_s, done, ending):
        if done and not ending and not self.injected:
            joint = joints[0]
            angle = bullet.getJointState(arm, joint)[0]
            bullet.resetJointState(
                arm, joint,
                targetValue=angle + math.radians(self.displacement_deg),
                targetVelocity=0.0)
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

    impulse = OneSafeJointDisplacement()
    options = DemoOptions(
        scenario="six_axis",
        headless=True,
        frame_delay_s=0.0,
        timeout_s=180,
        hold_window=True,
        interactive_hold=True,
        hold_duration_s=15.0,
        diagnostics_timeout_s=30,
        test_obstacle_hold_recovery=True,
    )
    report = run(
        args.elf, args.output_dir, args.qemu, options,
        on_physics=impulse,
        prepare_motion=make_avoidance_preflight(args.library, "blocked"))
    assert impulse.injected, "The controlled disturbance was never applied"
    assert report["passed"], report
    assert report["segments_settled"] == 4, report
    assert report["obstacle_recovery_preflight"]["passed"], report
    assert report["obstacle_recovery_segments_settled"] >= 1, report
    assert report["hold_recoveries"] >= 1, report
    assert report["hold_final_error_deg"] <= 0.15, report
    assert report["runtime_full_link_collision_free"] is True, report
    assert not report["native_gui_picking"], report
    assert report["fault_counters"] == {
        "controller_faults": 0, "planner_faults": 0, "feedback_stale_count": 0
    }, report
    print("CONTROLLED C APF HOLD RECOVERY ACTUAL QEMU UART PASS")


if __name__ == "__main__":
    main()
