"""Read-only 5cm placement diagnostic. Never sends UART or actuates a robot.

The scenario remains the single source for robot/obstacle geometry and safety.
Probe variants are *in-memory* only. The experiment JSON must be updated
explicitly and retested with real QEMU UART before a placement is accepted.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/pybullet_ur5/src"))

import pybullet as physics
import qemu_uart_receiver as wire
from stage4_dynamic_closed_loop import START_RAW, URDF, joint_indices
from stage5_avoidance_uart_gate import load_scene, make_avoidance_preflight
from stage5_avoidance_preflight import RejectedRoute


def scan(library: Path, scene_case: str) -> list[dict]:
    base, policy = load_scene(scene_case)
    if not base.get("automatic_bypass", False):
        raise ValueError("The placement diagnostic requires automatic bypass")
    offset = base["obstacle"].get("center_offset_ee_mm")
    if offset is None:
        raise ValueError("The placement diagnostic requires an EE-relative box")
    half = base["obstacle"]["half_extents_m"]
    edge_mm = 2 * half[0] * 1000.0
    half_edge_mm = edge_mm / 2.0
    # Sample placements around the original blocked-line fixture, not
    # robot/control defaults. Each remains subject to direct-blocking checks.
    x_coords = [offset[0] - edge_mm, offset[0] - half_edge_mm,
                offset[0], offset[0] + half_edge_mm,
                offset[0] + edge_mm]
    y_coords = [offset[1], offset[1] + half_edge_mm / 2,
                offset[1] - half_edge_mm / 2]
    rows = []
    client = physics.connect(physics.DIRECT)
    if client < 0:
        raise RuntimeError("Could not start Bullet DIRECT")
    try:
        for x in x_coords:
            for y in y_coords:
                physics.resetSimulation(physicsClientId=client)
                arm = physics.loadURDF(str(URDF), useFixedBase=True,
                                       physicsClientId=client)
                joints = joint_indices(physics, arm)
                for index, raw in zip(joints, START_RAW):
                    physics.resetJointState(
                        arm, index, math.radians(raw / wire.JOINT_ANGLE_SCALE),
                        physicsClientId=client)
                trial = copy.deepcopy(base)
                trial["obstacle"]["center_offset_ee_mm"] = [x, y, offset[2]]
                entry = {"center_offset_ee_mm": [x, y, offset[2]]}
                try:
                    with patch("stage5_avoidance_uart_gate.load_scene",
                               return_value=(trial, policy)):
                        preflight = make_avoidance_preflight(library, scene_case)
                        report = preflight(physics, arm, joints, client)
                    entry.update(passed=True,
                                 candidate_count=report.get("bypass_candidates_attempted"),
                                 minimum_planned_full_link_clearance_m=report.get(
                                     "minimum_planned_full_link_clearance_m"))
                except RejectedRoute as err:
                    entry.update(passed=False, reason=str(err)[:350])
                rows.append(entry)
                print(json.dumps(entry, ensure_ascii=False), flush=True)
    finally:
        physics.disconnect(physicsClientId=client)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--scene", default="blocked_large")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if not args.library.is_file():
        parser.error("The production C APF/IK shared library is missing")
    rows = scan(args.library, args.scene)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"placement scan: {sum(1 for row in rows if row['passed'])}/"
          f"{len(rows)} geometry placements passed preflight (not motion)")
    

if __name__ == "__main__":
    main()
