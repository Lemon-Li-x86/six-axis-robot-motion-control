"""Stage5 first vertical slice: UR5 + static cuboid in PyBullet DIRECT.

Build stage5_apf_bridge using tests/avoidance/CMakeLists.txt. Extract the
ACTUAL PyBullet obstacle AABB and pass it to the exact production C APF.
Probe one conservative Cartesian end-effector step; the arm does NOT move,
there is NO IK, full robot link collision check or actuated closed-loop here.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "simulation/pybullet_ur5/models/ur5/urdf/ur5.urdf"


class Point(ctypes.Structure):
    _fields_ = [("mm", ctypes.c_double * 3)]


class AABB(ctypes.Structure):
    _fields_ = [("min_corner", Point), ("max_corner", Point)]


class Config(ctypes.Structure):
    _fields_ = [
        ("attraction_gain", ctypes.c_double),
        ("repulsion_gain", ctypes.c_double),
        ("influence_distance_mm", ctypes.c_double),
        ("clearance_mm", ctypes.c_double),
        ("maximum_step_mm", ctypes.c_double),
        ("goal_tolerance_mm", ctypes.c_double),
    ]


class StepInfo(ctypes.Structure):
    _fields_ = [
        ("nearest_surface_distance_mm", ctypes.c_double),
        ("chosen_step_mm", ctypes.c_double),
        ("reached_goal", ctypes.c_uint),
    ]


def point(coordinates):
    return Point((ctypes.c_double * 3)(*coordinates))


def probe(library: Path, output_dir: Path):
    import pybullet as p

    if not library.is_file():
        raise FileNotFoundError(library)

    c = ctypes.CDLL(str(library.resolve()))
    c.robot_apf_step.restype = ctypes.c_int
    c.robot_apf_step.argtypes = [
        ctypes.POINTER(Point), ctypes.POINTER(Point), ctypes.POINTER(AABB),
        ctypes.c_size_t, ctypes.POINTER(Config), ctypes.POINTER(Point),
        ctypes.POINTER(StepInfo),
    ]

    client = p.connect(p.DIRECT)
    if client < 0:
        raise RuntimeError("PyBullet DIRECT unavailable")
    try:
        robot = p.loadURDF(str(URDF), useFixedBase=True)
        end_link = next(
            (index for index in range(p.getNumJoints(robot))
             if p.getJointInfo(robot, index)[12] == b"ee_link"), None
        )
        if end_link is None:
            raise RuntimeError("UR5 URDF has no ee_link")

        origin_m = p.getLinkState(
            robot, end_link, computeForwardKinematics=True)[4]
        # The scene fixture is deliberately local to this single probe.
        # Never copy these box dimensions into Robot Model or firmware config.
        half_extents_m = [0.035, 0.05, 0.04]
        centre_m = [
            origin_m[0] + 0.15, origin_m[1], origin_m[2],
        ]
        shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=half_extents_m)
        obstacle = p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=shape,
            basePosition=centre_m)
        minimum_m, maximum_m = p.getAABB(obstacle)
        bounds = AABB(
            point([x * 1000.0 for x in minimum_m]),
            point([x * 1000.0 for x in maximum_m]),
        )

        current = point([x * 1000.0 for x in origin_m])
        goal = point([
            current.mm[0] + 300.0,
            current.mm[1] + 120.0,
            current.mm[2],
        ])
        # Test-only APF policy, passed via the production C API.
        config = Config(1.0, 50000.0, 150.0, 10.0, 20.0, 2.0)
        next_point = Point()
        details = StepInfo()
        status = c.robot_apf_step(
            ctypes.byref(current), ctypes.byref(goal),
            ctypes.byref(bounds), 1, ctypes.byref(config),
            ctypes.byref(next_point), ctypes.byref(details),
        )
        if status != 0:
            raise RuntimeError(f"C APF rejected the test step: {status}")

        actual_step = math.dist(
            list(current.mm), list(next_point.mm))
        if not math.isfinite(actual_step) or actual_step > 20.0 + 1e-7:
            raise AssertionError("C APF exceeded the configured step limit")

        report = {
            "scope": "single_unactuated_end_effector_point_step",
            "algorithm_source": "firmware/freertos_demo/algorithms/avoidance/potential_field.c",
            "urdf": str(URDF.relative_to(ROOT)),
            "start_mm": list(current.mm),
            "goal_mm": list(goal.mm),
            "obstacle_aabb_mm": {
                "min": list(bounds.min_corner.mm),
                "max": list(bounds.max_corner.mm),
            },
            "next_mm": list(next_point.mm),
            "actual_step_mm": actual_step,
            "nearest_surface_distance_mm": details.nearest_surface_distance_mm,
            "reached_goal": bool(details.reached_goal),
            "passed": True,
            "not_yet_tested": [
                "6-link collision avoidance", "continuous APF route",
                "IK/trajectory integration", "UART",
                "FreeRTOS execution", "physical robot",
            ],
        }
    finally:
        p.disconnect(client)

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "stage5_c_apf_static_box_probe.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "build/stage5-avoidance")
    args = parser.parse_args()
    print(json.dumps(probe(args.library, args.output_dir), indent=2))


if __name__ == "__main__":
    main()
