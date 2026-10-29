"""Offline blocked-scene feasibility scan for Stage5/Stage6 fixture design.

This is NOT a runtime planner. It exercises the existing production C APF,
production C UR5 IK and actual PyBullet swept-link preflight. It never sends
UART commands or mutates robot model/control constants. Its JSON evidence
lets an engineer choose ONE explicit reproducible scene after inspection.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "simulation/pybullet_ur5/src"
sys.path.insert(0, str(SRC))

from stage4_dynamic_closed_loop import START_RAW, DEFAULT_JSON, URDF, joint_indices
from stage5_static_obstacle_probe import Config, AABB, point
from stage5_avoidance_preflight import preflight_joint_avoidance, RejectedRoute
from stage5_avoidance_uart_gate import _direct_is_blocked, load_scene


def candidate_offsets():
    """A bounded, deterministic experimental workspace search, in mm.

    The list is a *diagnostic* set of target displacements, not learned
    "safe" paths, a robot model, or a production motion primitive.
    """
    return (
        (-120, 0, 0), (-180, 0, 0), (-220, 0, 0),
        (120, 0, 0), (180, 0, 0), (220, 0, 0),
        (0, -120, 0), (0, -180, 0), (0, -220, 0),
        (0, 120, 0), (0, 180, 0),
        (0, 0, 120), (0, 0, 180), (0, 0, -120),
        (-120, -80, 0), (-160, -80, 0),
        (-80, 0, 120), (0, -120, 100), (0, -160, 100),
        (100, -100, 0), (-100, 100, 0),
    )


def waypoint_options(delta):
    """Select side corridors in Cartesian space, not raw joint overrides."""
    dx, dy, dz = delta
    normal = math.hypot(dx, dy)
    options = []
    if normal > 0:
        side = (-dy / normal, dx / normal, 0.0)
        for distance in (80.0, -80.0, 130.0, -130.0):
            options.append(tuple(
                0.5 * v + distance * d
                for v, d in zip(delta, side)))
    for rise in (90.0, -90.0):
        options.append((dx * 0.5, dy * 0.5, dz * 0.5 + rise))
    return options


def scan(library: Path, output_file: Path, *, limit: int = 126):
    import pybullet as p
    if not library.is_file():
        raise FileNotFoundError(library)
    scene, policy = load_scene("blocked")
    apf = Config(*(float(scene["apf"][name]) for name, _ in Config._fields_))
    motion = json.loads(DEFAULT_JSON.read_text(encoding="utf-8"))["stage4_motion"]
    c = p.connect(p.DIRECT)
    if c < 0:
        raise RuntimeError("DIRECT physics unavailable")
    results = []
    attempted = 0
    try:
        robot = p.loadURDF(str(URDF), useFixedBase=True, physicsClientId=c)
        joints = joint_indices(p, robot)
        for joint, raw in zip(joints, START_RAW):
            p.resetJointState(robot, joint, math.radians(raw / 100.0),
                              physicsClientId=c)
        ee = next(i for i in range(p.getNumJoints(robot, physicsClientId=c))
                  if p.getJointInfo(robot, i, physicsClientId=c)[12]
                  == b"ee_link")
        start_m = p.getLinkState(
            robot, ee, computeForwardKinematics=True,
            physicsClientId=c)[4]
        start_mm = tuple(x*1000 for x in start_m)
        for delta in candidate_offsets():
            for via in waypoint_options(delta):
                if attempted >= limit:
                    break
                attempted += 1
                goal = tuple(start_mm[i]+delta[i] for i in range(3))
                midpoint = tuple(
                    start_m[i]+delta[i]/2000 for i in range(3))
                # A small box at the center of the nominal EE straight path:
                # its actual world AABB comes only from PyBullet.
                half = (0.010, 0.010, 0.010)
                shape = p.createCollisionShape(
                    p.GEOM_BOX, halfExtents=half, physicsClientId=c)
                obstacle = p.createMultiBody(
                    baseMass=0, baseCollisionShapeIndex=shape,
                    basePosition=midpoint, physicsClientId=c)
                entry = {
                    "goal_offset_mm": delta,
                    "via_offset_mm": via,
                    "obstacle_center_offset_mm": tuple(v*0.5 for v in delta),
                    "half_extents_m": half,
                    "number": attempted,
                }
                try:
                    lo, hi = p.getAABB(obstacle, physicsClientId=c)
                    bounds = AABB(point([1000*x for x in lo]),
                                  point([1000*x for x in hi]))
                    if not _direct_is_blocked(
                            library, start_mm, goal, bounds,
                            apf.clearance_mm):
                        entry.update(stage="direct_obstruction", accepted=False)
                        continue
                    p.performCollisionDetection(physicsClientId=c)
                    start_contacts = []
                    for link in range(-1,p.getNumJoints(robot, physicsClientId=c)):
                        start_contacts.extend(p.getClosestPoints(
                            robot, obstacle, scene["minimum_full_link_clearance_mm"]/1000.0,
                            linkIndexA=link, linkIndexB=-1,
                            physicsClientId=c))
                    if start_contacts:
                        entry.update(
                            stage="initial_collision", accepted=False,
                            link=min(start_contacts,key=lambda pt:pt[8])[3],
                            signed_distance_m=min(pt[8] for pt in start_contacts))
                        continue
                    result = preflight_joint_avoidance(
                        library, p, robot, [obstacle], joints, ee, goal,
                        apf_config=apf,
                        max_velocity_deg_s=motion["max_velocity_deg_s"],
                        max_acceleration_deg_s2=motion["max_acceleration_deg_s2"],
                        max_fk_position_error_mm=None,
                        max_fk_rotation_element_error=None,
                        waypoint_capacity=scene["waypoint_capacity"],
                        max_joint_step_rad=policy["max_joint_step_rad"],
                        max_collision_samples=policy["max_collision_samples"],
                        physics_client_id=c,
                        via_points_mm=[tuple(
                            start_mm[i]+via[i] for i in range(3))],
                        minimum_full_link_clearance_m=(
                            scene["minimum_full_link_clearance_mm"]/1000.0))
                    entry.update(
                        stage="full_arm_safe", accepted=True,
                        cartesian_waypoints=result["cartesian_waypoints"],
                        samples=result["sampled_full_arm_states"],
                        planned_s=result["total_minimum_rest_to_rest_duration_s"])
                except RejectedRoute as exc:
                    entry.update(stage="preflight", accepted=False,
                                 reason=str(exc))
                finally:
                    p.removeBody(obstacle, physicsClientId=c)
                    results.append(entry)
            if attempted >= limit:
                break
        summary = {
            "scope": "offline_fixture_scan_no_uart_no_actuation",
            "scanned": attempted,
            "successes": sum(bool(r.get("accepted")) for r in results),
            "start_mm": start_mm,
            "full_link_clearance_m": scene["minimum_full_link_clearance_mm"]/1000,
            "entries": results,
        }
        output_file.parent.mkdir(parents=True, exist_ok=True)
        output_file.write_text(json.dumps(summary, indent=2),
                               encoding="utf-8")
        print("Searched:", attempted, "Safe routes:", summary["successes"])
        for item in results:
            if item.get("accepted"):
                print("SAFE FIXTURE:", json.dumps(item))
        for item in results[:10]:
            if not item.get("accepted"):
                print("REJECT:", json.dumps(item))
        return summary
    finally:
        p.disconnect(c)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=126)
    args = parser.parse_args()
    if not 1 <= args.limit <= 126:
        parser.error("limit must be between 1 and 126")
    scan(args.library, args.output, limit=args.limit)


if __name__ == "__main__":
    main()
