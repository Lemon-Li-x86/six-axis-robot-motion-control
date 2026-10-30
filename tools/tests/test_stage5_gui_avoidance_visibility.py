"""Guard actual Bullet obstacle visibility and strict runtime safety in Stage5 GUI."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))

from stage4_dynamic_closed_loop import URDF, joint_indices  # noqa: E402
from stage5_qemu_uart_visual_demo import (  # noqa: E402
    Stage5RuntimeClearanceError,
    enforce_runtime_obstacle_clearance,
    draw_avoidance_gui_evidence,
)


class FakePhysics:
    def __init__(self, distance=0.009787):
        self.distance = distance
        self.performed = 0

    def performCollisionDetection(self):
        self.performed += 1

    def getNumJoints(self, body):
        return 7

    def getClosestPoints(self, arm, obstacle, clearance, *, linkIndexA, linkIndexB):
        if linkIndexA == 5 and obstacle == 1 and self.distance <= clearance:
            contact = [None] * 9
            contact[8] = self.distance
            return [contact]
        return []


class EvidenceDrawing:
    def __init__(self):
        self.lines = []
        self.texts = []

    def addUserDebugLine(self, first, last, color, **kwargs):
        self.lines.append((list(first), list(last), tuple(color)))

    def addUserDebugText(self, text, position, **kwargs):
        self.texts.append((text, list(position)))


class Stage5GuiAvoidanceSafetyTests(unittest.TestCase):
    def test_hold_drag_clearance_violation_remains_fatal_with_actual_distance(self):
        physics = FakePhysics(0.009787)
        with self.assertRaises(Stage5RuntimeClearanceError) as caught:
            enforce_runtime_obstacle_clearance(
                physics, 0, [1], 0.010, "interactive_hold")
        fault = caught.exception
        self.assertEqual(fault.phase, "interactive_hold")
        self.assertEqual(fault.link_index, 5)
        self.assertEqual(fault.obstacle_id, 1)
        self.assertAlmostEqual(fault.distance_m, 0.009787)
        self.assertAlmostEqual(fault.required_m, 0.010)
        self.assertEqual(physics.performed, 1)
        self.assertIn("clearance violation", str(fault))

    def test_motion_clearance_violation_is_not_hidden(self):
        with self.assertRaises(Stage5RuntimeClearanceError) as caught:
            enforce_runtime_obstacle_clearance(
                FakePhysics(-0.002), 0, [1], 0.010, "planned_motion")
        self.assertEqual(caught.exception.phase, "planned_motion")
        self.assertLess(caught.exception.distance_m, 0)

    def test_strict_threshold_is_unchanged(self):
        safe = FakePhysics(0.010001)
        enforce_runtime_obstacle_clearance(
            safe, 0, [1], 0.010, "interactive_hold")
        self.assertEqual(safe.performed, 1)

    def test_rejects_invalid_clearance_without_touching_physics(self):
        physics = FakePhysics()
        with self.assertRaises(ValueError):
            enforce_runtime_obstacle_clearance(
                physics, 0, [1], float("nan"), "planned_motion")
        self.assertEqual(physics.performed, 0)

    def test_overlay_draws_direct_route_and_C_planned_waypoints(self):
        renderer = EvidenceDrawing()
        plan = {
            "cartesian_start_m": [0.5, 0.1, 0.4],
            "cartesian_goal_m": [0.32, 0.1, 0.4],
            "obstacle_center_m": [0.41, 0.1, 0.4],
            "direct_cartesian_segment_blocked": True,
            "cartesian_route_mm": [[410, 100, 490], [320, 100, 400]],
        }
        draw_avoidance_gui_evidence(renderer, plan)
        self.assertEqual(len(renderer.lines), 3)
        self.assertEqual(renderer.lines[0][0], plan["cartesian_start_m"])
        self.assertEqual(renderer.lines[0][1], plan["cartesian_goal_m"])
        self.assertEqual(renderer.lines[1][1], [0.41, 0.1, 0.49])
        self.assertEqual(renderer.lines[2][1], plan["cartesian_goal_m"])
        self.assertTrue(any("OBSTACLE" in text for text, _ in renderer.texts))

    def test_blocked_fixture_has_visible_shape_matching_real_collision_body(self):
        try:
            import pybullet as p
        except ImportError:
            self.skipTest("PyBullet not installed in non-physics test job")
        from stage5_avoidance_uart_gate import make_avoidance_preflight, load_scene
        cid = p.connect(p.DIRECT)
        self.assertGreaterEqual(cid, 0)
        try:
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            original_scene, _ = load_scene("blocked")
            sample_plan = {
                "joint_waypoints_rad": [tuple([0.0] * 6), tuple([0.0] * 6)],
                "static_collision_clearance_m":
                    original_scene["minimum_full_link_clearance_mm"] / 1000.0,
                "cartesian_route_mm": [[0.0, 0.0, 0.0]],
            }
            with patch("stage5_avoidance_uart_gate._direct_is_blocked",
                       return_value=True), patch(
                "stage5_avoidance_uart_gate.preflight_joint_avoidance",
                return_value=sample_plan):
                prepare = make_avoidance_preflight(Path("unused-c-library"),
                                                    "blocked")
                report = prepare(p, arm, joints, cid)
            obstacle = report["obstacle_ids"][0]
            visual = p.getVisualShapeData(obstacle, physicsClientId=cid)
            self.assertTrue(visual, "Red blocker must be visible in native GUI")
            self.assertEqual(visual[0][2], p.GEOM_BOX)
            self.assertGreater(visual[0][7][0], 0.8)
            self.assertLess(visual[0][7][1], 0.4)
            self.assertTrue(report["direct_cartesian_segment_blocked"])
            self.assertEqual(report["obstacle_half_extents_m"], (0.01,)*3)
            lower, upper = p.getAABB(obstacle, physicsClientId=cid)
            center = [(lo + hi) / 2 for lo, hi in zip(lower, upper)]
            for actual, expected in zip(center, report["obstacle_center_m"]):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertAlmostEqual(
                math.dist(report["cartesian_start_m"],
                          report["obstacle_center_m"]), 0.09, places=5)
        finally:
            p.disconnect(cid)


if __name__ == "__main__":
    unittest.main()
