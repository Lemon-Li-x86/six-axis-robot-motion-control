"""GUI-only single-turn joint stops: never rewrite firmware or URDF travel."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))
from stage4_dynamic_closed_loop import URDF, joint_indices  # noqa: E402
from stage5_qemu_uart_visual_demo import (  # noqa: E402
    configure_native_urdf_joint_stops,
    enforce_gui_joint_hard_stops,
    closest_physical_motor_targets,
    GUI_JOINT_TRAVEL_MODES,
    DEFAULT_GUI_JOINT_TRAVEL_MODE,
)


class DummyBullet:
    JOINT_REVOLUTE = 0

    def __init__(self):
        self.stops = {}
        self.positions = [0.0] * 6

    def getJointInfo(self, arm, joint):
        del arm
        # Same ranges as the repository's URDF. Only used to probe policy;
        # physical test loads the actual URDF instead.
        low, high = (-math.pi, math.pi) if joint == 2 else (-2*math.pi, 2*math.pi)
        values = [None] * 12
        values[2], values[8], values[9], values[10] = (
            self.JOINT_REVOLUTE, low, high, 150.0)
        return values

    def changeDynamics(self, arm, joint, **kwargs):
        self.stops[joint] = kwargs


class GuiJointTravelUnitTests(unittest.TestCase):
    def test_single_turn_is_default_GUI_only_policy(self):
        self.assertEqual(DEFAULT_GUI_JOINT_TRAVEL_MODE, "single-turn")
        self.assertEqual(set(GUI_JOINT_TRAVEL_MODES), {"single-turn", "urdf"})

    def test_default_original_api_retains_full_model_limits(self):
        physics = DummyBullet()
        original = configure_native_urdf_joint_stops(physics, 0, list(range(6)))
        self.assertAlmostEqual(original[0][1], 2 * math.pi)
        self.assertAlmostEqual(original[2][1], math.pi)
        self.assertAlmostEqual(physics.stops[0]["jointUpperLimit"], 2*math.pi)

    def test_one_turn_policy_enforces_each_axis_without_extending_urdf(self):
        physics = DummyBullet()
        bounds = configure_native_urdf_joint_stops(
            physics, 0, list(range(6)), travel_mode="single-turn")
        self.assertEqual(len(bounds), 6)
        for joint, (low, high) in enumerate(bounds):
            self.assertAlmostEqual(low, -math.pi, places=10, msg=f"J{joint+1}")
            self.assertAlmostEqual(high, math.pi, places=10, msg=f"J{joint+1}")
            self.assertAlmostEqual(
                physics.stops[joint]["jointUpperLimit"], high)
        # A canonical zero servo target can no longer be lifted to +360deg.
        target = closest_physical_motor_targets(
            [0] * 6, [35900, 0, 0, 0, 0, 0], bounds)
        self.assertAlmostEqual(target[0], 0.0, places=10)

    def test_unknown_gui_policy_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "travel policy"):
            configure_native_urdf_joint_stops(
                DummyBullet(), 0, list(range(6)), travel_mode="unlimited")


class RealUrdfGuiTravelTests(unittest.TestCase):
    def test_one_turn_limits_and_clamp_on_real_pybullet_robot(self):
        try:
            import pybullet as p
        except ImportError:
            self.skipTest("PyBullet not installed; unit tests still run")
        client = p.connect(p.DIRECT)
        self.assertGreaterEqual(client, 0)
        try:
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            limits = configure_native_urdf_joint_stops(
                p, arm, joints, travel_mode="single-turn")
            self.assertEqual(len(limits), 6)
            for i, (low, high) in enumerate(limits):
                self.assertAlmostEqual(low, -math.pi, places=5)
                self.assertAlmostEqual(high, math.pi, places=5)
                # Simulates physically overdriving a picked link; the active
                # GUI guard must act on each link, not merely on J1.
                p.resetJointState(
                    arm, joints[i],
                    targetValue=(high + math.radians(12) if i % 2 == 0
                                 else low - math.radians(12)),
                    targetVelocity=2.0)
            corrections = enforce_gui_joint_hard_stops(p, arm, joints, limits)
            self.assertEqual({e["joint_index"] for e in corrections}, set(range(6)))
            for i, joint in enumerate(joints):
                angle, speed = p.getJointState(arm, joint)[:2]
                self.assertAlmostEqual(angle, limits[i][1 if i % 2 == 0 else 0])
                self.assertAlmostEqual(speed, 0.0)
            for _ in range(15):
                p.stepSimulation()
                enforce_gui_joint_hard_stops(p, arm, joints, limits)
            for i, joint in enumerate(joints):
                angle = p.getJointState(arm, joint)[0]
                self.assertLessEqual(angle, limits[i][1] + 1e-7)
                self.assertGreaterEqual(angle, limits[i][0] - 1e-7)
        finally:
            p.disconnect(client)


if __name__ == "__main__":
    unittest.main()
