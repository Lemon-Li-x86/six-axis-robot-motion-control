"""Safe bounded Stage5 GUI jog: reject BEFORE pose mutation; one-shot UI."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))

from stage5_safe_gui_jog import SafeGuiJog


class ButtonBullet:
    def __init__(self):
        self.angles = [math.radians(v) for v in [40, -67, 94, -16, 15, 14]]
        self.widgets = {}
        self.next_id = 10
        self.resets = []

    def addUserDebugParameter(self, label, lo, hi, initial, physicsClientId):
        self.next_id += 1
        self.widgets[self.next_id] = 0
        return self.next_id

    def readUserDebugParameter(self, widget, physicsClientId):
        return self.widgets[widget]

    def getJointState(self, robot, joint, physicsClientId):
        return self.angles[joint], 0.0, None, None

    def resetJointState(self, robot, joint, targetValue, targetVelocity,
                        physicsClientId):
        self.resets.append((joint, targetValue))
        self.angles[joint] = targetValue

    def click(self, widget):
        self.widgets[widget] += 1


def jog(physics=None, limit=math.pi, clearance=0.01):
    physics = physics or ButtonBullet()
    policy = SafeGuiJog(
        physics, 0, range(6), [1], 0,
        [(-limit, limit) for _ in range(6)],
        maximum_joint_step_rad=math.radians(1),
        minimum_clearance_m=clearance,
        maximum_samples=100)
    return physics, policy


class GuardedJogTests(unittest.TestCase):
    def test_rejects_outside_joint_limit_before_path_gate_or_pose_change(self):
        p, ctl = jog(limit=math.radians(40.5))
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path") as gate:
            ctl(p, 0, list(range(6)), 0.0, True, False)
            gate.assert_not_called()
        self.assertEqual(p.resets, [])
        self.assertEqual(ctl.events[0]["reason"], "GUI_joint_limit")
        self.assertEqual(ctl.applied, 0)
        self.assertEqual(ctl.refused, 1)

    def test_rejects_collision_without_pose_edit_then_accepts_safe_button(self):
        p, ctl = jog()
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value={"passed": False, "collision": {
                       "robot_link_index": 5, "signed_distance_m": 0.0098,
                   }}) as gate:
            ctl(p, 0, list(range(6)), 0.0, True, False)
            self.assertAlmostEqual(
                gate.call_args.kwargs["clearance_m"], 0.012)
        self.assertFalse(p.resets)
        self.assertEqual(ctl.applied, 0)
        self.assertEqual(ctl.events[-1]["reason"], "full_arm_clearance")
        p.click(ctl.buttons[-1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value={"passed": True,
                                 "sampled_states": 3,
                                 "minimum_measured_clearance_m": 0.026}):
            ctl(p, 0, list(range(6)), 0.01, True, False)
        self.assertEqual(ctl.applied, 1)
        self.assertAlmostEqual(math.degrees(p.angles[0]), 38)
        self.assertEqual(len(p.resets), 1)
        self.assertEqual(ctl.events[-1]["event"], "safe_gui_jog_applied")

    def test_early_button_is_consumed_and_not_deferred_into_hold(self):
        p, ctl = jog()
        p.click(ctl.buttons[1])
        ctl(p, 0, list(range(6)), 0, False, False)
        self.assertEqual(ctl.events[-1]["reason"],
                         "not_in_idle_hold_or_already_used")
        with patch("stage5_safe_gui_jog.validate_joint_path") as gate:
            ctl(p, 0, list(range(6)), 0.01, True, False)
            gate.assert_not_called()
        self.assertEqual(ctl.applied, 0)

    def test_no_second_jog_during_or_after_first_recovery(self):
        p, ctl = jog()
        p.click(ctl.buttons[-1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value={"passed": True, "sampled_states": 3,
                                 "minimum_measured_clearance_m": .026}):
            ctl(p, 0, list(range(6)), 0, True, False)
        self.assertEqual(ctl.applied, 1)
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path") as gate:
            ctl(p, 0, list(range(6)), .01, True, False)
            gate.assert_not_called()
        self.assertEqual(ctl.applied, 1)
        self.assertEqual(ctl.events[-1]["reason"],
                         "not_in_idle_hold_or_already_used")

class PyBulletDebugButtons:
    """DIRECT uses real contact geometry but synthetic debug widget events."""
    def __init__(self, real):
        self.p = real
        self.buttons = {}
        self.id = 10

    def __getattr__(self, name):
        return getattr(self.p, name)

    def addUserDebugParameter(self, label, low, high, initial, physicsClientId):
        self.id += 1
        self.buttons[self.id] = 0
        return self.id

    def readUserDebugParameter(self, widget, physicsClientId):
        return self.buttons[widget]

    def click(self, widget):
        self.buttons[widget] += 1


class RealPhysicsJogGateTests(unittest.TestCase):
    def test_actual_pybullet_full_link_preview_rejects_occupied_space(self):
        try:
            import pybullet as p
            from stage4_dynamic_closed_loop import URDF, START_RAW, joint_indices
        except ImportError:
            self.skipTest("Real PyBullet not installed")
        cid = p.connect(p.DIRECT)
        self.assertGreaterEqual(cid, 0)
        try:
            robot = p.loadURDF(str(URDF), useFixedBase=True, physicsClientId=cid)
            joints = joint_indices(p, robot)
            for joint, angle in zip(joints, START_RAW):
                p.resetJointState(
                    robot, joint, math.radians(angle / 100),
                    physicsClientId=cid)
            # Make a real obstacle at the wrist link, rather than mocking the
            # collision rejection; any jog from this unsafe current pose fails.
            obstacle_at = p.getLinkState(
                robot, joints[4], computeForwardKinematics=True,
                physicsClientId=cid)[4]
            shape = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[.02] * 3,
                physicsClientId=cid)
            obstacle = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=shape,
                basePosition=obstacle_at, physicsClientId=cid)
            proxy = PyBulletDebugButtons(p)
            limits = [(
                max(-math.pi, p.getJointInfo(
                    robot, j, physicsClientId=cid)[8]),
                min(math.pi, p.getJointInfo(
                    robot, j, physicsClientId=cid)[9])
            ) for j in joints]
            control = SafeGuiJog(
                proxy, robot, joints, [obstacle], cid, limits,
                maximum_joint_step_rad=math.radians(3),
                minimum_clearance_m=.01, maximum_samples=100)
            before = [p.getJointState(robot, j, physicsClientId=cid)[0]
                      for j in joints]
            proxy.click(control.buttons[1])
            control(proxy, robot, joints, 0, True, False)
            after = [p.getJointState(robot, j, physicsClientId=cid)[0]
                     for j in joints]
            self.assertEqual(control.applied, 0)
            self.assertEqual(control.events[-1]["reason"],
                             "full_arm_clearance")
            self.assertEqual(before, after)
        finally:
            p.disconnect(cid)


if __name__ == "__main__":
    unittest.main()
