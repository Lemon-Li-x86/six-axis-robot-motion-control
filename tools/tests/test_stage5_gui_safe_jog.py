"""Stage5 selectable guarded GUI jog: preflight both legs and re-arm after recovery."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))
from stage5_safe_gui_jog import SafeGuiJog, JOG_MAX_SUCCESSES, JOG_MIN_DEG


def valid_return():
    return {"passed": True, "minimum_full_link_clearance_m": .010,
            "minimum_measured_full_link_clearance_m": .026,
            "joint_waypoints_rad": [(0.,)*6, (.1,)*6]}


class ButtonBullet:
    def __init__(self):
        self.angles = [math.radians(v) for v in [40, -67, 94, -16, 15, 14]]
        self.velocities = [0.] * 6
        self.widgets = {}
        self.next_id = 10
        self.resets = []
        self.collisions = 0

    def addUserDebugParameter(self, label, lo, hi, initial, physicsClientId):
        self.next_id += 1
        self.widgets[self.next_id] = initial
        return self.next_id

    def readUserDebugParameter(self, widget, physicsClientId):
        return self.widgets[widget]

    def getJointState(self, robot, joint, physicsClientId):
        return self.angles[joint], self.velocities[joint], None, None

    def resetJointState(self, robot, joint, targetValue, targetVelocity,
                        physicsClientId):
        self.resets.append((joint, targetValue))
        self.angles[joint] = targetValue
        self.velocities[joint] = targetVelocity

    def performCollisionDetection(self, physicsClientId):
        self.collisions += 1

    def click(self, widget):
        self.widgets[widget] += 1


def jog(physics=None, limit=math.pi, preview_return=None):
    p = physics or ButtonBullet()
    validator = preview_return or Mock(side_effect=lambda: valid_return())
    policy = SafeGuiJog(
        p, 0, range(6), [1], 0,
        [(-limit, limit) for _ in range(6)],
        maximum_joint_step_rad=math.radians(1),
        minimum_clearance_m=.01, maximum_samples=100,
        preview_return=validator)
    return p, policy, validator


def safe_gate():
    return {"passed": True, "sampled_states": 10,
            "minimum_measured_clearance_m": .028}


class GuardedJogTests(unittest.TestCase):
    def test_click_receipt_survives_native_preflight_failure(self):
        p, ctl, planner = jog()
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   side_effect=RuntimeError("native call failed")):
            with self.assertRaisesRegex(RuntimeError, "native call failed"):
                ctl(p, 0, range(6), 0, True, False)
        self.assertEqual(ctl.events[-1]["event"], "safe_gui_jog_click_received")
        self.assertEqual(ctl.applied, 0)
        self.assertEqual(p.resets, [])
        planner.assert_not_called()

    def test_visible_completion_requires_real_recovery(self):
        p, ctl, planner = jog()
        self.assertIn("WAIT", ctl.status_text(False))
        self.assertIn("READY", ctl.status_text(True))
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value=safe_gate()):
            ctl(p, 0, range(6), 0, True, False)
        self.assertIn("BUSY", ctl.status_text(True))
        ctl.complete_recovery()
        self.assertIn("RETURN COMPLETE", ctl.status_text(True))
        self.assertIn("returned=1", ctl.status_text(True))

    def test_six_axis_selection_and_small_step(self):
        for selected in range(1, 7):
            with self.subTest(joint=selected):
                p, ctl, planner = jog()
                p.widgets[ctl.axis_widget] = selected
                p.widgets[ctl.step_widget] = .5
                p.click(ctl.buttons[1])
                with patch("stage5_safe_gui_jog.validate_joint_path",
                           return_value=safe_gate()) as gate:
                    ctl(p, 0, list(range(6)), 0, True, False)
                self.assertEqual(ctl.applied, 1)
                event = ctl.events[-1]
                self.assertEqual((event["joint"], event["axis"]),
                                 (selected, selected-1))
                self.assertEqual(event["displacement_deg"], .5)
                self.assertAlmostEqual(
                    math.degrees(p.angles[selected-1]),
                    [40,-67,94,-16,15,14][selected-1]+.5)
                self.assertEqual(gate.call_args.kwargs["clearance_m"], .012)
                self.assertEqual(
                    gate.call_args.kwargs["maximum_joint_step_rad"],
                    math.radians(.25))
                planner.assert_called_once()

    def test_ci_scripted_j6_uses_real_preflight_and_requires_uart_settlement(self):
        p, ctl, planner = jog()
        # GUI selection is J1, but CI's explicit scope is J6 at minimum step.
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value=safe_gate()) as gate:
            event = ctl.request_ci_scripted_j6_half_degree()
        self.assertEqual(event["joint"], 6)
        self.assertEqual(event["displacement_deg"], JOG_MIN_DEG)
        self.assertEqual(event["source"], "ci_scripted_native_gui_jog")
        self.assertEqual(ctl.applied, 1)
        self.assertEqual(gate.call_args.kwargs["clearance_m"], .012)
        planner.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "completed real C recovery"):
            ctl.request_ci_finish()
        ctl.complete_recovery()
        ctl.request_ci_finish()
        self.assertTrue(ctl.finish_requested)
        self.assertEqual(ctl.completed, 1)

    def test_ci_scripted_j6_rejected_when_collision_gate_fails(self):
        p, ctl, planner = jog()
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value={"passed": False, "collision": {"robot_link_index": 2}}):
            with self.assertRaisesRegex(RuntimeError, "refused"):
                ctl.request_ci_scripted_j6_half_degree()
        self.assertEqual(ctl.applied, 0)
        self.assertEqual(ctl.refused, 1)
        planner.assert_not_called()
        self.assertEqual(ctl.events[-1]["event"], "safe_gui_jog_rejected")

    def test_finish_without_jog_is_cancel_not_pass(self):
        p, ctl, planner = jog()
        p.click(ctl.finish_widget)
        ctl(p, 0, list(range(6)), 0, True, False)
        self.assertFalse(ctl.finish_requested)
        self.assertTrue(ctl.cancel_requested)
        self.assertEqual(ctl.events[-1]["event"],
                         "safe_gui_jog_cancel_requested")
        self.assertEqual(ctl.applied, 0)
        # No new motion can be started while finishing the no-jog verdict.
        p.click(ctl.buttons[1])
        ctl(p, 0, list(range(6)), 1, True, False)
        self.assertEqual(ctl.applied, 0)
        planner.assert_not_called()

    def test_finish_requires_completed_uart_recovery(self):
        p, ctl, planner = jog()
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value=safe_gate()):
            ctl(p, 0, list(range(6)), 1, True, False)
        self.assertEqual(ctl.applied, 1)
        p.click(ctl.finish_widget)
        ctl(p, 0, list(range(6)), 2, True, False)
        self.assertFalse(ctl.finish_requested)
        self.assertEqual(ctl.events[-1]["event"],
                         "safe_gui_jog_finish_rejected")
        ctl.complete_recovery()
        # The rejected finish was consumed; only a NEW click can finish.
        ctl(p, 0, list(range(6)), 3, True, False)
        self.assertFalse(ctl.finish_requested)
        p.click(ctl.finish_widget)
        ctl(p, 0, list(range(6)), 4, True, False)
        self.assertTrue(ctl.finish_requested)
        self.assertEqual(ctl.events[-1]["event"],
                         "safe_gui_jog_finish_requested")
        self.assertEqual(ctl.events[-1]["completed_recoveries"], 1)
        # A finish request cannot authorize another jog while the final
        # diagnostics are being collected.
        before = ctl.applied
        p.click(ctl.buttons[1])
        ctl(p, 0, list(range(6)), 5, True, False)
        self.assertEqual(ctl.applied, before)
        planner.assert_called_once()

    def test_finish_after_maximum_completed_recoveries(self):
        p, ctl, planner = jog()
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value=safe_gate()):
            for cycle in range(JOG_MAX_SUCCESSES):
                p.click(ctl.buttons[1])
                ctl(p, 0, range(6), cycle, True, False)
                ctl.complete_recovery()
        self.assertFalse(ctl.available)
        p.click(ctl.finish_widget)
        ctl(p, 0, range(6), 5, True, False)
        self.assertTrue(ctl.finish_requested)

    def test_joint_limit_rejects_before_gate_or_planner(self):
        p, ctl, planner = jog(limit=math.radians(40.5))
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path") as gate:
            ctl(p, 0, list(range(6)), 0, True, False)
            gate.assert_not_called()
        planner.assert_not_called()
        self.assertEqual(p.resets, [])
        self.assertEqual(ctl.events[-1]["reason"], "GUI_joint_limit")

    def test_realistic_block_prevents_all_mutation(self):
        p, ctl, planner = jog()
        p.click(ctl.buttons[-1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value={"passed": False, "collision": {
                       "robot_link_index": 5, "signed_distance_m": .0098,
                   }}):
            ctl(p, 0, list(range(6)), 0, True, False)
        self.assertEqual(ctl.applied, 0)
        self.assertEqual(p.resets, [])
        self.assertEqual(ctl.events[-1]["reason"], "full_arm_clearance")
        planner.assert_not_called()

    def test_unplannable_return_rolls_back_pose_and_rejects(self):
        p = ButtonBullet()
        original = p.angles[:]
        p, ctl, planner = jog(
            p, preview_return=Mock(side_effect=ValueError("C IK branch invalid")))
        p.widgets[ctl.axis_widget] = 4
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value=safe_gate()):
            ctl(p, 0, list(range(6)), 0, True, False)
        self.assertEqual(p.angles, original)
        self.assertEqual(ctl.applied, 0)
        self.assertEqual(ctl.events[-1]["reason"], "recovery_preflight_failed")
        self.assertGreaterEqual(p.collisions, 1)
        planner.assert_called_once()

    def test_early_click_not_deferred_into_hold(self):
        p, ctl, planner = jog()
        p.click(ctl.buttons[1])
        ctl(p, 0, range(6), 0, False, False)
        with patch("stage5_safe_gui_jog.validate_joint_path") as gate:
            ctl(p, 0, range(6), 1, True, False)
            gate.assert_not_called()
        self.assertEqual(ctl.applied, 0)
        planner.assert_not_called()
        self.assertEqual(ctl.events[-1]["reason"],
                         "not_idle_or_recovery_pending_or_limit")

    def test_multiple_recoveries_only_rearm_after_actual_completion(self):
        p, ctl, planner = jog()
        with patch("stage5_safe_gui_jog.validate_joint_path",
                   return_value=safe_gate()):
            for cycle in range(JOG_MAX_SUCCESSES):
                p.widgets[ctl.axis_widget] = cycle + 1
                p.click(ctl.buttons[1])
                ctl(p, 0, range(6), cycle, True, False)
                self.assertEqual(ctl.applied, cycle+1)
                self.assertFalse(ctl.available)
                p.click(ctl.buttons[-1])
                ctl(p, 0, range(6), cycle+.01, True, False)
                self.assertEqual(ctl.applied, cycle+1)
                self.assertEqual(ctl.events[-1]["reason"],
                                 "not_idle_or_recovery_pending_or_limit")
                ctl.complete_recovery()
                self.assertEqual(ctl.completed, cycle+1)
        self.assertFalse(ctl.available)
        p.click(ctl.buttons[1])
        ctl(p, 0, range(6), 10, True, False)
        self.assertEqual(ctl.applied, JOG_MAX_SUCCESSES)
        with self.assertRaises(RuntimeError):
            ctl.complete_recovery()

    def test_malformed_slider_never_authorizes_motion(self):
        p, ctl, planner = jog()
        p.widgets[ctl.axis_widget] = float("nan")
        p.click(ctl.buttons[1])
        with patch("stage5_safe_gui_jog.validate_joint_path") as gate:
            ctl(p, 0, range(6), 0, True, False)
            gate.assert_not_called()
        self.assertEqual(ctl.events[-1]["reason"], "invalid_selection")


class PyBulletDebugButtons:
    """Use real Bullet collision geometry; synthesize only GUI widget clicks."""
    def __init__(self, physics):
        self.real = physics
        self.buttons = {}
        self.next_id = 100

    def __getattr__(self, name):
        return getattr(self.real, name)

    def addUserDebugParameter(self, label, low, high, initial, physicsClientId):
        self.next_id += 1
        self.buttons[self.next_id] = initial
        return self.next_id

    def readUserDebugParameter(self, widget, physicsClientId):
        return self.buttons[widget]

    def click(self, widget):
        self.buttons[widget] += 1


class RealPhysicsJogGateTests(unittest.TestCase):
    def test_real_Bullet_link_collision_blocks_selected_axis(self):
        try:
            import pybullet as p
            from stage4_dynamic_closed_loop import URDF, START_RAW, joint_indices
        except ImportError:
            self.skipTest("PyBullet not installed")
        cid = p.connect(p.DIRECT)
        self.assertGreaterEqual(cid, 0)
        try:
            robot = p.loadURDF(str(URDF), useFixedBase=True, physicsClientId=cid)
            joints = joint_indices(p, robot)
            for j, raw in zip(joints, START_RAW):
                p.resetJointState(robot, j, math.radians(raw/100),
                                  physicsClientId=cid)
            at = p.getLinkState(robot, joints[4],
                                computeForwardKinematics=True,
                                physicsClientId=cid)[4]
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.02]*3,
                                           physicsClientId=cid)
            obstacle = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=shape,
                basePosition=at, physicsClientId=cid)
            proxy = PyBulletDebugButtons(p)
            limits = [(max(-math.pi,p.getJointInfo(robot,j,physicsClientId=cid)[8]),
                       min(math.pi,p.getJointInfo(robot,j,physicsClientId=cid)[9]))
                      for j in joints]
            validator = Mock(side_effect=lambda: valid_return())
            ctl = SafeGuiJog(
                proxy, robot, joints, [obstacle], cid, limits,
                maximum_joint_step_rad=math.radians(3),
                minimum_clearance_m=.01, maximum_samples=100,
                preview_return=validator)
            proxy.buttons[ctl.axis_widget] = 5
            before = [p.getJointState(robot,j,physicsClientId=cid)[0]
                      for j in joints]
            proxy.click(ctl.buttons[1])
            ctl(proxy, robot, joints, 0, True, False)
            after = [p.getJointState(robot,j,physicsClientId=cid)[0]
                     for j in joints]
            self.assertEqual(before, after)
            self.assertEqual(ctl.applied, 0)
            self.assertEqual(ctl.events[-1]["reason"], "full_arm_clearance")
            validator.assert_not_called()
        finally:
            p.disconnect(cid)


if __name__ == "__main__":
    unittest.main()
