"""Real PyBullet + production C APF/IK recovery from a displaced HOLD pose."""
from __future__ import annotations

import math
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation" / "pybullet_ur5" / "src"))

from stage4_dynamic_closed_loop import URDF, START_RAW, joint_indices
from stage5_avoidance_uart_gate import make_avoidance_preflight
from stage5_obstacle_hold_recovery import plan_obstacle_hold_recovery
from stage5_avoidance_preflight import RejectedRoute, CartesianGoalAlreadyReached
from stage5_collision_validation import validate_joint_path


class RealCRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import pybullet as p
        except ImportError:
            raise unittest.SkipTest("PyBullet unavailable")
        cls.p = p
        name = os.environ.get("STAGE5_APF_TEST_DLL")
        if not name or not Path(name).is_file():
            raise unittest.SkipTest("STAGE5_APF_TEST_DLL needed for production C APF/IK")
        cls.dll = Path(name).resolve()

    def setUp(self):
        p = self.p
        self.client = p.connect(p.DIRECT)
        self.assertGreaterEqual(self.client, 0)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        self.arm = p.loadURDF(
            str(URDF), useFixedBase=True, physicsClientId=self.client)
        self.joints = joint_indices(p, self.arm)
        for index, raw in zip(self.joints, START_RAW):
            p.resetJointState(
                self.arm, index, math.radians(raw/100),
                physicsClientId=self.client)
        self.original = make_avoidance_preflight(
            self.dll, "blocked")(p, self.arm, self.joints, self.client)
        self.obstacles = self.original["obstacle_ids"]
        self.final_goal = self.original["joint_waypoints_rad"][-1]

    def tearDown(self):
        self.p.disconnect(self.client)

    def test_displaced_six_axis_pose_requires_new_C_route_and_exact_goal_gate(self):
        p = self.p
        for joint, angle in zip(self.joints, self.final_goal):
            p.resetJointState(self.arm, joint, angle, physicsClientId=self.client)
        # Controlled simulated post-HOLD joint perturbation, not GUI picking.
        q = self.final_goal[0] + math.radians(2)
        p.resetJointState(self.arm, self.joints[0], q,
                          physicsClientId=self.client)
        start = [p.getJointState(
            self.arm, joint, physicsClientId=self.client)[0]
                 for joint in self.joints]
        route = plan_obstacle_hold_recovery(
            self.dll, p, self.arm, self.joints, self.obstacles, self.client,
            self.original["cartesian_goal_m"], self.final_goal, "blocked")
        self.assertTrue(route["passed"])
        self.assertGreaterEqual(route["C_apf_ik_joint_segments"], 1)
        self.assertEqual(tuple(route["start_joint_rad"]), tuple(start))
        self.assertEqual(tuple(route["joint_waypoints_rad"][-1]),
                         tuple(self.final_goal))
        self.assertGreaterEqual(
            route["minimum_measured_full_link_clearance_m"], 0.01)
        # Preflight MUST restore the actual (displaced) joint state.
        actual = [p.getJointState(
            self.arm, joint, physicsClientId=self.client)[0]
                  for joint in self.joints]
        self.assertEqual(actual, start)

    def test_each_joint_outbound_and_C_return_are_attempted_in_real_Bullet(self):
        """An axis can be refused by C IK; refusal is not fake acceptance."""
        from stage5_avoidance_uart_gate import load_scene
        from stage5_collision_validation import InvalidPath
        p = self.p
        scene, policy = load_scene("blocked")
        permitted = []
        rejected = []
        for axis in range(6):
            for delta_deg in (-0.5, +0.5):
                for j, q in zip(self.joints, self.final_goal):
                    p.resetJointState(self.arm, j, q,
                                      physicsClientId=self.client)
                target = list(self.final_goal)
                target[axis] += math.radians(delta_deg)
                try:
                    outgoing = validate_joint_path(
                        p, self.arm, self.obstacles, self.joints,
                        [self.final_goal, target],
                        maximum_joint_step_rad=math.radians(.25),
                        clearance_m=.012,
                        maximum_samples=policy["max_collision_samples"],
                        physics_client_id=self.client)
                    if not outgoing["passed"]:
                        rejected.append((axis, delta_deg, "outbound_clearance"))
                        continue
                    for j, q in zip(self.joints, target):
                        p.resetJointState(self.arm, j, q,
                                          physicsClientId=self.client)
                    try:
                        route = plan_obstacle_hold_recovery(
                            self.dll, p, self.arm, self.joints,
                            self.obstacles, self.client,
                            self.original["cartesian_goal_m"],
                            self.final_goal, "blocked",
                            minimum_extra_clearance_m=.002)
                    except (RejectedRoute, InvalidPath, ValueError, RuntimeError) as error:
                        rejected.append((axis, delta_deg, type(error).__name__))
                    else:
                        self.assertTrue(route["passed"])
                        self.assertGreaterEqual(
                            route["minimum_measured_full_link_clearance_m"], .012)
                        self.assertEqual(
                            tuple(route["joint_waypoints_rad"][-1]),
                            tuple(self.final_goal))
                        permitted.append((axis, delta_deg))
                finally:
                    # Entire test is authorization only, never actuation.
                    for j, q in zip(self.joints, self.final_goal):
                        p.resetJointState(self.arm, j, q,
                                          physicsClientId=self.client)
                actual = tuple(p.getJointState(
                    self.arm, j, physicsClientId=self.client)[0]
                    for j in self.joints)
                self.assertEqual(actual, tuple(self.final_goal))
        print("REAL C APF six-axis 0.5deg candidate feasibility:",
              "permitted", permitted, "rejected", rejected, flush=True)
        self.assertEqual(len(permitted) + len(rejected), 12)
        # J1 is known reachable; the other eleven are not assumed reachable.
        self.assertTrue(permitted, "No six-axis GUI jog candidate was feasible")

    def test_real_production_C_goal_near_J6_half_degree_is_not_misclassified(self):
        """Reproduce observed J6+0.5deg with REAL C APF and actual Bullet links."""
        p = self.p
        for joint, angle in zip(self.joints, self.final_goal):
            p.resetJointState(self.arm, joint, angle, physicsClientId=self.client)
        p.resetJointState(
            self.arm, self.joints[5],
            self.final_goal[5] + math.radians(.5),
            physicsClientId=self.client)
        route = plan_obstacle_hold_recovery(
            self.dll, p, self.arm, self.joints, self.obstacles,
            self.client, self.original["cartesian_goal_m"],
            self.final_goal, "blocked", minimum_extra_clearance_m=.002)
        self.assertTrue(route["passed"])
        self.assertGreaterEqual(
            route["minimum_measured_full_link_clearance_m"], .012)
        self.assertEqual(
            tuple(route["joint_waypoints_rad"][-1]),
            tuple(self.final_goal))
        self.assertIn(route["recovery_strategy"], (
            "bounded_full_link_validated_joint_only_C_APF_cartesian_goal_reached",
            "production_C_APF_C_IK_then_exact_joint_goal"))
        self.assertGreaterEqual(len(route["joint_waypoints_rad"]), 2)

    def test_zero_C_cartesian_waypoints_can_return_subdegree_J6_only_after_full_link_gate(self):
        """J6 may rotate without displacing EE beyond production C tolerance."""
        p = self.p
        for joint, angle in zip(self.joints, self.final_goal):
            p.resetJointState(self.arm, joint, angle, physicsClientId=self.client)
        displaced = self.final_goal[5] + math.radians(0.5)
        p.resetJointState(self.arm, self.joints[5], displaced,
                          physicsClientId=self.client)
        before = tuple(p.getJointState(
            self.arm, joint, physicsClientId=self.client)[0]
            for joint in self.joints)
        with patch("stage5_obstacle_hold_recovery.preflight_joint_avoidance",
                   side_effect=CartesianGoalAlreadyReached("C APF goal already reached")):
            route = plan_obstacle_hold_recovery(
                self.dll, p, self.arm, self.joints, self.obstacles,
                self.client, self.original["cartesian_goal_m"],
                self.final_goal, "blocked", minimum_extra_clearance_m=.002)
        self.assertTrue(route["passed"])
        self.assertEqual(route["C_apf_ik_joint_segments"], 0)
        self.assertTrue(route["exact_final_joint_goal_segment"])
        self.assertIn("joint_only", route["recovery_strategy"])
        self.assertEqual(len(route["joint_waypoints_rad"]), 2)
        self.assertGreaterEqual(route["sampled_full_arm_states"], 3)
        self.assertGreaterEqual(route["minimum_measured_full_link_clearance_m"], .012)
        self.assertEqual(tuple(route["joint_waypoints_rad"][-1]),
                         tuple(self.final_goal))
        self.assertEqual(before, tuple(p.getJointState(
            self.arm, joint, physicsClientId=self.client)[0]
            for joint in self.joints))

    def test_zero_C_cartesian_waypoints_cannot_excuse_large_joint_jump(self):
        p = self.p
        for joint, angle in zip(self.joints, self.final_goal):
            p.resetJointState(self.arm, joint, angle, physicsClientId=self.client)
        p.resetJointState(self.arm, self.joints[5],
                          self.final_goal[5] + math.radians(3),
                          physicsClientId=self.client)
        with patch("stage5_obstacle_hold_recovery.preflight_joint_avoidance",
                   side_effect=CartesianGoalAlreadyReached("C APF goal already reached")):
            with self.assertRaisesRegex(RejectedRoute, "too far in joint space"):
                plan_obstacle_hold_recovery(
                    self.dll, p, self.arm, self.joints, self.obstacles,
                    self.client, self.original["cartesian_goal_m"],
                    self.final_goal, "blocked", minimum_extra_clearance_m=.002)

    def test_other_C_APF_failure_never_uses_direct_joint_fallback(self):
        p = self.p
        for joint, angle in zip(self.joints, self.final_goal):
            p.resetJointState(self.arm, joint, angle, physicsClientId=self.client)
        p.resetJointState(self.arm, self.joints[5],
                          self.final_goal[5] + math.radians(.5),
                          physicsClientId=self.client)
        with patch("stage5_obstacle_hold_recovery.preflight_joint_avoidance",
                   side_effect=RejectedRoute("C APF blocked")):
            with self.assertRaisesRegex(RejectedRoute, "C APF blocked"):
                plan_obstacle_hold_recovery(
                    self.dll, p, self.arm, self.joints, self.obstacles,
                    self.client, self.original["cartesian_goal_m"],
                    self.final_goal, "blocked", minimum_extra_clearance_m=.002)

    def test_unsafe_current_pose_or_goal_cannot_be_authorized(self):
        p=self.p
        for joint, angle in zip(self.joints, self.final_goal):
            p.resetJointState(self.arm, joint, angle, physicsClientId=self.client)
        with self.assertRaisesRegex(RejectedRoute, "invalid dynamic recovery"):
            plan_obstacle_hold_recovery(
                self.dll, p, self.arm, self.joints, self.obstacles,
                self.client, self.original["cartesian_goal_m"],
                (float("nan"),)*6, "blocked")
        with self.assertRaisesRegex(RejectedRoute, "invalid dynamic recovery"):
            plan_obstacle_hold_recovery(
                self.dll, p, self.arm, self.joints, self.obstacles,
                self.client, self.original["cartesian_goal_m"],
                self.final_goal, "blocked",
                minimum_extra_clearance_m=-0.001)


if __name__ == "__main__":
    unittest.main()
