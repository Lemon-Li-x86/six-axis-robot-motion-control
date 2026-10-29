"""Real PyBullet + production C APF/IK full-arm safety preflight."""
import math
import os
import sys
import unittest
from pathlib import Path
import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from stage5_static_obstacle_probe import Config
from stage5_avoidance_preflight import (
    preflight_joint_avoidance, RejectedRoute,
)

LIBRARY = Path(os.environ["STAGE5_APF_BRIDGE"])
URDF = ROOT / "models/ur5/urdf/ur5.urdf"


class RealCJointRoutePreflightTests(unittest.TestCase):
    def setUp(self):
        self.client = p.connect(p.DIRECT)
        self.robot = p.loadURDF(str(URDF), useFixedBase=True,
                                physicsClientId=self.client)
        self.joints = [
            j for j in range(p.getNumJoints(self.robot, physicsClientId=self.client))
            if p.getJointInfo(self.robot, j, physicsClientId=self.client)[2]
            == p.JOINT_REVOLUTE
        ]
        self.assertEqual(len(self.joints), 6)
        self.ee = next(
            j for j in range(p.getNumJoints(self.robot, physicsClientId=self.client))
            if p.getJointInfo(self.robot, j, physicsClientId=self.client)[12]
            == b"ee_link")
        for joint, angle in zip(self.joints, [90, -60, 80, 35, 55, -120]):
            p.resetJointState(self.robot, joint, math.radians(angle),
                              physicsClientId=self.client)
        self.initial = tuple(p.getJointState(
            self.robot, j, physicsClientId=self.client)[0]
            for j in self.joints)
        start_m = p.getLinkState(
            self.robot, self.ee, computeForwardKinematics=True,
            physicsClientId=self.client)[4]
        self.goal = [start_m[0]*1000+3, start_m[1]*1000+1.5,
                     start_m[2]*1000]

    def tearDown(self):
        p.disconnect(self.client)

    def box(self, centre, half):
        collision = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
        return p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=collision,
            basePosition=centre, physicsClientId=self.client)

    def preflight(self, obstacle, *, minimum_full_link_clearance_m=None,
                  via_points_mm=(), position_gate_mm=0.5,
                  rotation_gate=0.003):
        return preflight_joint_avoidance(
            LIBRARY, p, self.robot, [obstacle], self.joints, self.ee,
            self.goal,
            apf_config=Config(1.0, 0.0, 150.0, 5.0, 1.5, 0.1),
            max_velocity_deg_s=[15.0]*6,
            max_acceleration_deg_s2=[30.0]*6,
            max_fk_position_error_mm=position_gate_mm,
            max_fk_rotation_element_error=rotation_gate,
            waypoint_capacity=16,
            max_joint_step_rad=0.02,
            max_collision_samples=1000,
            physics_client_id=self.client,
            minimum_full_link_clearance_m=minimum_full_link_clearance_m,
            via_points_mm=via_points_mm)

    def assert_unmodified(self):
        observed = tuple(p.getJointState(
            self.robot, j, physicsClientId=self.client)[0]
            for j in self.joints)
        self.assertEqual(observed, self.initial)

    def test_real_c_apf_ik_and_all_link_gate_pass_for_clear_path(self):
        distant = self.box([5, 5, 5], [0.05]*3)
        result = self.preflight(distant)
        self.assertTrue(result["passed"])
        self.assertGreaterEqual(result["cartesian_waypoints"], 2)
        self.assertEqual(len(result["joint_waypoints_rad"]),
                         result["cartesian_waypoints"]+1)
        self.assertGreater(result["sampled_full_arm_states"], 1)
        self.assertTrue(all(math.isfinite(x) and x>=0
                            for x in result["segment_duration_s"]))
        self.assert_unmodified()

    def test_point_clearance_is_not_full_link_clearance(self):
        # 5mm Cartesian APF point clearance is separate from a 20mm
        # required robot-body clearance, the original source of false safety.
        distant = self.box([5, 5, 5], [0.05]*3)
        result = self.preflight(
            distant, minimum_full_link_clearance_m=0.020)
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["cartesian_point_clearance_m"], 0.005)
        self.assertAlmostEqual(result["static_collision_clearance_m"], 0.020)
        self.assert_unmodified()

    def test_python_ik_limits_default_to_authoritative_c_not_json(self):
        distant=self.box([5,5,5],[0.05]*3)
        route=self.preflight(distant,position_gate_mm=None,
                             rotation_gate=None)
        self.assertTrue(route["passed"])
        with self.assertRaisesRegex(RejectedRoute, "production C limits"):
            self.preflight(distant,position_gate_mm=1000.0,
                           rotation_gate=None)
        self.assert_unmodified()

    def test_invalid_full_link_clearance_fails_closed(self):
        distant = self.box([5, 5, 5], [0.05]*3)
        for invalid in (-0.001, float("nan"), float("inf")):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(RejectedRoute, "clearance"):
                    self.preflight(
                        distant, minimum_full_link_clearance_m=invalid)
                self.assert_unmodified()

    def test_multi_leg_mission_waypoint_uses_same_c_apf_and_ik(self):
        distant = self.box([5,5,5], [0.05]*3)
        via=(self.goal[0]-3.0, self.goal[1]+5.0,
             self.goal[2])
        route=self.preflight(distant, via_points_mm=[via])
        self.assertTrue(route["passed"])
        self.assertEqual(route["via_points_mm"], [via])
        self.assertGreater(route["cartesian_waypoints"], 4)
        self.assertAlmostEqual(route["cartesian_route_mm"][-1][0],
                               self.goal[0], delta=0.1)
        self.assert_unmodified()

    def test_unreachable_mission_via_rejects_whole_route(self):
        distant = self.box([5,5,5], [0.05]*3)
        with self.assertRaisesRegex(RejectedRoute, "C APF route leg"):
            self.preflight(distant,
                           via_points_mm=[(5000.0,0.0,0.0)])
        self.assert_unmodified()

    def test_blocked_middle_link_rejected_without_motion(self):
        low,high = p.getAABB(
            self.robot,self.joints[2],physicsClientId=self.client)
        centre = [(a+b)/2 for a,b in zip(low,high)]
        half = [max(0.05,(b-a)/2+0.02) for a,b in zip(low,high)]
        overlapping = self.box(centre,half)
        with self.assertRaisesRegex(RejectedRoute,"full-arm collision"):
            self.preflight(overlapping)
        self.assert_unmodified()

    def test_unreachable_goal_fails_closed_before_joint_motion(self):
        distant = self.box([5,5,5], [0.05]*3)
        self.goal=[5000.0,0.0,0.0]
        with self.assertRaisesRegex(RejectedRoute,"production C APF"):
            self.preflight(distant)
        self.assert_unmodified()


if __name__ == "__main__":
    unittest.main()
