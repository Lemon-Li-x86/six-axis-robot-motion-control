"""Real PyBullet DIRECT tests for the Stage5 discrete full-link obstacle gate."""
import math
import sys
import unittest
from pathlib import Path

import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from stage5_collision_validation import InvalidPath, validate_joint_path

URDF = ROOT / "models" / "ur5" / "urdf" / "ur5.urdf"


class WholeArmObstacleTests(unittest.TestCase):
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
        self.start = tuple(
            p.getJointState(self.robot, j, physicsClientId=self.client)[0]
            for j in self.joints)

    def tearDown(self):
        p.disconnect(self.client)

    def box(self, centre, half):
        collision = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=half, physicsClientId=self.client)
        return p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=collision,
            basePosition=centre, physicsClientId=self.client)

    def check(self, box_id, route, maximum_samples=128):
        return validate_joint_path(
            p, self.robot, [box_id], self.joints, route,
            maximum_joint_step_rad=0.02, clearance_m=0.005,
            maximum_samples=maximum_samples, physics_client_id=self.client)

    def assert_restored(self):
        now = tuple(p.getJointState(
            self.robot, j, physicsClientId=self.client)[0]
            for j in self.joints)
        self.assertEqual(now, self.start)

    def test_far_box_passes_full_link_sample_and_restores(self):
        obstacle = self.box([5.0, 5.0, 5.0], [0.05, 0.05, 0.05])
        finish = tuple(q + (0.04 if i == 0 else 0.0)
                       for i,q in enumerate(self.start))
        report = self.check(obstacle, [self.start, finish])
        self.assertTrue(report["passed"])
        self.assertGreaterEqual(report["sampled_states"], 3)
        self.assert_restored()

    def test_box_overlapping_middle_link_fails_even_if_ee_elsewhere(self):
        link = self.joints[2]
        lo,hi = p.getAABB(self.robot, link, physicsClientId=self.client)
        centre = [(a+b)*0.5 for a,b in zip(lo,hi)]
        half = [max(0.05,(b-a)*0.5+0.02) for a,b in zip(lo,hi)]
        obstacle = self.box(centre, half)
        report = self.check(obstacle, [self.start])
        self.assertFalse(report["passed"])
        self.assertIsNotNone(report["collision"])
        self.assertLessEqual(report["collision"]["signed_distance_m"], 0.005)
        self.assert_restored()

    def test_joint_limit_and_budget_reject_before_mutation(self):
        obstacle = self.box([5.0,5.0,5.0],[0.05]*3)
        invalid = list(self.start)
        invalid[0] = 50.0
        with self.assertRaisesRegex(InvalidPath, "joint limits"):
            self.check(obstacle, [self.start, invalid])
        self.assert_restored()
        finish = tuple(q + (0.20 if i == 0 else 0.0)
                       for i,q in enumerate(self.start))
        with self.assertRaisesRegex(InvalidPath, "budget"):
            self.check(obstacle, [self.start,finish], maximum_samples=2)
        self.assert_restored()


if __name__ == "__main__":
    unittest.main()
