"""Regression for the *runtime* Stage5 embedded-APF UR5 full-link gate.

The planning preflight already sweeps links, but physical execution must
separately inspect the current measured posture. No real PyBullet required:
a fake physics API makes the default-base-only regression deterministic.
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_stage5_embedded_apf_uart import require_runtime_full_link_clearance


class FakeBullet:
    def __init__(self, near_link=None, distance=0.009):
        self.near_link = near_link
        self.distance = distance
        self.checked = []

    def getNumJoints(self, arm, *, physicsClientId):
        assert arm == 7 and physicsClientId == 3
        return 6

    def getClosestPoints(self, arm, obstacle, limit, *, linkIndexA, linkIndexB,
                         physicsClientId):
        assert (arm, obstacle, linkIndexB, physicsClientId) == (7, 8, -1, 3)
        assert limit == 0.01
        self.checked.append(linkIndexA)
        if linkIndexA == self.near_link:
            # PyBullet contact tuple's signed contact distance is index 8.
            return [tuple([None] * 8 + [self.distance])]
        return []


class RuntimeAllLinkTests(unittest.TestCase):
    def test_clear_arm_probes_base_and_all_six_links(self):
        bullet = FakeBullet()
        checks = require_runtime_full_link_clearance(bullet, 7, 8, 0.01, 3)
        self.assertEqual(checks, 7)
        self.assertEqual(bullet.checked, [-1, 0, 1, 2, 3, 4, 5])

    def test_wrist_link_near_box_fails_even_when_base_is_clear(self):
        bullet = FakeBullet(near_link=5, distance=0.009)
        with self.assertRaisesRegex(RuntimeError, r"link=5"):
            require_runtime_full_link_clearance(bullet, 7, 8, 0.01, 3)
        self.assertEqual(bullet.checked, [-1, 0, 1, 2, 3, 4, 5])

    def test_base_near_box_also_fails(self):
        bullet = FakeBullet(near_link=-1, distance=-0.001)
        with self.assertRaisesRegex(RuntimeError, r"link=-1"):
            require_runtime_full_link_clearance(bullet, 7, 8, 0.01, 3)

    def test_clearance_boundary_is_not_silently_allowed(self):
        bullet = FakeBullet(near_link=2, distance=0.01)
        with self.assertRaisesRegex(RuntimeError, r"link=2"):
            require_runtime_full_link_clearance(bullet, 7, 8, 0.01, 3)

    def test_nonfinite_contact_fails_closed(self):
        bullet = FakeBullet(near_link=4, distance=math.nan)
        with self.assertRaisesRegex(RuntimeError, r"link=4"):
            require_runtime_full_link_clearance(bullet, 7, 8, 0.01, 3)

    def test_invalid_clearance_fails_before_physics_query(self):
        for limit in (0, -0.001, math.nan, math.inf):
            with self.subTest(limit=limit):
                bullet = FakeBullet()
                with self.assertRaises(ValueError):
                    require_runtime_full_link_clearance(bullet, 7, 8, limit, 3)
                self.assertEqual(bullet.checked, [])


if __name__ == "__main__":
    unittest.main()
