"""Pure application retry policy, with production C geometry covered by host CTests.

These tests require no PyBullet or physical side effects. They prove that
a rejected IK/collision candidate cannot become an accepted motion path.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                        "simulation" / "pybullet_ur5" / "src"))
from stage5_avoidance_preflight import RejectedRoute, select_bypass_candidate


class AutoBypassSelectionTests(unittest.TestCase):
    def test_rejected_first_candidate_tries_second_without_overriding_gate(self):
        calls = []
        def full_gate(via):
            calls.append(via)
            if via == (0.0, 0.0, 10.0):
                raise RejectedRoute("C IK or full-arm clearance refused")
            return {"passed": True, "joint_waypoints_rad": [(0,) * 6, (1,) * 6]}
        result = select_bypass_candidate(
            [(0.0, 0.0, 10.0), (0.0, 15.0, 0.0)], full_gate)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result["bypass_candidates_attempted"], 2)
        self.assertEqual(len(result["bypass_candidate_rejections"]), 1)
        self.assertIn("clearance", result["bypass_candidate_rejections"][0]["reason"])

    def test_every_rejection_fails_closed(self):
        def reject(_):
            raise RejectedRoute("full-link collision")
        with self.assertRaisesRegex(RejectedRoute, "all 2"):
            select_bypass_candidate([(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)], reject)

    def test_no_obstacle_calls_unchanged_direct_gate(self):
        calls = []
        def direct(via):
            calls.append(via)
            return {"passed": True}
        result = select_bypass_candidate([], direct)
        self.assertEqual(calls, [()])
        self.assertEqual(result["bypass_candidates_attempted"], 1)

    def test_unapproved_route_never_gets_promoted(self):
        with self.assertRaisesRegex(RejectedRoute, "all 1"):
            select_bypass_candidate([(1.0, 0.0, 0.0)],
                                   lambda _: {"passed": False})


if __name__ == "__main__":
    unittest.main()
