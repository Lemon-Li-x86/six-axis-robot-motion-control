"""Stage5 acceptance must reject mock-only, cancelled and unsafe GUI traces."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.check_stage5_safe_gui_acceptance import evaluate  # noqa: E402


def good() -> dict:
    return dict(
        passed=True, headless=False, gui_safe_jog=True, cancelled=False,
        motor_commands="virtual_serial_only",
        segments_requested=4, segments_settled=4,
        avoidance_preflight={"passed": True},
        obstacle_recovery_preflight={"passed": True},
        runtime_full_link_collision_free=True,
        safe_gui_jog_applied=1, safe_gui_jog_completed=1, hold_recoveries=1,
        obstacle_recovery_routes=[{"completed": True, "planned_segments": 1}],
        safe_gui_jog_events=[{
            "event": "safe_gui_jog_applied", "joint": 6,
            "displacement_deg": 0.5,
        }],
        bad_checksum_frames=0, bad_ack_frames=0,
        clock_alignment_passed=True,
        fault_counters=dict(controller_faults=0, planner_faults=0,
                            feedback_stale_count=0),
        hold_final_error_deg=0.01,
    )


class RealGuiAcceptanceTests(unittest.TestCase):
    def test_native_c_uart_j6_full_link_evidence_accepts(self):
        self.assertEqual(evaluate(good()), [])

    def test_just_original_route_without_a_jog_is_not_pass(self):
        x = good()
        x.update(safe_gui_jog_applied=0, safe_gui_jog_completed=0,
                 hold_recoveries=0, obstacle_recovery_routes=[],
                 safe_gui_jog_events=[], obstacle_recovery_preflight=None)
        self.assertNotEqual(evaluate(x), [])

    def test_cancel_and_gui_disconnect_cannot_be_promoted_to_pass(self):
        x = good()
        x.update(passed=False, cancelled=True,
                 fault_type="stage5_gui_disconnected")
        self.assertNotEqual(evaluate(x), [])

    def test_incomplete_return_and_uart_fault_are_rejected(self):
        for name, value in (
            ("safe_gui_jog_completed", 0),
            ("hold_recoveries", 0),
            ("obstacle_recovery_routes", [{"completed": False}]),
            ("bad_checksum_frames", 2),
            ("bad_ack_frames", 1),
            ("clock_alignment_passed", False),
            ("runtime_full_link_collision_free", False),
        ):
            with self.subTest(name=name):
                x = good()
                x[name] = value
                self.assertNotEqual(evaluate(x), [])

    def test_wrong_axis_or_displacement_and_fault_counter_are_rejected(self):
        for axis, amount in ((1, 0.5), (6, 2), (6, -0.5)):
            with self.subTest(axis=axis, amount=amount):
                x = good()
                x["safe_gui_jog_events"][0]["joint"] = axis
                x["safe_gui_jog_events"][0]["displacement_deg"] = amount
                self.assertNotEqual(evaluate(x), [])
        x = good()
        x["fault_counters"]["controller_faults"] = 1
        self.assertNotEqual(evaluate(x), [])


if __name__ == "__main__":
    unittest.main()
