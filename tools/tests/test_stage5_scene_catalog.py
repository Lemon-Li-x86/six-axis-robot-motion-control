"""Future Stage6 scene-loader compatibility, no PyBullet or QEMU needed."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SIM = Path(__file__).resolve().parents[2] / "simulation" / "pybullet_ur5"
sys.path.insert(0, str(SIM / "src"))
import stage5_avoidance_uart_gate as gate


class DeclarativeSceneCatalogTests(unittest.TestCase):
    def test_named_clear_and_blocked_scenes_keep_distinct_safety_policies(self):
        clear, safety = gate.load_scene("clear")
        blocked, same_policy = gate.load_scene("blocked")
        self.assertEqual(safety, same_policy)
        self.assertNotIn("max_fk_position_error_mm", safety)
        self.assertNotIn("max_fk_rotation_element_error", safety)
        self.assertFalse(clear["expect_direct_blocked"])
        self.assertTrue(blocked["expect_direct_blocked"])
        self.assertGreater(blocked["apf"]["clearance_mm"],
                           blocked["minimum_full_link_clearance_mm"])
        self.assertEqual(blocked["minimum_full_link_clearance_mm"], 10)
        self.assertEqual(len(blocked["goal_offset_mm"]), 3)
        self.assertEqual(len(blocked["obstacle"]["half_extents_m"]), 3)

    def test_uncompressed_test_override_is_explicit_and_type_safe(self):
        # The override is a harness setting; no scene JSON gets mutated.
        before=gate.SCENE_CATALOG.read_bytes()
        with self.assertRaisesRegex(ValueError, "shortcut_override"):
            gate.make_avoidance_preflight(Path("unused.so"), "blocked",
                                         shortcut_override="false")
        self.assertTrue(callable(gate.make_avoidance_preflight(
            Path("unused.so"), "blocked", shortcut_override=False)))
        self.assertEqual(gate.SCENE_CATALOG.read_bytes(), before)

    def test_unknown_fixture_is_not_silently_replaced(self):
        with self.assertRaisesRegex(ValueError, "unknown scene"):
            gate.load_scene("missing")

    def test_schema_and_invalid_geometry_fail_closed(self):
        catalog=json.loads(gate.SCENE_CATALOG.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as root:
            p=Path(root)/"scenes.json"
            def check_invalid(value):
                p.write_text(json.dumps(value),encoding="utf-8")
                with patch.object(gate,"SCENE_CATALOG",p):
                    with self.assertRaises(ValueError):
                        gate.load_scene("blocked")
            bad=json.loads(json.dumps(catalog))
            bad["schema_version"]=77
            check_invalid(bad)
            bad=json.loads(json.dumps(catalog))
            bad["scenes"]["blocked"]["obstacle"]["half_extents_m"][1]=0
            check_invalid(bad)
            bad=json.loads(json.dumps(catalog))
            bad["scenes"]["blocked"]["expect_direct_blocked"]="yes"
            check_invalid(bad)
            bad=json.loads(json.dumps(catalog))
            bad["scenes"]["blocked"]["minimum_full_link_clearance_mm"]=-1
            check_invalid(bad)
            bad=json.loads(json.dumps(catalog))
            bad["scenes"]["blocked"]["obstacle"]["world_center_m"]=[1,2,3]
            check_invalid(bad)


if __name__ == "__main__":
    unittest.main()
