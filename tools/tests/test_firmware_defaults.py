"""Configuration rejection tests: prevent invalid policies entering ARM builds."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "firmware_defaults", ROOT / "tools/generate_firmware_defaults.py")
defaults = importlib.util.module_from_spec(spec)
spec.loader.exec_module(defaults)


class FirmwareDefaultsTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / "firmware/freertos_demo/config/robot_defaults.json")
                                 .read_text(encoding="utf-8"))

    def test_current_config_generates_all_six_independent_pid_entries(self):
        motion = self.config["stage4_motion"]
        motion["pid"][5]["kp"] = 0.125
        header = defaults.generate_header(self.config)
        self.assertIn("FW_DEFAULT_STAGE4_PID", header)
        self.assertIn("0.125F", header)
        self.assertIn("FW_DEFAULT_STAGE4_MAX_VELOCITY_DEG_S", header)
        self.assertIn("FW_DEFAULT_STAGE4_REFERENCE_TIMEOUT_TRAJECTORY_PERIODS", header)

    def test_reject_non_numeric_and_non_finite_pid(self):
        for value in (True, "0.2", None, float("nan"), float("inf"), -0.1, 10**400):
            with self.subTest(value=value):
                cfg = copy.deepcopy(self.config)
                cfg["stage4_motion"]["pid"][2]["kp"] = value
                with self.assertRaises(ValueError):
                    defaults.generate_header(cfg)

    def test_zero_output_limit_is_rejected(self):
        self.config["stage4_motion"]["pid"][0]["output_limit_deg"] = 0
        with self.assertRaises(ValueError):
            defaults.generate_header(self.config)

    def test_zero_pid_gains_remain_valid(self):
        for field in defaults.PID_FIELDS:
            if field != "output_limit_deg":
                self.config["stage4_motion"]["pid"][0][field] = 0
        self.assertIn("0.0F", defaults.generate_header(self.config))

    def test_each_axis_requires_positive_finite_rates(self):
        for field in ("max_velocity_deg_s", "max_acceleration_deg_s2"):
            for value in (0, -1, True, float("inf"), float("nan")):
                with self.subTest(field=field, value=value):
                    cfg = copy.deepcopy(self.config)
                    cfg["stage4_motion"][field][4] = value
                    with self.assertRaises(ValueError):
                        defaults.generate_header(cfg)

    def test_all_axis_lists_require_exactly_six_values(self):
        for field in ("pid", "max_velocity_deg_s", "max_acceleration_deg_s2"):
            for count in (0, 5, 7):
                cfg = copy.deepcopy(self.config)
                values = cfg["stage4_motion"][field]
                cfg["stage4_motion"][field] = [values[0]] * count
                with self.subTest(field=field, count=count), self.assertRaises(ValueError):
                    defaults.generate_header(cfg)

    def test_reject_unknown_or_missing_motion_and_pid_fields(self):
        for field in ("stage4_motion", "pid"):
            cfg = copy.deepcopy(self.config)
            section = cfg["stage4_motion"] if field == "stage4_motion" else cfg["stage4_motion"]["pid"][0]
            section["misspelled_policy"] = 1
            with self.subTest(field=field), self.assertRaises(ValueError):
                defaults.generate_header(cfg)
        del self.config["stage4_motion"]["feedback_timeout_ms"]
        with self.assertRaises(ValueError):
            defaults.generate_header(self.config)

    def test_timeouts_require_positive_integers(self):
        for field in ("feedback_timeout_ms", "reference_timeout_trajectory_periods"):
            for value in (0, True, 1.5, -1, 100000):
                cfg = copy.deepcopy(self.config)
                cfg["stage4_motion"][field] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    defaults.generate_header(cfg)

    def test_partial_frame_timeout_rejects_invalid_values(self):
        for value in (0, True, 1.5, -1, 1001):
            cfg = copy.deepcopy(self.config)
            cfg["protocol_frame_timeout_ms"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                defaults.generate_header(cfg)

    def test_c_scalar_literal_handles_integer_and_exponent_forms(self):
        self.assertEqual(defaults.real_literal(15, "test"), "15.0F")
        self.assertEqual(defaults.real_literal(1e-9, "test"), "1e-09F")


if __name__ == "__main__":
    unittest.main()
