"""Regression guard: the recovered native GUI must accept optional C APF inputs.

These tests check the old GUI CLI handoff without launching QEMU or PyBullet.
The full C APF -> FreeRTOS UART -> real Bullet validation stays in CI.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "simulation" / "pybullet_ur5" / "src"
sys.path.insert(0, str(SIM))

import stage5_qemu_uart_visual_demo as gui  # noqa: E402


class Stage5GuiApfEntrypointTests(unittest.TestCase):
    def test_ordinary_gui_cli_does_not_enable_avoidance(self):
        with mock.patch.object(sys, "argv", [
            "stage5_qemu_uart_visual_demo.py", "--elf", "firmware.elf",
            "--headless", "--no-hold",
        ]), mock.patch.object(gui, "run", return_value={"passed": True}) as runner:
            self.assertEqual(gui.main(), 0)
        self.assertIsNone(runner.call_args.kwargs["prepare_motion"])

    def test_opt_in_production_c_apf_is_passed_to_existing_gui_worker(self):
        marker = object()
        with mock.patch.object(sys, "argv", [
            "stage5_qemu_uart_visual_demo.py", "--elf", "firmware.elf",
            "--headless", "--no-hold",
            "--stage5-c-apf-library", "real-host-C-bridge.so",
            "--stage5-route-case", "blocked", "--stage5-raw-route",
        ]), mock.patch(
            "stage5_avoidance_uart_gate.make_avoidance_preflight",
            return_value=marker,
        ) as make, mock.patch.object(
            gui, "run", return_value={"passed": True}
        ) as runner:
            self.assertEqual(gui.main(), 0)
        make.assert_called_once_with(
            Path("real-host-C-bridge.so"), "blocked", shortcut_override=False
        )
        self.assertIs(runner.call_args.kwargs["prepare_motion"], marker)

    def test_raw_route_without_C_bridge_is_refused(self):
        with mock.patch.object(sys, "argv", [
            "stage5_qemu_uart_visual_demo.py", "--elf", "firmware.elf",
            "--stage5-raw-route",
        ]), mock.patch.object(gui, "run") as runner:
            with self.assertRaises(SystemExit) as exited:
                gui.main()
        self.assertEqual(exited.exception.code, 2)
        runner.assert_not_called()


if __name__ == "__main__":
    unittest.main()
