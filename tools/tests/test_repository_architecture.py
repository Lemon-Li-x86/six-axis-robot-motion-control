"""Enforce key source boundaries without linking or running the firmware."""
from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware" / "freertos_demo"


class RepositoryArchitectureTests(unittest.TestCase):
    def test_algorithms_do_not_import_rtos_or_hardware(self):
        forbidden = {
            "FreeRTOS.h", "task.h", "queue.h", "semphr.h",
            "motor_driver.h", "uart_driver.h", "timer_driver.h",
            "gpio_driver.h", "protocol.h", "motor_state.h",
        }
        sources = sorted((FIRMWARE / "algorithms").rglob("*.c"))
        sources += sorted((FIRMWARE / "algorithms").rglob("*.h"))
        self.assertTrue(sources, "missing firmware algorithm sources")
        for source in sources:
            with self.subTest(source=source.relative_to(ROOT)):
                includes = set(re.findall(
                    r'^\s*#\s*include\s*[<"]([^">]+)[">]',
                    source.read_text(encoding="utf-8"), flags=re.MULTILINE,
                ))
                self.assertFalse(
                    includes & forbidden,
                    f"Algorithm {source.name} depends on RTOS/driver: "
                    f"{sorted(includes & forbidden)}",
                )

    def test_runtime_defaults_have_one_source(self):
        defaults = FIRMWARE / "config" / "robot_defaults.json"
        generator = ROOT / "tools" / "generate_firmware_defaults.py"
        task_config = (FIRMWARE / "application" / "task_config.h").read_text(
            encoding="utf-8"
        )
        stage4 = (FIRMWARE / "config" / "stage4_motion_config.c").read_text(
            encoding="utf-8"
        )
        self.assertTrue(defaults.is_file())
        self.assertTrue(generator.is_file())
        self.assertIn('#include "firmware_defaults_generated.h"', task_config)
        self.assertIn('#include "firmware_defaults_generated.h"', stage4)
        self.assertIn("FW_DEFAULT_STAGE4_PID", stage4)
        self.assertNotRegex(
            stage4,
            r'^\s*#\s*define\s+FW_DEFAULT_',
        )

    def test_uart_priority_is_shared_with_freertos(self):
        board = (FIRMWARE / "board.h").read_text(encoding="utf-8")
        rtos = (FIRMWARE / "FreeRTOSConfig.h").read_text(encoding="utf-8")
        uart = (FIRMWARE / "drivers" / "uart_driver.c").read_text(
            encoding="utf-8"
        )
        self.assertRegex(board, r'#define\s+BOARD_UART_IRQ_PRIORITY\s+0x80U')
        self.assertIn("BOARD_UART_IRQ_PRIORITY", rtos)
        self.assertIn("UART_IRQ_PRIORITY_VALUE BOARD_UART_IRQ_PRIORITY", uart)
        self.assertNotRegex(uart, r'#define\s+UART_IRQ_PRIORITY_VALUE\s+0x80U')


if __name__ == "__main__":
    unittest.main()
