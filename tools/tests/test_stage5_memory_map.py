"""Fixture-only tests: numeric expectations are NOT measurements of firmware."""
import importlib.util
import tempfile
import unittest
from pathlib import Path

MODULE = Path(__file__).resolve().parents[1] / "stage5_memory_map.py"
spec = importlib.util.spec_from_file_location("stage5_memory_map", MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

HEADER = """Memory Configuration

Name             Origin             Length             Attributes
FLASH            0x00000000         0x00001000         xr
RAM              0x20000000         0x00001000         xrw
*default*        0x00000000         0xffffffff

Linker script and memory map

.isr_vector      0x00000000         0x00000040
 .isr_vector     0x00000000         0x00000040 vector.o
.text            0x00000040         0x00000100
 .text           0x00000040         0x00000020 main.o
.data            0x20000000         0x00000020 load address 0x00000140
 .data           0x20000000         0x00000020 main.o
.bss             0x20000020         0x00000080
 .bss            0x20000020         0x00000080 main.o
"""


class LinkerMapTests(unittest.TestCase):
    def parse(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "test.map"
            p.write_text(source, encoding="utf-8")
            return module.parse_map(p)

    def test_output_sections_and_flash_initializers_count_once(self):
        result = self.parse(HEADER)
        self.assertEqual(result["flash_vma_bytes"], 0x140)
        self.assertEqual(result["ram_initial_values_in_flash_bytes"], 0x20)
        self.assertEqual(result["flash_allocated_bytes"], 0x160)
        self.assertEqual(result["ram_allocated_bytes"], 0xA0)
        self.assertEqual(len(result["sections"]), 4)

    def test_missing_memory_region_rejected(self):
        with self.assertRaisesRegex(ValueError, "FLASH and RAM"):
            self.parse(HEADER.replace("RAM              0x", "SRAM             0x"))

    def test_nonempty_section_outside_regions_rejected(self):
        with self.assertRaisesRegex(ValueError, "outside declared memory"):
            self.parse(HEADER + ".mystery         0x30000000         0x00000001\n")

    def test_lma_outside_regions_rejected(self):
        with self.assertRaisesRegex(ValueError, "Load images outside"):
            self.parse(HEADER.replace("load address 0x00000140", "load address 0x00100000"))

    def test_overlapping_ram_sections_rejected(self):
        with self.assertRaisesRegex(ValueError, "Overlapping RAM"):
            self.parse(HEADER + ".overlap         0x20000050         0x00000010\n")

    def test_unallocated_debug_does_not_count_as_flash(self):
        result = self.parse(
            HEADER + ".debug_info      0x00000000         0x00000400\n"
            + ".ARM.attributes  0x00000000         0x00000030\n"
        )
        self.assertEqual(result["flash_allocated_bytes"], 0x160)
        self.assertEqual(len(result["sections"]), 4)

    def test_zero_size_section_is_harmless(self):
        result = self.parse(HEADER + ".unused          0x00000160         0x00000000\n")
        self.assertEqual(result["flash_allocated_bytes"], 0x160)

    def test_missing_sections_rejected(self):
        with self.assertRaisesRegex(ValueError, "No GNU ld output sections"):
            self.parse(HEADER.split(".isr_vector", 1)[0])

    def test_delta_and_memory_layout_guard(self):
        before = self.parse(HEADER)
        after = self.parse(HEADER.replace("0x00000080", "0x000000A0"))
        self.assertEqual(module.compare_maps(after, before), {"flash": 0, "ram": 0x20})
        different = self.parse(HEADER.replace("0x00001000", "0x00002000"))
        with self.assertRaisesRegex(ValueError, "different region definitions"):
            module.compare_maps(after, different)


if __name__ == "__main__":
    unittest.main()
