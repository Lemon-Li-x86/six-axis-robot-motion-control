"""Check consistent module documentation across firmware and Python sources."""
from __future__ import annotations

import ast
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware" / "freertos_demo"


class CommentStyleTests(unittest.TestCase):
    def test_c_files_have_file_header_and_brief(self):
        sources = sorted(FIRMWARE.rglob("*.c")) + sorted(FIRMWARE.rglob("*.h"))
        self.assertTrue(sources, "firmware sources not found")
        for source in sources:
            text = source.read_text(encoding="utf-8")
            with self.subTest(source=source.relative_to(ROOT)):
                self.assertRegex(
                    text, r"\A/\*\*\s*\n\s*\* @file [^\n]+\n\s*\* @brief [^\n]+"
                )

    def test_c_comment_spacing(self):
        sources = sorted(FIRMWARE.rglob("*.c")) + sorted(FIRMWARE.rglob("*.h"))
        for source in sources:
            text = source.read_text(encoding="utf-8")
            with self.subTest(source=source.relative_to(ROOT)):
                self.assertNotRegex(text, r"\n[ \t]*\n[ \t]*\n")
                self.assertNotRegex(text, r"(?m)^[ \t]+$")
                self.assertNotRegex(text, r"(?m)^/\*\s*={10,}")

    def test_python_modules_have_docstrings_and_parse(self):
        sources = sorted((ROOT / "tools").rglob("*.py"))
        sources += sorted((ROOT / "simulation").rglob("*.py"))
        self.assertTrue(sources, "Python sources not found")
        for source in sources:
            with self.subTest(source=source.relative_to(ROOT)):
                syntax = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
                summary = ast.get_docstring(syntax)
                self.assertTrue(summary and summary.strip())


if __name__ == "__main__":
    unittest.main()
