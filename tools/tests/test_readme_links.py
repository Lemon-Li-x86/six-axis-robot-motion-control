"""Prevent the project README from silently pointing at renamed/deleted docs."""
from __future__ import annotations

import re
import unittest
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]
README = ROOT / "README.md"


class ReadmeLocalLinkTests(unittest.TestCase):
    def test_relative_markdown_links_exist(self):
        content = README.read_text(encoding="utf-8")
        links = re.findall(r"(?<!!)\[[^\]]+\]\(([^)]+)\)", content)
        self.assertTrue(links, "README link check did not find any Markdown links")
        for link in links:
            if link.startswith(("https://", "http://", "mailto:", "#")):
                continue
            relative = unquote(link.split("#", 1)[0])
            if not relative:
                continue
            with self.subTest(link=link):
                self.assertTrue(
                    (ROOT / relative).is_file(),
                    f"README points to a missing repository file: {link}",
                )


if __name__ == "__main__":
    unittest.main()
