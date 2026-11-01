"""Prevent the project README from silently pointing at renamed/deleted docs."""
from __future__ import annotations

import re
import unittest
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]
README = ROOT / "README.md"
HANDOFF = ROOT / "HANDOFF_BOOK.md"
DOCUMENTS = (README, HANDOFF)


class ReadmeLocalLinkTests(unittest.TestCase):
    def test_relative_markdown_links_exist(self):
        for document in DOCUMENTS:
            with self.subTest(document=document.name):
                self.assertTrue(document.is_file(), f"Missing {document.name}")
                content = document.read_text(encoding="utf-8")
                links = re.findall(r"(?<!!)\[[^\]]+\]\(([^)]+)\)", content)
                self.assertTrue(links, f"{document.name} has no Markdown links")
                for link in links:
                    if link.startswith(("https://", "http://", "mailto:", "#")):
                        continue
                    relative = unquote(link.split("#", 1)[0])
                    if not relative:
                        continue
                    with self.subTest(document=document.name, link=link):
                        self.assertTrue(
                            (document.parent / relative).is_file(),
                            f"{document.name} points to a missing repository file: {link}",
                        )

    def test_readme_links_to_handoff_book(self):
        content = README.read_text(encoding="utf-8")
        self.assertIn("(HANDOFF_BOOK.md)", content)


if __name__ == "__main__":
    unittest.main()
