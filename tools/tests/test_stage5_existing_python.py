"""Regression for read-only CP311 package reuse and Windows spawn worker."""
from __future__ import annotations

import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / "tools" / "stage5_existing_python.py"


class ExistingPackageBridgeTests(unittest.TestCase):
    def test_probe_and_spawn_importable_worker_without_environment_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            packages = root / "preexisting_packages"
            packages.mkdir()
            (packages / "numpy.py").write_text("__version__ = 'fixture'\n",
                                                 encoding="utf-8")
            (packages / "pybullet.py").write_text("# existing fixture\n",
                                                    encoding="utf-8")
            program = root / "stage5_spawn_fixture.py"
            program.write_text(textwrap.dedent('''\
                import multiprocessing as mp
                import sys

                def worker(value):
                    import numpy
                    import pybullet
                    return value + (1 if numpy.__version__ == "fixture" else -1)

                def main():
                    with mp.get_context("spawn").Pool(1) as pool:
                        result = pool.map(worker, [41])
                    assert result == [42], result
                    assert "--fixture-arg" in sys.argv, sys.argv
                    print("SPAWN WORKER PASS")
                    return 0
            '''), encoding="utf-8")
            for extra, expected in (
                (["--probe"], "NumPy: fixture"),
                (["--script", str(program), "--fixture-arg"], "SPAWN WORKER PASS"),
            ):
                with self.subTest(extra=extra):
                    finished = subprocess.run(
                        [sys.executable, str(BRIDGE), "--site-packages",
                         str(packages), *extra],
                        cwd=root, text=True, capture_output=True,
                        timeout=50, check=False,
                    )
                    self.assertEqual(
                        finished.returncode, 0,
                        f"stdout: {finished.stdout}\nstderr: {finished.stderr}",
                    )
                    self.assertIn(expected, finished.stdout)

    def test_missing_site_packages_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [sys.executable, str(BRIDGE), "--site-packages",
                 str(Path(tmp) / "missing"), "--probe"],
                text=True, capture_output=True, timeout=15, check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("does not exist", result.stderr)


if __name__ == "__main__":
    unittest.main()
