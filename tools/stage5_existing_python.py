"""Run Stage5 with an existing CPython interpreter and package directory.

The selected site-packages path is added to this process's import search path.
The user's installed environments remain unchanged.
"""
from __future__ import annotations

import argparse
import importlib
import sys
import traceback
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site-packages", required=True, type=Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--probe", action="store_true")
    mode.add_argument("--script", type=Path)
    args, program_args = parser.parse_known_args()

    packages = args.site_packages.resolve()
    if not packages.is_dir():
        parser.error(f"Existing site-packages directory does not exist: {packages}")

    sys.path.insert(0, str(packages))
    try:
        numpy = importlib.import_module("numpy")
        importlib.import_module("pybullet")
    except Exception:
        print(f"[Stage5] Existing packages cannot load with {sys.executable}",
              file=sys.stderr)
        traceback.print_exc()
        return 2
    print(f"[Stage5] Python: {sys.executable}; NumPy: {numpy.__version__}; "
          "PyBullet: imported", flush=True)
    if args.probe:
        return 0

    script = args.script.resolve()
    if not script.is_file():
        parser.error(f"Stage5 script missing: {script}")
    # Import under the real module name rather than __main__. The native GUI
    # worker uses multiprocessing.spawn and must unpickle capture_worker from
    # an importable stage5_qemu_uart_live_gui module on Windows.
    sys.path.insert(0, str(script.parent))
    sys.argv = [str(script), *program_args]
    module = importlib.import_module(script.stem)
    if not callable(getattr(module, "main", None)):
        parser.error(f"Stage5 script has no main() entry point: {script}")
    result = module.main()
    return result if isinstance(result, int) else 0


if __name__ == "__main__":
    raise SystemExit(main())
