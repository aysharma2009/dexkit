"""Cross-platform CLI integration check; uses temporary config and state, never real hardware.

Run with the interpreter where DexKit is installed: python scripts/check_mock.py
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="dexkit-cli-check-") as folder:
        work = Path(folder)
        shutil.copytree(root / "config", work / "config")
        env = dict(os.environ, DEXKIT_CONFIG=str(work / "config"), DEXKIT_DATA=str(work / "data"),
                   PYTHONIOENCODING="utf-8")
        rec = work / "sample.npz"
        action = np.zeros((3, 16))
        action[:, 0] = [0, .1, 0]
        np.savez(rec, t=np.array([0., .1, .2]), action=action, state=action)
        commands = [
            ("dexkit.tools.scan_bus", ["--mock", "--max-id", "12"]),
            ("dexkit.tools.calibrate_hand", ["--mock", "--scripted", "--force"]),
            ("dexkit.tools.calibrate_hand", ["--mock", "--wiring"]),
            ("dexkit.tools.calibrate_hand", ["--mock", "--scripted", "--tight-only", "--trust-current", "--servo", "0"]),
            ("dexkit.tools.calibrate_gantry", ["--mock", "--scripted"]),
            ("dexkit.control.poses", ["--mock", "--yes", "fist", "--duration", ".1", "--hold", ".1"]),
            ("dexkit.control.teleop", ["--mock", "--yes", "--duration", ".6", "--script", "0.1:r,0.2:f,0.4:r,0.5:esc"]),
            ("dexkit.control.recorder", ["--mock", "--yes", "--hand-only", str(rec)]),
            ("dexkit.control.sequence", ["--mock", "--yes", "--mock-speed", "20", "examples/pick_and_show.yaml"]),
            ("dexkit.policy.collect", ["--mock", "--yes", "--task", "check", "--duration", ".6", "--script", "0.1:r,0.5:r"]),
            ("dexkit.tools.relax", ["--mock", "--now"]),
            ("dexkit.tools.estop", ["--mock"]),
            ("dexkit.tools.estop", ["--mock", "--clear"]),
        ]
        for module, args in commands:
            result = subprocess.run([sys.executable, "-m", module, *args], cwd=root, env=env,
                                    capture_output=True, text=True, encoding="utf-8", timeout=120)
            if result.returncode:
                print(result.stdout)
                print(result.stderr, file=sys.stderr)
                raise SystemExit(f"FAILED: {module} {' '.join(args)}")
            print(f"PASS: {module} {' '.join(args)}", flush=True)
        print(f"All {len(commands)} simulated CLI workflows passed.")


if __name__ == "__main__":
    main()
