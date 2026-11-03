# -*- coding: utf-8 -*-
"""Per-module test runner with timeouts, used during handover review.

Runs each app/tests/test_*.py separately so a single hang cannot block the rest.
"""
import subprocess
import sys
import time
from pathlib import Path

APP = Path(__file__).resolve().parents[1] / "app"
TESTS = sorted(APP.joinpath("tests").glob("test_*.py"))
rows = []
for path in TESTS:
    started = time.monotonic()
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", path.name],
            cwd=str(APP), capture_output=True, text=True, timeout=180,
        )
        elapsed = time.monotonic() - started
        tail = (proc.stderr or proc.stdout or "").strip().splitlines()
        summary = next((ln for ln in reversed(tail) if ln.startswith(("OK", "FAILED"))), "?")
        rows.append((path.name, proc.returncode, round(elapsed, 1), summary))
    except subprocess.TimeoutExpired:
        rows.append((path.name, "TIMEOUT", 180.0, ">180s, killed"))
    print(rows[-1], flush=True)

print("\n==== SUMMARY ====")
bad = [r for r in rows if r[1] != 0]
for r in bad:
    print("FAIL", r)
print("modules=%d ok=%d bad=%d" % (len(rows), len(rows) - len(bad), len(bad)))
