"""Run the full offline suite in one process and persist the summary.

Some Tk tests leave interpreter-shutdown noise on this machine, so the summary
is written to a file from inside the process instead of relying on stdout.
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(os.path.dirname(HERE), "app")
os.chdir(APP)
sys.path.insert(0, APP)

suite = unittest.defaultTestLoader.discover("tests")
log = open(os.path.join(HERE, "test_full_log.txt"), "w", encoding="utf-8")
runner = unittest.TextTestRunner(stream=log, verbosity=1)
result = runner.run(suite)
with open(os.path.join(HERE, "test_full_summary.txt"), "w", encoding="utf-8") as f:
    f.write(
        "ran=%d failures=%d errors=%d skipped=%d expectedFailures=%d\n"
        % (
            result.testsRun,
            len(result.failures),
            len(result.errors),
            len(result.skipped),
            len(result.expectedFailures),
        )
    )
    for kind, entries in (("FAIL", result.failures), ("ERROR", result.errors)):
        for case, tb in entries:
            f.write("%s %s\n%s\n" % (kind, case, tb.splitlines()[-1] if tb else ""))
log.close()
