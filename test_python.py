# Thin runner to execute tests/test_python.py from project root
import os
import runpy
import sys

ROOT = os.path.abspath(os.path.dirname(__file__))
TESTS = os.path.join(ROOT, 'tests')

if TESTS not in sys.path:
    sys.path.insert(0, TESTS)

ENTRY = os.path.join(TESTS, 'test_python.py')
if not os.path.isfile(ENTRY):
    raise FileNotFoundError(f"Test script not found: {ENTRY}")

runpy.run_path(ENTRY, run_name='__main__')
