"""Puts the project root and sim/ on sys.path so the tests can import your modules."""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(_ROOT, "sim"), _ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)
