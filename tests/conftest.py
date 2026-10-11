"""Shared set-up of the smoke tests: headless Qt, single-threaded numerics,
and the import paths of the calibration package and of the headless driver."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("OMP_NUM_THREADS", "1")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HEADLESS = os.path.join(ROOT, "experiments", "public", "experiment")
TOOL = os.environ.get("CALIB_TOOL_DIR", os.path.join(ROOT, "tool"))
RIG_A = os.environ.get("RIG_A_DIR", os.path.join(ROOT, "data", "rig_a"))

# rap_headless.py puts its PySide6 stub first on the path, so the algorithmic
# core is imported without the interface.
for p in (TOOL, HEADLESS):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402

cv2.setNumThreads(1)
