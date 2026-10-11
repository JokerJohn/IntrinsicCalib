"""The interactive tool, rendered offscreen on Rig-A: the window detects the
39 images and its Calibrate action returns the same selection as the headless
driver (20 retained images, k3 estimated).  Runs tests/gui_offscreen.py in a
separate process, so that the real PySide6 is used."""
import os
import subprocess
import sys

import pytest

from conftest import ROOT

pytest.importorskip("PySide6")


def test_gui_offscreen_on_rig_a():
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", OMP_NUM_THREADS="1")
    p = subprocess.run([sys.executable, os.path.join(ROOT, "tests", "gui_offscreen.py")],
                       capture_output=True, text=True, env=env, timeout=300)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "detected: 39" in p.stdout
    assert "used=20" in p.stdout and "k3=free" in p.stdout
