#!/usr/bin/env python
"""Offscreen driver of the interactive tool, used by test_gui.py.

Opens the Rig-A session in the calibration window, runs the Detect All and
Calibrate actions as a user would, and prints the number of detected images
and the selection result.  Requires PySide6 (requirements.txt).
"""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get('CALIB_TOOL_DIR', os.path.join(HERE, '..', 'tool')))

from PySide6.QtWidgets import QApplication  # noqa: E402
from intrinsic_calib.camera_intrinsics import IntrinsicsDialog  # noqa: E402

DATA = Path(os.environ.get('RIG_A_DIR', os.path.join(HERE, '..', 'data', 'rig_a'))).resolve()

app = QApplication(sys.argv)
dlg = IntrinsicsDialog(current_dataset_dir=str(DATA.parent))
dlg.resize(1600, 690)
dlg.show()
app.processEvents()
dlg.load_folder(DATA)
app.processEvents()
print('loaded %d entries' % len(dlg.entries), flush=True)


def pump(until, timeout=900, label=''):
    t0 = time.time()
    while not until():
        app.processEvents()
        time.sleep(0.02)
        if time.time() - t0 > timeout:
            raise RuntimeError('timeout waiting for ' + label)
    print('%s done in %.1fs' % (label, time.time() - t0), flush=True)


dlg._detect_all()
pump(lambda: dlg.detect_btn.isEnabled(), label='detection')
print('detected:', sum(1 for e in dlg.entries if e.detected), flush=True)

dlg._calibrate()
pump(lambda: dlg.calib_btn.isEnabled() and dlg.result is not None, label='calibration')
r = dlg.result
print('result: rms=%.4f  used=%d  k3=%s'
      % (r['rms'], len(r.get('used_names') or []), r.get('k3_choice')), flush=True)
