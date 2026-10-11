#!/usr/bin/env python
"""Screenshot of the calibration tool (not in the paper).

The tool is rendered offscreen on the Rig-A session and the window is grabbed
after detection and calibration have finished, so the figure shows the actual
software in its post-calibration state rather than a mock-up.

Requires the PySide6 environment of the tool (requirements.txt):
    QT_QPA_PLATFORM=offscreen python figures/make_fig8_gui.py
"""
import os, sys, time
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get('CALIB_TOOL_DIR', os.path.join(HERE, '..', 'tool')))

from pathlib import Path
from PySide6.QtWidgets import QApplication
from fast_calib_offline_gui.camera_intrinsics import IntrinsicsDialog

DATA = Path(os.environ.get('RIG_A_DIR', os.path.join(HERE, '..', 'data', 'rig_a'))).resolve()
OUT = os.path.join(HERE, 'out', 'gui_overview.png')
os.makedirs(os.path.dirname(OUT), exist_ok=True)

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

# --- detection -----------------------------------------------------------
dlg._detect_all()
pump(lambda: dlg.detect_btn.isEnabled(), label='detection')
print('detected:', sum(1 for e in dlg.entries if e.detected), flush=True)

# --- calibration ---------------------------------------------------------
dlg._calibrate()
pump(lambda: dlg.calib_btn.isEnabled() and dlg.result is not None,
     label='calibration')
r = dlg.result
print('result: rms=%.4f  used=%d  k3=%s'
      % (r['rms'], len(r.get('used_names') or []), r.get('k3_choice')), flush=True)

# --- show the point-distribution view and grab ---------------------------
for idx in range(dlg.tabs.count()):
    if dlg.tabs.tabText(idx).lower().startswith('cover'):
        dlg.tabs.setCurrentIndex(idx)
        break
app.processEvents()
dlg._refresh_views()
for _ in range(30):
    app.processEvents(); time.sleep(0.02)

pix = dlg.grab()
pix.save(OUT)
print('saved', OUT, pix.width(), 'x', pix.height(), flush=True)
