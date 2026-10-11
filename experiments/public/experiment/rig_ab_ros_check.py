#!/usr/bin/env python
"""Reproduce the Rig-A / Rig-B consistency numbers quoted in the paper.

Runs the project's own detector and calibrator with both methods disabled
(all detected frames, M2) on the Rig-A and Rig-B sessions and compares
the result with the ost.yaml the ROS calibrator wrote for the same data
(Supplement, consistency checks).
"""
import sys, glob, os, yaml
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get('CALIB_TOOL_DIR', os.path.join(HERE, '..', '..', '..', 'tool')))
from fast_calib_offline_gui.camera_intrinsics import (
    BoardSpec, detect_corners, calibrate)

SPEC = BoardSpec(pattern='chessboard', cols=11, rows=8, square_mm=45.0,
                 detector='auto', extractor='legacy')
_DATA = os.path.join(HERE, '..', '..', '..', 'data')
# Rig-A and Rig-B image folders (override with RIG_A_DIR / RIG_B_DIR)
SESSIONS = (os.environ.get('RIG_A_DIR', os.path.join(_DATA, 'rig_a')),
            os.environ.get('RIG_B_DIR', os.path.join(_DATA, 'rig_b')))

out = {}
for d in SESSIONS:
    corners, size = [], None
    for p in sorted(glob.glob(os.path.join(d, '*.png'))):
        g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        size = (g.shape[1], g.shape[0])
        c = detect_corners(g, SPEC)
        if c is not None:
            corners.append(c)
    r = calibrate(corners, SPEC, size, fix_k3=True)
    K, D = np.array(r['K']), r['dist']
    y = yaml.safe_load(open(os.path.join(d, 'ost.yaml')))
    Kr = y['camera_matrix']['data']
    Dr = y['distortion_coefficients']['data']
    print('%-20s %d frames' % (os.path.basename(d.rstrip('/')), len(corners)))
    print('   ours  fx %.2f fy %.2f cx %.2f cy %.2f  k1 %.5f k2 %.5f'
          % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], D[0], D[1]))
    print('   ROS   fx %.2f fy %.2f cx %.2f cy %.2f  k1 %.5f k2 %.5f'
          % (Kr[0], Kr[4], Kr[2], Kr[5], Dr[0], Dr[1]))
    print('   |d|   fx %.2f fy %.2f cx %.2f cy %.2f  k1 %.6f k2 %.6f'
          % (abs(K[0, 0]-Kr[0]), abs(K[1, 1]-Kr[4]), abs(K[0, 2]-Kr[2]),
             abs(K[1, 2]-Kr[5]), abs(D[0]-Dr[0]), abs(D[1]-Dr[1])))
    out[d] = (corners, K, D, Kr, Dr)

y0 = yaml.safe_load(open(os.path.join(SESSIONS[0], 'ost.yaml')))
y1 = yaml.safe_load(open(os.path.join(SESSIONS[1], 'ost.yaml')))
a, b = y0['camera_matrix']['data'], y1['camera_matrix']['data']
print('\nROS session to session: dfx %.2f dfy %.2f dcx %.2f dcy %.2f px'
      % (abs(a[0]-b[0]), abs(a[4]-b[4]), abs(a[2]-b[2]), abs(a[5]-b[5])))

corners, K, D, Kr, Dr = out[SESSIONS[0]]
obj = SPEC.object_points().astype(np.float64)

def pose_only(K, D):
    K = np.asarray(K, float).reshape(3, 3)
    D = np.asarray((list(D) + [0.0]*5)[:5], float)
    v = []
    for uv in corners:
        uv = uv.astype(np.float64)
        ok, rv, tv = cv2.solvePnP(obj, uv, K, D)
        proj, _ = cv2.projectPoints(obj, rv, tv, K, D)
        v.append(float(np.linalg.norm(proj.reshape(-1, 2) - uv, axis=1).mean()))
    return float(np.mean(v))

print('pose-only on identical corners: ROS %.3f px | ours %.3f px'
      % (pose_only(Kr, Dr), pose_only(K, D)))
