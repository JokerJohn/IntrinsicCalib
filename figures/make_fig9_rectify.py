#!/usr/bin/env python
"""Rectification comparison (not in the paper), two sessions side by side.

Left block  (public wide-angle set): raw / ROS parameters / selected parameters
Right block (our own rig session)  : raw / M2 / M3, where the two orders agree
                                     inside the observed range and diverge far
                                     outside it.
Faces are blurred before anything is written to disk.
"""
import json, os, glob, sys
import numpy as np
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plotstyle import apply_style, save, SIZES
import matplotlib.pyplot as plt

SCR  = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(SCR, '..')
# results of experiments/public/experiment/run_all.sh (plus the Rig-A ROS/mrcal runs, see README)
OUTD = os.path.join(ROOT, 'experiments', 'public', 'experiment', 'out')
PUB  = os.path.join(ROOT, 'experiments', 'public', 'opencalib_front')
OURS = os.environ.get('RIG_A_DIR', os.path.join(ROOT, 'data', 'rig_a'))

def load_json_K(p):
    d = json.load(open(p))
    K = np.array(d['K'], float)
    D = np.array((list(d['dist']) + [0.0]*5)[:5], float)
    return K, D

Kr, Dr = load_json_K(os.path.join(OUTD, 'opencalib_front_ros.json'))
Ko, Do = load_json_K(os.path.join(OUTD, 'opencalib_front_rap.json'))

# mrcal at its documented example configuration (LENSMODEL_OPENCV8), fold 0
_m = [1055.14563, 1056.718188, 963.0442501, 580.986538, -0.1273162132,
      0.0532709671, -0.0005030125432, -0.0004019162114, -0.009505004981,
      0.02290049187, -0.0402876674, 0.01150535266]
Km = np.array([[_m[0], 0, _m[2]], [0, _m[1], _m[3]], [0, 0, 1]])
Dm = np.array(_m[4:])          # k1 k2 p1 p2 k3 k4 k5 k6

_f = json.load(open(os.path.join(OUTD, 'fs39_mrcal.json')))['mrcal-OPENCV8']
Kfm = np.array(_f['K'], float); Dfm = np.array(_f['dist'], float)
_fr = json.load(open(os.path.join(OUTD, 'fs39_ros.json')))
Kfr = np.array(_fr['K'], float)
Dfr = np.array((list(_fr['dist']) + [0.0]*5)[:5], float)

B = json.load(open(os.path.join(SCR, 'data', 'resultsB.json')))['B15']
K2 = np.array([[B['M2']['fx'], 0, B['M2']['cx']],
               [0, B['M2']['fy'], B['M2']['cy']], [0, 0, 1]])
D2 = np.array([B['M2'][k] for k in ('k1', 'k2', 'p1', 'p2', 'k3')])
K3 = np.array([[B['M3']['fx'], 0, B['M3']['cx']],
               [0, B['M3']['fy'], B['M3']['cy']], [0, 0, 1]])
D3 = np.array([B['M3'][k] for k in ('k1', 'k2', 'p1', 'p2', 'k3')])

# ------------------------------------------------------------------ faces
def _haarcascade_dir():
    """Directory of the OpenCV Haar cascades: cv2.data when the wheel ships it,
    otherwise <sys.prefix>/share/opencv4/haarcascades, or $OPENCV_HAARCASCADES."""
    d = os.environ.get('OPENCV_HAARCASCADES')
    if d:
        return d.rstrip('/') + '/'
    if hasattr(cv2, 'data'):
        return cv2.data.haarcascades
    return os.path.join(sys.prefix, 'share', 'opencv4', 'haarcascades') + '/'


CASC = _haarcascade_dir()
det_face = cv2.CascadeClassifier(CASC + 'haarcascade_frontalface_default.xml')
det_prof = cv2.CascadeClassifier(CASC + 'haarcascade_profileface.xml')

def anonymize(img):
    """Blur every detected face; enlarge the box so hair and jaw go too."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    boxes = []
    for det in (det_face, det_prof):
        for sf, mn in ((1.05, 4), (1.10, 3)):
            boxes += list(det.detectMultiScale(g, sf, mn, minSize=(24, 24)))
    H, W = img.shape[:2]
    for (x, y, w, h) in boxes:
        if max(w, h) > 0.12 * W:       # reject spurious large detections
            continue
        pad = int(0.35 * max(w, h))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(img.shape[1], x + w + pad), min(img.shape[0], y + h + pad)
        roi = img[y0:y1, x0:x1]
        if roi.size:
            k = max(9, (min(roi.shape[:2]) // 2) * 2 + 1)
            img[y0:y1, x0:x1] = cv2.GaussianBlur(roi, (k, k), 0)
    return img, len(boxes)

# ------------------------------------------------------------------ figure
pub_names  = ['4.png', '13.png', '23.png']
ours_names = ['left-0030.png', 'left-0005.png', 'left-0034.png']

apply_style()
plt.rcParams.update({'axes.grid': False})
def crop169(im):
    """Centre-crop to a common 16:9 frame.

    The two sessions differ in aspect ratio (1920x1200 vs 1920x1080), which
    left the panels filling their cells unevenly.  Rectification is applied to
    the full image first and only the *display* is cropped, so the geometry
    shown is unchanged.
    """
    h, w = im.shape[:2]
    hh = int(round(w * 9.0 / 16.0))
    if hh < h:
        y0 = (h - hh) // 2
        return im[y0:y0 + hh]
    ww = int(round(h * 16.0 / 9.0))
    x0 = (w - ww) // 2
    return im[:, x0:x0 + ww]


fig, ax = plt.subplots(4, 6, figsize=(7.05, 2.95))
row_lab = ['raw', 'ROS-calib', 'mrcal', 'ours']
nfaces = 0

for c, n in enumerate(pub_names):
    img = cv2.imread(os.path.join(PUB, n))
    img, k = anonymize(img); nfaces += k
    for r, im in enumerate((img,
                            cv2.undistort(img, Kr, Dr, None, Kr),
                            cv2.undistort(img, Km, Dm, None, Km),
                            cv2.undistort(img, Ko, Do, None, Ko))):
        a = ax[r, c]; a.imshow(cv2.cvtColor(crop169(im), cv2.COLOR_BGR2RGB))
        a.set_xticks([]); a.set_yticks([])
        for s in a.spines.values(): s.set_linewidth(0.4); s.set_color('#999')

for c, n in enumerate(ours_names):
    img = cv2.imread(os.path.join(OURS, n))
    img, k = anonymize(img); nfaces += k
    for r, im in enumerate((img,
                            cv2.undistort(img, Kfr, Dfr, None, Kfr),
                            cv2.undistort(img, Kfm, Dfm, None, Kfm),
                            cv2.undistort(img, K3, D3, None, K3))):
        a = ax[r, c + 3]; a.imshow(cv2.cvtColor(crop169(im), cv2.COLOR_BGR2RGB))
        a.set_xticks([]); a.set_yticks([])
        for s in a.spines.values(): s.set_linewidth(0.4); s.set_color('#999')

for r in range(4):
    ax[r, 0].set_ylabel(row_lab[r], fontsize=SIZES['title'], labelpad=2)
ax[0, 1].set_title('OpenCalib', fontsize=SIZES['title'], pad=3)
ax[0, 4].set_title('Rig-A', fontsize=SIZES['title'], pad=3)
fig.subplots_adjust(wspace=0.010, hspace=0.002)
save(fig, 'fig_undistort.pdf')
print('blurred %d face boxes; wrote fig_undistort.pdf' % nfaces)
