#!/usr/bin/env python
"""3 x 5 qualitative comparison on opencalib_front: raw, ROS-rectified,
and rectified with the parameters this paper's pipeline selects."""
import json, os, glob
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DS   = os.path.join(HERE, '..', 'opencalib_front')
OUTD = os.path.join(HERE, 'out')
FIG  = os.path.join(HERE, '..', '..', '..', 'figures', 'out')
os.makedirs(FIG, exist_ok=True)

def load(p):
    d = json.load(open(p))
    K = np.array(d['K'], float)
    D = np.array((list(d['dist']) + [0.0]*5)[:5], float)
    return K, D, tuple(d['image_size'])

Kr, Dr, size = load(os.path.join(OUTD, 'opencalib_front_ros.json'))
Ko, Do, _    = load(os.path.join(OUTD, 'opencalib_front_rap.json'))
w, h = size
print('ROS  k=', np.round(Dr, 5))
print('ours k=', np.round(Do, 5))

# --- how far apart are the two rectifications, along the image diagonal ----
cx, cy = Ko[0, 2], Ko[1, 2]
half = np.hypot(max(cx, w-cx), max(cy, h-cy))
rows = []
for frac in (0.2, 0.4, 0.6, 0.8, 0.81, 0.9, 1.0):
    r = frac*half
    p = np.array([[[cx + r/np.sqrt(2), cy + r/np.sqrt(2)]]], np.float64)
    a = cv2.undistortPoints(p, Kr, Dr, P=Kr).reshape(2)
    b = cv2.undistortPoints(p, Ko, Do, P=Ko).reshape(2)
    rows.append((frac, float(np.linalg.norm(a-b))))
    print('  r/half=%.2f  |ROS - ours| = %.2f px' % (frac, rows[-1][1]))
json.dump(rows, open(os.path.join(OUTD, 'opencalib_front_divergence.json'), 'w'))

# --- pick five frames whose board reaches different parts of the frame ----
names = ['4.png', '8.png', '13.png', '18.png', '23.png']
paths = [os.path.join(DS, n) for n in names]
for p in paths:
    assert os.path.exists(p), p

plt.rcParams.update({'font.size': 7, 'font.family': 'serif',
                     'font.serif': ['Times New Roman', 'DejaVu Serif'],
                     'savefig.bbox': 'tight', 'savefig.pad_inches': 0.01})
fig, ax = plt.subplots(3, 5, figsize=(7.0, 3.05))
labels = ['raw', 'ROS', 'ours']
for c, p in enumerate(paths):
    img = cv2.imread(p)
    rect_r = cv2.undistort(img, Kr, Dr, None, Kr)
    rect_o = cv2.undistort(img, Ko, Do, None, Ko)
    for r, im in enumerate((img, rect_r, rect_o)):
        a = ax[r, c]
        a.imshow(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
        a.set_xticks([]); a.set_yticks([])
        for s in a.spines.values():
            s.set_linewidth(0.4); s.set_color('#888')
        if c == 0:
            a.set_ylabel(labels[r], fontsize=7.5, labelpad=2)
fig.subplots_adjust(wspace=0.02, hspace=0.02)
fig.savefig(os.path.join(FIG, 'fig_undistort.pdf'), dpi=200)
print('wrote fig_undistort.pdf')
