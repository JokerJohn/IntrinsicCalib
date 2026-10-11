#!/usr/bin/env python
"""Fig. 3: distortion-order selection on Rig-A.

(a) mean corner error of both candidates on the calibration set and on the
    validation views;
(b) per-view residual against the radial distance from the principal point.

Inputs are cached in data/figdata.json, written by the headless run of the
pipeline (experiment/rap_headless.py) on the Rig-A session.
"""
import sys, os, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plotstyle import (apply_style, save, axbox, xlabel,
                       SIZES, COL_W, C_M2, C_M3, C_M2_E, C_M3_E)
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

apply_style()
HERE = os.path.dirname(os.path.abspath(__file__))
d = json.load(open(os.path.join(HERE, 'data', 'figdata.json')))
m2, m3, rad = d['m2'], d['m3'], d['radial']

fig, ax = plt.subplots(1, 2, figsize=(COL_W, 1.20),
                       gridspec_kw={'width_ratios': [0.82, 1.0]})

# ------------------------------------------------------------------ panel (a)
a = ax[0]
gx, w = np.arange(3), 0.32
v2 = [m2['calib'], m2['val'], m2['valmax']]
v3 = [m3['calib'], m3['val'], m3['valmax']]
a.bar(gx - w / 2 - 0.02, v2, w, color=C_M2, edgecolor=C_M2_E, lw=0.7,
      label=r'$\mathrm{M}_2$')
a.bar(gx + w / 2 + 0.02, v3, w, color=C_M3, edgecolor=C_M3_E, lw=0.7,
      label=r'$\mathrm{M}_3$')
a.set_xticks(gx)
a.set_xticklabels(['calib.\nset', 'valid.\nmean', 'valid.\nmax'])
a.set_ylabel('error [px]', labelpad=1.5)
a.set_ylim(0, 0.86)
a.legend(loc='upper center', ncol=2, fontsize=SIZES['legend'] + 1.0,
         columnspacing=0.8, handlelength=0.9, borderaxespad=0.3)
axbox(a)
a.tick_params(axis='x', pad=3.0)

# ------------------------------------------------------------------ panel (b)
b = ax[1]
D2 = np.array([-0.1167,  0.0090, 0.0, 0.0, 0.0000])   # M2 distortion vector
D3 = np.array([-0.0928, -0.0187, 0.0, 0.0, 0.0091])   # M3 distortion vector
f3 = 0.5 * (661.73 + 662.43)                          # mean focal length [px]
cx, cy, IW, IH = 986.94, 537.62, 1920, 1080
r_corner = float(np.hypot(max(cx, IW - cx), max(cy, IH - cy)))
r_obs = 841.0                    # furthest calibration point, not a bin centre


def radial_scale(D, rho):
    return 1 + D[0] * rho ** 2 + D[1] * rho ** 4 + D[4] * rho ** 6


rho = np.linspace(0, 2.6, 4000)
s2, s3 = radial_scale(D2, rho), radial_scale(D3, rho)
r_img = f3 * rho * s3
keep = (r_img <= r_corner) & (r_img >= 0)
rs = r_img[keep]
diff = 100.0 * np.abs(s2 - s3)[keep] / np.abs(s3)[keep]
print('at r_obs=%.0f px: %.2f%%   at corner=%.0f px: %.1f%%'
      % (r_obs, np.interp(r_obs, rs, diff), r_corner, diff[-1]))

b.plot(rad['M2'][0], rad['M2'][1], '-o', color=C_M2, ms=1.8)
b.plot(rad['M3'][0], rad['M3'][1], '-s', color=C_M3, ms=1.8)
r_max = max(rad['M2'][0] + rad['M3'][0])   # crop to the observed range
b.set_xlim(0, r_max * 1.04)
b.set_ylim(0, 0.82)

b.legend(handles=[Line2D([], [], color=C_M2, marker='o', ms=2.2,
                         label=r'$\mathrm{M}_2$'),
                  Line2D([], [], color=C_M3, marker='s', ms=2.2,
                         label=r'$\mathrm{M}_3$')],
         loc='upper center', ncol=2, fontsize=SIZES['legend'] + 1.0,
         columnspacing=0.8, borderaxespad=0.3, handlelength=0.9)
# the two panels share the same unit; 0.00 is already fixed by the x-axis.
# Keep only ticks inside the limits, then restore them: set_yticks() would
# otherwise widen the view back out to the locator's outermost tick.
_lo, _hi = b.get_ylim()
b.set_yticks([t for t in b.get_yticks() if 1e-9 < t <= _hi])
b.set_ylim(_lo, _hi)
axbox(b, grid='both')
b.tick_params(axis='x', pad=3.0)

fig.tight_layout(pad=0.2, w_pad=1.4)
save(fig, 'fig_order.pdf')
