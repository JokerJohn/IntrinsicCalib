#!/usr/bin/env python
"""Fig. 2: image selection.

(a) how many images each stage removes on every session;
(b) the four candidate image sets of Rig-A compared on a common validation
    subset under the lower-order model.

The numbers are the ones reported in Section V-B and are kept here explicitly
so that the figure can be redrawn without rerunning the pipeline.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plotstyle import (apply_style, save, axbox, SIZES, COL_W,
                       C_KEEP, C_STAGE2, C_STAGE1, C_CAL, C_VAL,
                       C_CAL_E, C_VAL_E)
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

apply_style()

# ------------------------------------------------------------------ panel (a)
# The right camera of each stereo pair has identical counts to the left one
# (ROS-R 11/1/0 = ROS-L, OpenCV-R 11/0/2 = OpenCV-L), so it is redundant here
# and omitted; Table I still reports both.
SESSIONS = ['Rig-A', 'OpenCalib', 'ROS-L', 'OpenCV-L']
KEPT     = [20, 22, 11, 11]
STAGE2   = [7, 0, 1, 0]
STAGE1   = [12, 0, 0, 2]
TOTAL    = [a + b + c for a, b, c in zip(KEPT, STAGE2, STAGE1)]

# ------------------------------------------------------------------ panel (b)
SETS  = ['all\n(39)', 'S2\n(38)', 'S1\n(27)', 'S1+S2\n(26)']
CALIB = [0.467, 0.454, 0.357, 0.342]     # calibration-set mean corner error
VALID = [0.419, 0.419, 0.408, 0.316]     # common validation subset

fig, ax = plt.subplots(1, 2, figsize=(COL_W, 1.22),
                       gridspec_kw={'width_ratios': [1.06, 1.0]})

a = ax[0]
x = np.arange(len(SESSIONS))
a.bar(x, KEPT, 0.62, color=C_KEEP, edgecolor=C_CAL_E, lw=0.7)
a.bar(x, STAGE2, 0.62, bottom=KEPT, color=C_STAGE2,
      edgecolor=C_VAL_E, lw=0.7)
a.bar(x, STAGE1, 0.62, bottom=[k + b for k, b in zip(KEPT, STAGE2)],
      color=C_STAGE1, edgecolor='#7A7A7A', lw=0.7)
for i in range(len(SESSIONS)):
    a.text(x[i], TOTAL[i] + 1.2, '%d' % TOTAL[i], ha='center',
           fontsize=SIZES['annot'])
    a.text(x[i], KEPT[i] / 2.0, '%d' % KEPT[i], ha='center', va='center',
           fontsize=SIZES['annot'], color='white')
a.set_xticks(x)
a.set_xticklabels(SESSIONS, rotation=20, ha='right')
a.set_ylabel('images', labelpad=1.5)
a.set_ylim(0, 45)
a.legend(handles=[Patch(facecolor=C_KEEP, edgecolor=C_CAL_E, lw=0.5,
                        label='calib. set'),
                  Patch(facecolor=C_STAGE2, edgecolor=C_VAL_E, lw=0.5,
                        label='stage 2 rej.'),
                  Patch(facecolor=C_STAGE1, edgecolor='#7A7A7A', lw=0.5,
                        label='stage 1 rej.')],
         loc='upper right', ncol=1, fontsize=SIZES['legend'] + 1.0,
         labelspacing=0.28, handlelength=0.9, handletextpad=0.35,
         borderaxespad=0.35)
axbox(a)
a.tick_params(axis='x', pad=3.0)

b = ax[1]
xb, w = np.arange(len(SETS)), 0.32
b.bar(xb - w / 2 - 0.02, CALIB, w, color=C_CAL, edgecolor=C_CAL_E, lw=0.7,
      label='calib. set')
b.bar(xb + w / 2 + 0.02, VALID, w, color=C_VAL, edgecolor=C_VAL_E, lw=0.7,
      label='valid. set')
b.set_xticks(xb)
b.set_xticklabels(SETS)
b.set_ylabel('error [px]', labelpad=1.5)
b.set_ylim(0, 0.62)
b.legend(loc='upper center', ncol=2, fontsize=SIZES['legend'] + 1.0,
         columnspacing=0.6, handlelength=0.9, handletextpad=0.3,
         borderaxespad=0.3)
axbox(b)
b.tick_params(axis='x', pad=3.0)

fig.tight_layout(pad=0.2, w_pad=0.9)
save(fig, 'fig_select2.pdf')
