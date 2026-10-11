#!/usr/bin/env python
"""Corner-zone coverage (not in the paper): calibration points inside the four 20% corner zones,
for the two sessions on which the distortion-order criterion runs.

An empty zone marks an image region for which the session provides no
observations, which is what bounds the validity of the order decision.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plotstyle import (apply_style, save, axbox, SIZES, COL_W,
                       C_CAL, C_VAL, C_CAL_E, C_VAL_E)
import matplotlib.pyplot as plt

apply_style()

ZONES = ['top left', 'top right', 'bottom left', 'bottom right']
COUNTS = {'Rig-A': [34, 26, 0, 0], 'OpenCalib': [8, 12, 6, 3]}

fig, a = plt.subplots(figsize=(COL_W, 1.18))
x, w = np.arange(len(ZONES)), 0.32
v1, v2 = COUNTS['Rig-A'], COUNTS['OpenCalib']
a.bar(x - w / 2 - 0.02, v1, w, color=C_CAL, edgecolor=C_CAL_E, lw=0.7,
      label='Rig-A')
a.bar(x + w / 2 + 0.02, v2, w, color=C_VAL, edgecolor=C_VAL_E, lw=0.7,
      label='OpenCalib')
for xi, v in zip(x - w / 2 - 0.02, v1):
    a.text(xi, v + 1.8, '%d' % v, ha='center',
           fontsize=SIZES['annot'] + 0.8,
           color='#c0392b' if v == 0 else 'black')
for xi, v in zip(x + w / 2 + 0.02, v2):
    a.text(xi, v + 1.8, '%d' % v, ha='center',
           fontsize=SIZES['annot'] + 0.8)
a.set_xticks(x)
a.set_xticklabels(ZONES, fontsize=SIZES['tick'] + 0.8)
a.tick_params(axis='x', pad=3.0)
a.set_ylabel('calibration points', labelpad=1.5,
              fontsize=SIZES['label'] + 0.8)
a.set_ylim(0, 45)
a.legend(loc='upper center', ncol=2, fontsize=SIZES['legend'] + 1.0,
         columnspacing=0.8, handlelength=0.9, borderaxespad=0.3)
axbox(a)
fig.tight_layout(pad=0.2)
save(fig, 'fig_zones.pdf')
