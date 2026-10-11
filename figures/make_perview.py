#!/usr/bin/env python
"""Per-view residuals (not in the paper): per-view residual of every acquired Rig-A image
under the final model, coloured by the outcome of the two selection stages.

Input: data/figdata.json, field ``per_view`` = [(name, error, outcome), ...]
with outcome in {used, pruned, excluded}.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plotstyle import (apply_style, save, axbox, xlabel, SIZES, COL_W,
                       C_KEEP, C_STAGE2, C_STAGE1, C_CAL_E, C_VAL_E)
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

apply_style()
HERE = os.path.dirname(os.path.abspath(__file__))
d = json.load(open(os.path.join(HERE, 'data', 'figdata.json')))
per, med = d['per_view'], float(d['median_used'])

COLOUR = {'used': C_KEEP, 'pruned': C_STAGE2, 'excluded': C_STAGE1}
EDGE   = {'used': C_CAL_E, 'pruned': C_VAL_E, 'excluded': '#7A7A7A'}

fig, a = plt.subplots(figsize=(COL_W, 1.30))
for i, rec in enumerate(per):
    a.bar(i, rec[1], color=COLOUR[rec[2]], width=0.78,
          edgecolor=EDGE[rec[2]], linewidth=0.45)
a.set_yscale('log')
# plain tick labels rather than 10^x; the data span 0.09-15 px, so a linear
# axis capped at 5 would clip the five outliers this figure exists to show
a.set_yticks([0.1, 1.0, 10.0])
a.set_yticklabels(['0.1', '1', '10'])
a.minorticks_off()
a.axhline(med, color='k', ls='--', lw=0.7)
a.axhline(2 * med, color=C_STAGE2, ls=':', lw=1.0)
a.set_ylabel('per-view error [px]', labelpad=1.5)
a.set_xlim(-0.8, 39.0)
a.set_xticks([0, 9, 19, 29, 38])
a.set_xticklabels(['1', '10', '20', '30', '39'])
a.set_ylim(0.06, 90)
a.legend(handles=[Patch(facecolor=C_KEEP, edgecolor=C_CAL_E, lw=0.5,
                        label='calib. set'),
                  Patch(facecolor=C_STAGE2, edgecolor=C_VAL_E, lw=0.5,
                        label='stage 2 rej.'),
                  Patch(facecolor=C_STAGE1, edgecolor='#7A7A7A', lw=0.5,
                        label='stage 1 rej.')],
         loc='upper center', ncol=3, fontsize=SIZES['legend'] + 1.6,
         columnspacing=0.6, handlelength=0.8, handletextpad=0.3,
         borderaxespad=0.3)
axbox(a, grid='both')
a.tick_params(axis='x', pad=3.0)
fig.tight_layout(pad=0.2)
save(fig, 'fig_perview.pdf')
