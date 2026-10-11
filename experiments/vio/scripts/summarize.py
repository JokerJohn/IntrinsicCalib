#!/usr/bin/env python3
"""Markdown table of ATE/RPE results in results/<set>/<bag>/*_vs_lio.json (plus trajectory coverage)."""
import glob, json, os, sys, numpy as np
root = sys.argv[1]
rows = []
for j in sorted(glob.glob(os.path.join(root, '*', '*_vs_lio.json'))):
    r = json.load(open(j)); bag = os.path.basename(os.path.dirname(j)); sysname = os.path.basename(j).replace('_vs_lio.json', '')
    est = np.loadtxt(r['est'], comments='#'); ref = np.loadtxt(r['ref'], comments='#')
    g = lambda k, f='rmse': r.get(k, {}).get(f, float('nan'))
    cov = (est[-1, 0] - est[0, 0]) / (ref[-1, 0] - ref[0, 0]) * 100 if est.ndim == 2 and len(est) > 1 else 0
    rows.append((bag, sysname, g('ate_trans_m'), g('ate_rot_deg'), g('rpe_1m_trans_m'), g('rpe_1m_rot_deg'), g('rpe_5m_trans_m'), g('rpe_5m_rot_deg'), len(est) if est.ndim == 2 else 0, cov))
print(f'# {os.path.basename(root)}\n')
print('| bag | system | ATE trans RMSE [m] | ATE rot RMSE [deg] | RPE@1m trans [m] | RPE@1m rot [deg] | RPE@5m trans [m] | RPE@5m rot [deg] | poses | time coverage [%] |')
print('|---|---|---|---|---|---|---|---|---|---|')
for b, s, *v in rows:
    print(f'| {b} | {s} | {v[0]:.4f} | {v[1]:.3f} | {v[2]:.4f} | {v[3]:.3f} | {v[4]:.4f} | {v[5]:.3f} | {v[6]} | {v[7]:.0f} |')
