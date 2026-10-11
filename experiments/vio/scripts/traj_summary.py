#!/usr/bin/env python3
"""Print duration, path length, extent and max speed of a TUM file (quick sanity check)."""
import sys, numpy as np
for fn in sys.argv[1:]:
    a = np.loadtxt(fn)
    if a.ndim != 2 or len(a) < 2:
        print(f'{fn}: {0 if a.ndim != 2 else len(a)} poses'); continue
    t, p = a[:, 0], a[:, 1:4]
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dt = np.diff(t); v = seg[dt > 1e-6] / dt[dt > 1e-6]
    print(f'{fn}: {len(a)} poses, {t[-1]-t[0]:.1f} s, path {seg.sum():.2f} m, bbox {np.ptp(p, axis=0).round(2)} m, '
          f'start->end {np.linalg.norm(p[-1]-p[0]):.2f} m, max speed {v.max():.2f} m/s, max |p| {np.linalg.norm(p,axis=1).max():.2f} m')
