#!/usr/bin/env python3
"""Drop LiDAR points whose per-point 'timestamp' lies outside [header.stamp, header.stamp + 0.15 s].
  sanitize_lio_bag.py <in_lio.bag> <out_lio.bag>
rigd_office1 /echo1_roi0 contains corrupted per-point stamps (e.g. +1.6e13 s at t0+84.8 s); FAST-LIVO2's FSLIDAR handler
has no upper bound, the scan end time becomes huge and the LIO sync blocks forever (reference stopped after 84 s).
Nothing else is changed (IMU and all other points copied verbatim)."""
import sys, rosbag, numpy as np
src, dst = sys.argv[1:3]; nb = ns = 0; tot = 0
with rosbag.Bag(src) as b, rosbag.Bag(dst, 'w') as o:
    for t, m, ts in b.read_messages():
        if t == '/echo1_roi0':
            a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
            pt = a[:, 24:32].copy().view(np.float64).ravel() * 1e-6 - m.header.stamp.to_sec(); tot += len(pt)
            ok = (pt >= -1e-4) & (pt <= 0.15)
            if not ok.all():
                ns += 1; nb += int((~ok).sum())
                print(f'scan t0+{m.header.stamp.to_sec()-1785481431.226597:.2f}s: {int((~ok).sum())} bad of {len(pt)}, bad dt {pt[~ok][:3]}')
                a = a[ok]; m.data = a.tobytes(); m.width = len(a); m.row_step = m.width * m.point_step
        o.write(t, m, ts)
print(f'{ns} scans affected, {nb} of {tot} points dropped')
