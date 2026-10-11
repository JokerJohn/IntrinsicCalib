#!/usr/bin/env python3
"""Tilt part of the LiDAR-IMU rotation, independent of the camera: on the static first 2 s of rigd_office1 the floor
normal in the LiDAR frame (RANSAC + SVD plane) must equal the gravity direction in the IMU frame if R_lidar_imu = I.
  lidar_imu_gravity_check.py <lio.bag> <out.json>"""
import sys, rosbag, numpy as np, json
b = rosbag.Bag(sys.argv[1]); pts = []; acc = []; T0 = None
for t, m, ts in b.read_messages():
    s = m.header.stamp.to_sec(); T0 = T0 or s
    if s > T0 + 2.0: break
    if t == '/imu_raw': acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    else:
        a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step); pts.append(a[:, :12].copy().view(np.float32).reshape(-1, 3))
P = np.vstack(pts).astype(float); g = np.mean(acc, 0); gn = g / np.linalg.norm(g)
low = P[P[:, 2] < P[:, 2].min() + 0.3]; rng = np.random.default_rng(0); best = None
for i in range(2000):
    s = low[rng.choice(len(low), 3, replace=False)]; n = np.cross(s[1] - s[0], s[2] - s[0])
    if np.linalg.norm(n) < 1e-9: continue
    n /= np.linalg.norm(n); d = -n @ s[0]; inl = np.abs(P @ n + d) < 0.02
    if best is None or inl.sum() > best[0]: best = (int(inl.sum()), inl)
Q = P[best[1]]; c = Q.mean(0); _, _, Vt = np.linalg.svd(np.cov((Q - c).T)); n = Vt[2]; n *= np.sign(n[2])
r = {'acc_static_mean_g': g.tolist(), 'imu_z_vs_gravity_deg': float(np.degrees(np.arccos(gn[2]))),
     'floor_normal_lidar': n.tolist(), 'floor_inliers': best[0], 'floor_height_m': float(-(n @ c)),
     'lidar_z_vs_floor_normal_deg': float(np.degrees(np.arccos(n[2]))),
     'floor_normal_lidar_vs_gravity_imu_deg (R_lidar_imu = I)': float(np.degrees(np.arccos(n @ gn)))}
json.dump(r, open(sys.argv[2], 'w'), indent=1); print(json.dumps(r, indent=1))
