#!/usr/bin/env python3
"""Camera-IMU time offset from the yaw channel (the only well-excited rotation of a ground robot).
  td_yaw_fit.py <check_pairs.npz> <extrinsic_cam_lidar.yaml> <out.json>
Uses the per-frame-pair visual rotations saved by check_cam_imu.py. For each td (t_imu = t_cam + td) the gyro is
integrated over [t_k+td, t_{k+1}+td], rotated into the camera with R_cam_imu = Rcl (R_lidar_imu = I), and the
camera-y component (vertical axis: cam y = -LiDAR z) is compared with the visual one on all moving pairs
(median KLT flow > 2 px). Reports RMS residual vs td, best td, the yaw gain (visual/gyro least-squares slope),
the yaw correlation, and the best td on each half of the sequence (clock-drift check).
Also fits the full hand-eye rotation on pairs with > 2 deg rotation (observable part only)."""
import sys, json, numpy as np, yaml
from scipy.spatial.transform import Rotation as Rot
z = np.load(sys.argv[1]); e = yaml.safe_load(open(sys.argv[2])); Rci = np.array(e['Rcl']).reshape(3, 3)
it, g, ct, rvc, ninl, flow = z['imu_t'], z['gyr'], z['cam_t'], z['rv_c'], z['ninl'], z['flow']
def integ(t0, t1, n=20):
    s = np.linspace(t0, t1, n + 1); w = np.column_stack([np.interp(s, it, g[:, k]) for k in range(3)])
    R = Rot.identity()
    for k in range(n): R = R * Rot.from_rotvec(0.5 * (w[k] + w[k + 1]) * (s[k + 1] - s[k]))
    return R.as_rotvec()
sel = (ninl > 50) & (flow > 2.0)
def pred(td): return np.array([integ(a + td, b + td) for a, b in ct]) @ Rci.T
tds = np.round(np.arange(-0.20, 0.2001, 0.005), 4); rms = []
yv = rvc[:, 1]
for td in tds:
    p = pred(td)[:, 1]; rms.append(float(np.sqrt(np.mean(np.degrees(yv[sel] - p[sel]) ** 2))))
rms = np.array(rms); k = int(np.argmin(rms))
fine = np.round(np.arange(tds[k] - 0.006, tds[k] + 0.00601, 0.001), 4)
fr = [float(np.sqrt(np.mean(np.degrees(yv[sel] - pred(t)[sel, 1]) ** 2))) for t in fine]; td = float(fine[int(np.argmin(fr))])
P = pred(td); p = P[:, 1]
gain = float(np.dot(p[sel], yv[sel]) / np.dot(p[sel], p[sel])); corr = float(np.corrcoef(p[sel], yv[sel])[0, 1])
half = {}
idx = np.arange(len(ct))
for name, h in (('first_half', idx < len(ct) // 2), ('second_half', idx >= len(ct) // 2)):
    v = [np.sqrt(np.mean(np.degrees(yv[sel & h] - pred(t)[sel & h, 1]) ** 2)) for t in np.arange(td - 0.04, td + 0.0401, 0.002)]
    half[name] = float(np.arange(td - 0.04, td + 0.0401, 0.002)[int(np.argmin(v))])
# hand-eye on strongly rotating pairs
big = sel & (np.linalg.norm(rvc, axis=1) > np.radians(2.0))
rvi = P @ Rci  # back to IMU frame
H = rvi[big].T @ rvc[big]; U, S, Vt = np.linalg.svd(H); D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]); Rf = Vt.T @ D @ U.T
# angle between the IMU vertical axis mapped by the composed and the free estimate (the observable part for yaw-dominant motion)
axis_c = Rci @ np.array([0, 0, 1.0]); axis_f = Rf @ np.array([0, 0, 1.0])
res = {'n_pairs_used': int(sel.sum()), 'n_pairs_gt2deg': int(big.sum()), 'td_grid': tds.tolist(), 'yaw_rms_deg_grid': rms.tolist(),
       'td_best_s': td, 'yaw_rms_deg_at_best': float(min(fr)), 'yaw_rms_deg_at_td0': float(rms[np.argmin(abs(tds))]),
       'yaw_rms_deg_at_pm50ms': [float(rms[np.argmin(abs(tds + 0.05))]), float(rms[np.argmin(abs(tds - 0.05))])],
       'yaw_gain_visual_over_gyro': gain, 'yaw_corr': corr, 'td_first_half_s': half['first_half'], 'td_second_half_s': half['second_half'],
       'imu_vertical_axis_in_cam_composed_vs_free_deg': float(np.degrees(np.arccos(np.clip(axis_c @ axis_f, -1, 1)))),
       'full_rotation_composed_vs_free_deg': float(np.degrees(Rot.from_matrix(Rf @ Rci.T).magnitude())),
       'singular_values_handeye': S.tolist(),
       'resid_rms_deg_all_axes_composed': np.sqrt(np.mean(np.degrees(rvc[big] - rvi[big] @ Rci.T) ** 2, axis=0)).tolist(),
       'resid_rms_deg_all_axes_free': np.sqrt(np.mean(np.degrees(rvc[big] - rvi[big] @ Rf.T) ** 2, axis=0)).tolist()}
json.dump(res, open(sys.argv[3], 'w'), indent=1)
print(json.dumps({k: v for k, v in res.items() if not k.endswith('grid')}, indent=1))
print('td grid / yaw rms:', ' '.join(f'{t:+.3f}:{r:.4f}' for t, r in zip(tds[::4], rms[::4])))
