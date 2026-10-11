#!/usr/bin/env python3
"""Camera-IMU rotation and time-offset check on the Rig-D sequence (done once; the result is fixed for every set).
  check_cam_imu.py <raw.bag> <intrinsics.yaml> <Rcl source extrinsics/extrinsic_cam_lidar.yaml> <out.json>
Visual: KLT tracks between consecutive frames (images at half resolution, coordinates scaled back), undistorted
with the given set, essential matrix (RANSAC) -> relative camera rotation R_c (x_{k+1} = R_c x_k + t).
Inertial: gyro integrated over [t_k + td, t_{k+1} + td] (convention t_imu = t_cam + td, Kalibr/OpenVINS/VINS) ->
R_i. Prediction with the composed extrinsic R_cam_imu = Rcl * R_lidar_imu (R_lidar_imu = I):
log(R_c^T) = R_cam_imu log(R_i). Reports the residual vs td, the best td, the hand-eye rotation fitted freely
(SVD on the rotation vectors at the best td) and its angle to the composed one, the gyro/visual magnitude ratio.
"""
import sys, json, numpy as np, yaml, rosbag, cv2
from scipy.spatial.transform import Rotation as Rot
bagf, intrf, extf, outf = sys.argv[1:5]
d = yaml.safe_load(open(intrf)); K = np.array([[d['cam_fx'], 0, d['cam_cx']], [0, d['cam_fy'], d['cam_cy']], [0, 0, 1]]); D = np.array(d['distortion_coeffs'])
e = yaml.safe_load(open(extf)); Rcl = np.array(e['Rcl']).reshape(3, 3); Rci = Rcl @ np.eye(3)
imu_t, gyr, cam_t, rots, ninl, flow = [], [], [], [], [], []
prev = None
with rosbag.Bag(bagf) as b:
    for topic, m, t in b.read_messages(topics=['/imu_raw', '/cam0/image_raw']):
        if topic == '/imu_raw':
            imu_t.append(m.header.stamp.to_sec()); gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]); continue
        g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width); g = cv2.resize(g, (m.width // 2, m.height // 2), interpolation=cv2.INTER_AREA)
        ts = m.header.stamp.to_sec()
        if prev is not None:
            p0 = cv2.goodFeaturesToTrack(prev[1], 400, 0.01, 15)
            if p0 is not None and len(p0) > 30:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(prev[1], g, p0, None, winSize=(21, 21), maxLevel=3)
                ok = st.ravel() == 1
                a = (p0[ok].reshape(-1, 2) * 2 + 0.5); c = (p1[ok].reshape(-1, 2) * 2 + 0.5)
                if len(a) > 30:
                    na = cv2.undistortPoints(a.reshape(-1, 1, 2), K, D).reshape(-1, 2); nc = cv2.undistortPoints(c.reshape(-1, 1, 2), K, D).reshape(-1, 2)
                    E, msk = cv2.findEssentialMat(na, nc, np.eye(3), cv2.RANSAC, 0.999, 1.0 / K[0, 0])
                    if E is not None and E.shape == (3, 3):
                        n, R, tt, msk2 = cv2.recoverPose(E, na, nc, np.eye(3), mask=msk)
                        cam_t.append((prev[0], ts)); rots.append(Rot.from_matrix(R.T).as_rotvec()); ninl.append(int(n)); flow.append(float(np.median(np.linalg.norm(c - a, axis=1))))
        prev = (ts, g)
imu_t = np.array(imu_t); gyr = np.array(gyr); o = np.argsort(imu_t, kind='stable'); imu_t, gyr = imu_t[o], gyr[o]
keep = np.r_[True, np.diff(imu_t) > 0]; imu_t, gyr = imu_t[keep], gyr[keep]
cam_t = np.array(cam_t); rv_c = np.array(rots); ninl = np.array(ninl); flow = np.array(flow)
np.savez(outf.replace('.json', '_pairs.npz'), imu_t=imu_t, gyr=gyr, cam_t=cam_t, rv_c=rv_c, ninl=ninl, flow=flow)
def integ(t0, t1, n=20):
    s = np.linspace(t0, t1, n + 1); w = np.column_stack([np.interp(s, imu_t, gyr[:, k]) for k in range(3)])
    R = Rot.identity()
    for k in range(n): R = R * Rot.from_rotvec(0.5 * (w[k] + w[k + 1]) * (s[k + 1] - s[k]))
    return R.as_rotvec()
rv_i0 = np.array([integ(a, b) for a, b in cam_t])
# static pairs (median flow < 2 px) give an arbitrary essential-matrix rotation -> excluded; only pairs with >= 0.5 deg gyro rotation carry time-offset information
good = (ninl > 50) & (flow > 2.0) & (np.linalg.norm(rv_i0, axis=1) > np.radians(0.5))
def resid(td, R=Rci):
    rv_i = np.array([integ(a + td, b + td) for a, b in cam_t])
    pred = rv_i @ R.T
    r = np.linalg.norm(rv_c - pred, axis=1)
    return rv_i, r
tds = np.round(np.arange(-0.15, 0.1501, 0.005), 4); stats = []
for td in tds:
    _, r = resid(td); stats.append(float(np.median(np.degrees(r[good]))))
stats = np.array(stats); k = int(np.argmin(stats))
# refine
fine = np.arange(tds[k] - 0.006, tds[k] + 0.0061, 0.001); fs = [float(np.median(np.degrees(resid(t)[1][good]))) for t in fine]
td = float(fine[int(np.argmin(fs))])
rv_i, r = resid(td)
mov = good & (np.linalg.norm(rv_c, axis=1) > np.radians(1.0))
# free hand-eye rotation: argmin sum |rv_c - R rv_i|^2 (Kabsch)
H = rv_i[mov].T @ rv_c[mov]; U, S, Vt = np.linalg.svd(H); Dg = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]); Rfree = Vt.T @ Dg @ U.T
ang = np.degrees(Rot.from_matrix(Rfree @ Rci.T).magnitude())
_, rfree = resid(td, Rfree)
mag_ratio = float(np.median(np.linalg.norm(rv_i[mov], axis=1) / np.linalg.norm(rv_c[mov], axis=1)))
# axis-wise agreement (camera frame)
pred = rv_i @ Rci.T
corr = [float(np.corrcoef(rv_c[mov, j], pred[mov, j])[0, 1]) for j in range(3)]
res = {'intrinsics': intrf, 'n_pairs': int(len(cam_t)), 'n_good': int(good.sum()), 'selection': 'inliers>50, median KLT flow>2 px, gyro rotation>0.5 deg per frame pair', 'n_moving_gt1deg': int(mov.sum()),
       'td_grid': tds.tolist(), 'median_resid_deg_grid': stats.tolist(), 'td_best_s': td,
       'median_resid_deg_at_best': float(np.median(np.degrees(r[good]))), 'median_resid_deg_at_td0': float(stats[np.argmin(abs(tds))]),
       'median_visual_rot_deg_per_frame': float(np.median(np.degrees(np.linalg.norm(rv_c[good], axis=1)))),
       'max_gyro_rate_deg_s': float(np.degrees(np.linalg.norm(gyr, axis=1)).max()),
       'gyro_over_visual_magnitude_median': mag_ratio, 'per_axis_corr_cam_frame': corr,
       'free_handeye_vs_composed_deg': float(ang), 'median_resid_deg_free_handeye': float(np.median(np.degrees(rfree[good]))),
       'R_cam_imu_composed': Rci.tolist(), 'R_cam_imu_free': Rfree.tolist(),
       'first_half_td': None, 'second_half_td': None}
# drift check: best td on each half
for name, sel in (('first_half_td', np.arange(len(cam_t)) < len(cam_t) // 2), ('second_half_td', np.arange(len(cam_t)) >= len(cam_t) // 2)):
    v = []
    for t in np.arange(td - 0.03, td + 0.0301, 0.002):
        _, rr = resid(t); v.append(np.median(np.degrees(rr[good & sel])))
    res[name] = float(np.arange(td - 0.03, td + 0.0301, 0.002)[int(np.argmin(v))])
json.dump(res, open(outf, 'w'), indent=1)
print(json.dumps({k: v for k, v in res.items() if k not in ('td_grid', 'median_resid_deg_grid')}, indent=1))
