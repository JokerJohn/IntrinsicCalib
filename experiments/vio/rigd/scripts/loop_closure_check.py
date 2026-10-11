#!/usr/bin/env python3
"""Loop-closure error of the LIO reference: the robot stands still at the start (0-9 s) and at the end (>152 s) at
nearly the same place. The true start->end relative pose is obtained by point-to-plane ICP between the accumulated
static LiDAR scans of the first 5 s and the last 5 s (LiDAR frame = IMU frame up to the lever arm, R_lidar_imu = I);
it is compared with the start->end relative pose of the LIO reference.
  loop_closure_check.py <lio.bag> <ref.tum> <out.json>"""
import sys, json, rosbag, numpy as np, open3d as o3d
from scipy.spatial.transform import Rotation as R
bag, ref, out = sys.argv[1:4]
b = rosbag.Bag(bag); t0, t1 = b.get_start_time(), b.get_end_time()
def acc(lo, hi):
    P = []
    for t, m, ts in b.read_messages(topics=['/echo1_roi0']):
        s = m.header.stamp.to_sec()
        if lo <= s <= hi:
            a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step); P.append(a[:, :12].copy().view(np.float32).reshape(-1, 3))
    p = np.vstack(P).astype(float); p = p[np.linalg.norm(p, axis=1) > 0.5]
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p)).voxel_down_sample(0.05)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30)); return pc
A = acc(t0 + 1.0, t0 + 6.0); B = acc(t1 - 6.0, t1 - 1.0)
r = np.loadtxt(ref); 
def pose(t):
    k = np.argmin(abs(r[:, 0] - t)); T = np.eye(4); T[:3, :3] = R.from_quat(r[k, 4:8]).as_matrix(); T[:3, 3] = r[k, 1:4]; return T
T_lio = np.linalg.inv(pose(t0 + 3.5)) @ pose(t1 - 3.5)   # end pose expressed in start frame (LIO)
res = []
for init in (T_lio, np.eye(4)):
    reg = o3d.pipelines.registration.registration_icp(B, A, 0.5, init, o3d.pipelines.registration.TransformationEstimationPointToPlane())
    reg = o3d.pipelines.registration.registration_icp(B, A, 0.1, reg.transformation, o3d.pipelines.registration.TransformationEstimationPointToPlane())
    res.append((reg.fitness, reg.inlier_rmse, np.array(reg.transformation, copy=True)))
fit, rmse, T_icp = max(res, key=lambda x: x[0])
E = np.linalg.inv(T_icp) @ T_lio
o = {'icp_fitness': fit, 'icp_inlier_rmse_m': rmse, 'icp_start_to_end_t_m': T_icp[:3, 3].tolist(),
     'icp_start_to_end_rot_deg': float(np.degrees(R.from_matrix(T_icp[:3, :3]).magnitude())),
     'lio_start_to_end_t_m': T_lio[:3, 3].tolist(), 'lio_start_to_end_rot_deg': float(np.degrees(R.from_matrix(T_lio[:3, :3]).magnitude())),
     'loop_error_trans_m': float(np.linalg.norm(E[:3, 3])), 'loop_error_trans_xyz_m': E[:3, 3].tolist(),
     'loop_error_rot_deg': float(np.degrees(R.from_matrix(E[:3, :3]).magnitude())),
     'per_init': [{'init': n, 'fitness': f, 'rmse': e, 't': T[:3, 3].tolist()} for n, (f, e, T) in zip(('lio', 'identity'), res)],
     'both_inits_agree_m': float(np.linalg.norm(res[0][2][:3, 3] - res[1][2][:3, 3]))}
json.dump(o, open(out, 'w'), indent=1); print(json.dumps(o, indent=1))
