#!/usr/bin/env python3
"""Derive FAST-LIVO2 extrinsics (LiDAR->IMU: extrinsic_R/extrinsic_T, LiDAR->camera: Rcl/Pcl)
from Kalibr T_cam_imu and FAST-Calib T_cam_lidar.

  T_imu_lidar = inv(T_cam_imu) @ T_cam_lidar

Writes extrinsics/derived_extrinsics.yaml with every candidate and a consistency table.
"""
import numpy as np, yaml, os, sys
np.set_printoptions(precision=6, suppress=True)
HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, '..', '..', '..', 'data', 'rigc')
kal = yaml.safe_load(open(f'{D}/kalibr_D2/calib-camchain-imu0-cam.yaml'))
T_c_i = {c: np.array(kal[c]['T_cam_imu']) for c in ('cam0', 'cam1')}
T_c1_c0 = np.array(kal['cam1']['T_cn_cnm1'])
G = os.path.join(HERE, '..', 'extrinsics', 'fastcalib')   # LiDAR-camera extrinsics from the FAST-Calib tool
cands = {
  'april_20260725_191352_cam0': (f'{G}/april_20260725_191352_cam0.yaml', 'cam0'),
  'april_20260725_191352_cam1': (f'{G}/april_20260725_191352_cam1.yaml', 'cam1'),
  'gui_20260727_203627_cam0':   (f'{G}/gui_20260727_203627_cam0.yaml', 'cam0'),
  'gui_20260727_003404_cam1':   (f'{G}/gui_20260727_003404_cam1.yaml', 'cam1'),
}
def inv(T):
    Ti = np.eye(4); Ti[:3, :3] = T[:3, :3].T; Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]; return Ti
def rotdeg(R): return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
out = {}
for name, (path, cam) in cands.items():
    Tcl = np.array(yaml.safe_load(open(path))['T_cam_lidar'])
    Til = inv(T_c_i[cam]) @ Tcl
    out[name] = {
        'source_T_cam_lidar': path, 'camera': cam,
        'extrinsic_R': [float(x) for x in Til[:3, :3].reshape(-1)],
        'extrinsic_T': [float(x) for x in Til[:3, 3]],
        'Rcl': [float(x) for x in Tcl[:3, :3].reshape(-1)],
        'Pcl': [float(x) for x in Tcl[:3, 3]],
    }
    print(f'{name}: T_cam_lidar t={Tcl[:3,3]}  ->  T_imu_lidar t={Til[:3,3]} (|t|={np.linalg.norm(Til[:3,3]):.4f} m)')
    print('  R_imu_lidar =\n', Til[:3, :3])
print('\nConsistency of T_imu_lidar derived from cam0 vs cam1 (should be ~0 if both calibrations agree):')
cons = {}
for a in cands:
    for b in cands:
        if cands[a][1] == 'cam0' and cands[b][1] == 'cam1':
            Ta = np.array(out[a]['extrinsic_R']).reshape(3, 3); ta = np.array(out[a]['extrinsic_T'])
            Tb = np.array(out[b]['extrinsic_R']).reshape(3, 3); tb = np.array(out[b]['extrinsic_T'])
            A = np.eye(4); A[:3, :3] = Ta; A[:3, 3] = ta
            B = np.eye(4); B[:3, :3] = Tb; B[:3, 3] = tb
            Dm = inv(A) @ B
            cons[f'{a} vs {b}'] = {'dt_mm': [float(x) for x in 1000 * Dm[:3, 3]], 'dR_deg': rotdeg(Dm[:3, :3])}
            print(f'  {a} vs {b}: dt = {np.round(1000*Dm[:3,3],1)} mm, dR = {rotdeg(Dm[:3,:3]):.3f} deg')
os.makedirs(os.path.join(os.path.dirname(__file__), '..', 'extrinsics'), exist_ok=True)
with open(os.path.join(os.path.dirname(__file__), '..', 'extrinsics', 'derived_extrinsics.yaml'), 'w') as f:
    yaml.safe_dump({'convention': 'extrinsic_R/extrinsic_T: p_imu = R p_lidar + T ; Rcl/Pcl: p_cam = Rcl p_lidar + Pcl ; T_imu_lidar = inv(T_cam_imu[Kalibr]) @ T_cam_lidar[FAST-Calib]',
                    'candidates': out, 'consistency_cam0_vs_cam1': cons}, f, sort_keys=False, default_flow_style=None)
print('written extrinsics/derived_extrinsics.yaml')
