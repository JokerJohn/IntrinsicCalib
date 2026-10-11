#!/usr/bin/env python3
"""Write the five Rig-A intrinsics sets (all estimated on the 39 Rig-A images) as flat-key yaml (cam_fx ... cam_d4, intrinsics, distortion_coeffs)
into rigd/intrinsics/<set>.yaml.
  ROS-calib     ost.yaml of the calibration folder (real ROS camera_calibration output, plumb_bob, k3 = 0)
  mrcal         out/fs39_mrcal.json["mrcal-OPENCV8"] (LENSMODEL_OPENCV8, 39 frames, outlier rejection on)
  Kalibr        rigd/kalibr/run1/train-camchain.yaml (scripts/run_kalibr_all39.py run1; pinhole-radtan, all 39 images)
  ours-M2   cv2.calibrateCamera with CALIB_FIX_K3|K4|K5|K6 on every detected image (legacy extractor),
                exactly the '[ablation] M2 no selection' code of final_comparison.py, on all 39 images
  ours          out/fs39_rap.json written by rap_headless.py on the same images (fx 661.73, M3, 20 final)
"""
import json, os, sys, yaml, numpy as np
RIGD = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXP = os.path.abspath(os.path.join(RIGD, '..', '..', 'public', 'experiment'))   # experiments/public/experiment
sys.path.insert(0, EXP); sys.path.insert(0, os.path.join(EXP, '_qtstub'))
sys.path.insert(0, os.environ.get('CALIB_TOOL_DIR', os.path.join(RIGD, '..', '..', '..', 'tool')))
import cv2
from intrinsic_calib.camera_intrinsics import BoardSpec
from protocol import ROS_FLAGS
from rap_headless import detect_all
OUT = f'{RIGD}/intrinsics'
RIGA = os.environ.get('RIG_A_DIR', os.path.join(RIGD, '..', '..', '..', 'data', 'rig_a'))
def write(name, K, D, model, note):
    K = np.asarray(K, float); D = [float(x) for x in np.ravel(D)]
    if len(D) == 4: D = D + [0.0]
    d = {'cam_model': 'Pinhole', 'cam_width': 1920, 'cam_height': 1080, 'scale': 1.0,
         'cam_fx': float(K[0, 0]), 'cam_fy': float(K[1, 1]), 'cam_cx': float(K[0, 2]), 'cam_cy': float(K[1, 2])}
    for i in range(5): d[f'cam_d{i}'] = D[i]
    d['intrinsics'] = [d['cam_fx'], d['cam_fy'], d['cam_cx'], d['cam_cy']]; d['distortion_coeffs'] = D
    d['distortion_model'] = model; d['note'] = note
    for i in range(5, len(D)): d[f'cam_d{i}'] = D[i]
    yaml.safe_dump(d, open(f'{OUT}/{name}.yaml', 'w'), sort_keys=False)
    print(f'{name:14s} fx {K[0,0]:.3f} fy {K[1,1]:.3f} cx {K[0,2]:.3f} cy {K[1,2]:.3f} D {np.round(D, 6).tolist()}')
# 1 ROS-calib
o = yaml.safe_load(open(f'{RIGA}/ost.yaml'))
write('ROS-calib', np.array(o['camera_matrix']['data']).reshape(3, 3), o['distortion_coefficients']['data'], 'radtan',
      f'Rig-A, data/rig_a/ost.yaml (ROS camera_calibration, plumb_bob, k3=0), 39 images')
# 2 mrcal OPENCV8
m = json.load(open(f'{EXP}/out/fs39_mrcal.json'))['mrcal-OPENCV8']; assert m['lensmodel'] == 'LENSMODEL_OPENCV8' and m['n_frames'] == 39
write('mrcal', m['K'], m['dist'], 'radtan8 (OpenCV rational: k1 k2 p1 p2 k3 k4 k5 k6)',
      f"Rig-A, mrcal LENSMODEL_OPENCV8 defaults (outlier rejection on, {m['outlier_corners']}/{m['total_corners']} corners cut), 39 images, experiments/public/experiment/out/fs39_mrcal.json")
# 3 Kalibr
for run, name in (('run1', 'Kalibr'),):
    cc = f'{RIGD}/kalibr/{run}/train-camchain.yaml'
    if os.path.exists(cc):
        c = yaml.safe_load(open(cc))['cam0']; fx, fy, cx, cy = c['intrinsics']
        write(name, [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], list(c['distortion_coeffs']) + [0.0], 'radtan',
              f'Rig-A, real Kalibr kalibr_calibrate_cameras pinhole-radtan defaults, 39 images, rigd/kalibr/{run}/train-camchain.yaml')
    else: print('missing', cc)
# 4/5 ours-M2 and ours (Full)
spec = BoardSpec(pattern='chessboard', cols=11, rows=8, square_mm=45.0, detector='auto', extractor='legacy')
entries, size = detect_all(RIGA, spec); entries.sort(key=lambda e: e['name']); assert len(entries) == 39, len(entries)
sets = [e['corners'] for e in entries]
rms, K, D, _r, _t = cv2.calibrateCamera([spec.object_points()] * len(sets), sets, size, None, None, flags=ROS_FLAGS)
write('ours-M2', K, np.ravel(D)[:5], 'radtan', f'Rig-A, M2 (k3 fixed) calibrateCamera on all {len(sets)} detected images, no selection (final_comparison.py ablation), rms {rms:.3f} px')
r = json.load(open(f'{EXP}/out/fs39_rap.json'))
write('ours', r['K'], r['dist'], 'radtan', f"Rig-A, rap_headless.py = out/fs39_rap.json: {r['trace']['order']['chosen']}, {r['trace']['n_final']} final of {r['trace']['n_input']}")
