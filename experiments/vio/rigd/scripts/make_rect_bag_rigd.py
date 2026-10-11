#!/usr/bin/env python3
"""Rig-D version of ../scripts/make_rect_bag.py (monocular, optional downscale, IMU in m/s^2).
  make_rect_bag_rigd.py <raw.bag> <intrinsics.yaml> <out.bag> [--scale 0.5]
Each image is undistorted with the FULL distortion model of the set (5 radtan coefficients, or 8 for the
OpenCV-rational mrcal OPENCV8 set): cv2.initUndistortRectifyMap(K, D, R=I, newK=K, same size 1920x1080),
bilinear remap, mono8; then (scale != 1) downscaled with INTER_AREA to 960x540. The VIO configs get
K scaled by s with the pixel-centre convention (f' = s f, c' = s (c + 0.5) - 0.5) and ZERO distortion.
IMU: /imu_raw copied with linear_acceleration * 9.80665 (the Rig-D IMU reports g), samples whose header stamp
does not increase are dropped (5 duplicated stamps in rigd_office1), gyro untouched (rad/s).
Writes <out.bag> (lz4) and <out.bag>.K.yaml with the scaled pinhole parameters.
"""
import argparse, yaml, rosbag, numpy as np, cv2
from sensor_msgs.msg import Image
ap = argparse.ArgumentParser(); ap.add_argument('src'); ap.add_argument('intr'); ap.add_argument('dst'); ap.add_argument('--scale', type=float, default=0.5)
ap.add_argument('--acc_bias', type=float, nargs=3, default=[0.0, 0.0, 0.0], help='constant accelerometer bias [m/s^2] subtracted after the g -> m/s^2 conversion (identical for every set)')
a = ap.parse_args()
d = yaml.safe_load(open(a.intr))
K = np.array([[d['cam_fx'], 0, d['cam_cx']], [0, d['cam_fy'], d['cam_cy']], [0, 0, 1]], float)
D = np.array(d['distortion_coeffs'], float); assert len(D) in (4, 5, 8), D
W, H = int(d['cam_width']), int(d['cam_height'])
m1, m2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), K, (W, H), cv2.CV_32FC1)
s = a.scale; w2, h2 = int(round(W * s)), int(round(H * s))
Ks = {'fx': float(s * K[0, 0]), 'fy': float(s * K[1, 1]), 'cx': float(s * (K[0, 2] + 0.5) - 0.5), 'cy': float(s * (K[1, 2] + 0.5) - 0.5), 'width': w2, 'height': h2,
      'scale': s, 'acc_bias_subtracted': [float(x) for x in a.acc_bias], 'source': a.intr, 'D_full': D.tolist()}
yaml.safe_dump(Ks, open(a.dst + '.K.yaml', 'w'), sort_keys=False)
print('K', K.diagonal()[:2], K[:2, 2], 'D', D, '-> scaled', Ks)
n = {}; last_imu = -1.0; dropped = 0
with rosbag.Bag(a.src) as b, rosbag.Bag(a.dst, 'w', compression='lz4') as o:
    for topic, m, t in b.read_messages(topics=['/imu_raw', '/cam0/image_raw']):
        if topic == '/imu_raw':
            ts = m.header.stamp.to_sec()
            if ts <= last_imu: dropped += 1; continue
            last_imu = ts
            m.linear_acceleration.x = m.linear_acceleration.x * 9.80665 - a.acc_bias[0]; m.linear_acceleration.y = m.linear_acceleration.y * 9.80665 - a.acc_bias[1]; m.linear_acceleration.z = m.linear_acceleration.z * 9.80665 - a.acc_bias[2]
            o.write(topic, m, m.header.stamp)
        else:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
            r = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
            if s != 1.0: r = cv2.resize(r, (w2, h2), interpolation=cv2.INTER_AREA)
            msg = Image(); msg.header = m.header; msg.height, msg.width = r.shape; msg.encoding = 'mono8'
            msg.is_bigendian = 0; msg.step = r.shape[1]; msg.data = r.tobytes(); o.write(topic, msg, m.header.stamp)
        n[topic] = n.get(topic, 0) + 1
print(a.dst, n, 'imu dropped (non-increasing stamp):', dropped)
