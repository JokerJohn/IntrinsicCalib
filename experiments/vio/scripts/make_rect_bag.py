#!/usr/bin/env python3
"""Pre-rectify a raw-image VIO bag with the FULL distortion model of an intrinsics set.
  make_rect_bag.py <raw_vio.bag> <cam0_intrinsics.yaml> <cam1_intrinsics.yaml> <out.bag>
Each camera is undistorted with its own set: cv2.initUndistortRectifyMap(K, D, R=I, newK=K, same size),
bilinear remap, mono8, with D = the full `distortion_coeffs` list of the set: 5 coefficients
[k1 k2 p1 p2 k3] for the radtan sets, 8 coefficients [k1 k2 p1 p2 k3 k4 k5 k6] (OpenCV rational model)
for the mrcal LENSMODEL_OPENCV8 set (2026-10-08; for the 5-coefficient sets the list equals cam_d0..cam_d4,
so their maps are unchanged). Falls back to cam_d0..cam_d4 when `distortion_coeffs` is absent.
IMU topics are copied. The VIO systems then use (fx fy cx cy) of the set and ZERO distortion.
"""
import sys, yaml, rosbag, numpy as np, cv2
from sensor_msgs.msg import Image
src, f0, f1, dst = sys.argv[1:5]
def load(f):
    d = yaml.safe_load(open(f))
    K = np.array([[d['cam_fx'], 0, d['cam_cx']], [0, d['cam_fy'], d['cam_cy']], [0, 0, 1]], float)
    D = np.array(d['distortion_coeffs'] if 'distortion_coeffs' in d else
                 [d.get('cam_d%d' % i, 0.0) for i in range(5)], float)
    assert len(D) in (4, 5, 8), D
    return K, D, (int(d['cam_width']), int(d['cam_height']))
maps = {}
for cam, f in (('cam0', f0), ('cam1', f1)):
    K, D, size = load(f)
    maps[cam] = cv2.initUndistortRectifyMap(K, D, np.eye(3), K, size, cv2.CV_32FC1)
    print(f'{cam}: K diag {K[0,0]:.2f} {K[1,1]:.2f} c {K[0,2]:.2f} {K[1,2]:.2f} D {D}')
n = {}
with rosbag.Bag(src) as b, rosbag.Bag(dst, 'w', compression='lz4') as o:
    for topic, m, t in b.read_messages():
        if topic.endswith('/image_raw'):
            cam = 'cam0' if '/cam0/' in topic else 'cam1'
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
            rect = cv2.remap(img, maps[cam][0], maps[cam][1], cv2.INTER_LINEAR)
            msg = Image(); msg.header = m.header; msg.height, msg.width = rect.shape; msg.encoding = 'mono8'
            msg.is_bigendian = 0; msg.step = rect.shape[1]; msg.data = rect.tobytes(); o.write(topic, msg, t)
        else:
            o.write(topic, m, t)
        n[topic] = n.get(topic, 0) + 1
print(dst, n)
