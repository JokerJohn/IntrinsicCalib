#!/usr/bin/env python3
"""Make a VIO-only bag: /sensors/imu0 (+/sensors/imu1) copied, /sensors/cam{0,1}/compressed decoded to
mono8 sensor_msgs/Image on /sensors/cam{0,1}/image_raw (same header stamps), LiDAR/mag dropped.
usage: make_raw_bag.py <in.bag> <out.bag>
"""
import sys, rosbag, numpy as np, cv2
from sensor_msgs.msg import Image
src, dst = sys.argv[1], sys.argv[2]
n = {}
with rosbag.Bag(src) as b, rosbag.Bag(dst, 'w', compression='lz4') as o:
    for topic, m, t in b.read_messages(topics=['/sensors/imu0', '/sensors/imu1', '/sensors/cam0/compressed', '/sensors/cam1/compressed']):
        if topic.endswith('/compressed'):
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            msg = Image(); msg.header = m.header; msg.height, msg.width = img.shape; msg.encoding = 'mono8'
            msg.is_bigendian = 0; msg.step = img.shape[1]; msg.data = img.tobytes()
            topic = topic.replace('/compressed', '/image_raw'); o.write(topic, msg, t)
        else:
            o.write(topic, m, t)
        n[topic] = n.get(topic, 0) + 1
print(dst, n)
