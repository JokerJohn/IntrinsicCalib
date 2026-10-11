#!/usr/bin/env python3
"""Write a ROS1 bag with one sensor_msgs/Image (mono8) per listed image,
on /cam0/image_raw, stamps 0.1 s apart.  Runs inside the Kalibr container.

usage: make_bag.py out.bag img1 img2 ...
"""
import sys
import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image

out, paths = sys.argv[1], sys.argv[2:]
with rosbag.Bag(out, "w") as bag:
    for i, p in enumerate(paths):
        g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        assert g is not None, p
        m = Image()
        m.header.seq = i
        m.header.stamp = rospy.Time(1.0 + 0.1 * i)
        m.header.frame_id = "cam0"
        m.height, m.width = g.shape
        m.encoding = "mono8"
        m.is_bigendian = 0
        m.step = g.shape[1]
        m.data = g.tobytes()
        bag.write("/cam0/image_raw", m, m.header.stamp)
print("wrote %d images to %s" % (len(paths), out))
