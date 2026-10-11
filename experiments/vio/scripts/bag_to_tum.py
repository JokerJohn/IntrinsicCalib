#!/usr/bin/env python3
"""Convert a pose topic (nav_msgs/Odometry, geometry_msgs/PoseWithCovarianceStamped or PoseStamped)
of a rosbag to a TUM trajectory file.
usage: bag_to_tum.py <in.bag> <out.tum> [topic]   (default topic /aft_mapped_to_init)
"""
import sys, rosbag
bag, out = sys.argv[1], sys.argv[2]
topic = sys.argv[3] if len(sys.argv) > 3 else '/aft_mapped_to_init'
n = 0
with rosbag.Bag(bag) as b, open(out, 'w') as f:
    for _, m, _ in b.read_messages(topics=[topic]):
        pose = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
        p, q = pose.position, pose.orientation
        f.write('%.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n' % (m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        n += 1
print('%s: %d poses -> %s' % (topic, n, out))
