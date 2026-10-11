#!/usr/bin/env python3
"""One pass over the original Rig-D bag:
  split_bag.py <rigd_office1.bag> <out_dir>
  -> <out_dir>/rigd_office1_lio.bag   /echo1_roi0 + /imu_raw verbatim (LiDAR-inertial input, not used by the VIO runs)
  -> <out_dir>/rigd_office1_raw.bag   /imu_raw verbatim + /cam0/image_raw mono8 1920x1080 (rgb8 -> gray, header kept)
  -> <out_dir>/frames/*.png          a few full-resolution colour frames for the lens sanity check
  -> <out_dir>/stream_stats.json     message counts, stamp monotonicity, rates, header-vs-bag-time
"""
import sys, os, json, numpy as np, rosbag, cv2
from sensor_msgs.msg import Image
src, od = sys.argv[1:3]; os.makedirs(od + '/frames', exist_ok=True)
st = {}; last = {}; nimg = 0
with rosbag.Bag(src) as b, rosbag.Bag(od + '/rigd_office1_lio.bag.tmp', 'w') as lio, \
     rosbag.Bag(od + '/rigd_office1_raw.bag.tmp', 'w', compression='lz4') as raw:
    for topic, m, t in b.read_messages():
        s = st.setdefault(topic, {'n': 0, 'nonmono': 0, 'stamps': [], 'bag_minus_header': []})
        hs = m.header.stamp.to_sec(); s['n'] += 1
        if topic in last and hs <= last[topic]: s['nonmono'] += 1
        last[topic] = hs; s['stamps'].append(hs); s['bag_minus_header'].append(t.to_sec() - hs)
        if topic in ('/echo1_roi0', '/imu_raw'): lio.write(topic, m, t)
        if topic == '/imu_raw': raw.write(topic, m, t)
        if topic == '/nv12_frame':
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            if nimg % 200 == 0: cv2.imwrite(f'{od}/frames/f{nimg:04d}.png', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            o = Image(); o.header = m.header; o.header.frame_id = 'cam0'; o.height, o.width = g.shape; o.encoding = 'mono8'
            o.is_bigendian = 0; o.step = g.shape[1]; o.data = g.tobytes(); raw.write('/cam0/image_raw', o, t); nimg += 1
out = {}
for k, s in st.items():
    ts = np.array(s['stamps']); d = np.diff(ts); bm = np.array(s['bag_minus_header'])
    out[k] = {'n': s['n'], 'non_monotonic': s['nonmono'], 't0': ts[0], 't1': ts[-1], 'rate_hz': (len(ts) - 1) / (ts[-1] - ts[0]),
              'dt_median': float(np.median(d)), 'dt_max': float(d.max()), 'dt_min': float(d.min()),
              'bag_minus_header_median': float(np.median(bm)), 'bag_minus_header_p95': float(np.percentile(bm, 95))}
json.dump(out, open(od + '/stream_stats.json', 'w'), indent=1)
os.rename(od + '/rigd_office1_lio.bag.tmp', od + '/rigd_office1_lio.bag'); os.rename(od + '/rigd_office1_raw.bag.tmp', od + '/rigd_office1_raw.bag')
print(json.dumps(out, indent=1))
