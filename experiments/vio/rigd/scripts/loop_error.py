#!/usr/bin/env python3
"""loop_error.py <loop_closure_check.json> <traj.tum> ...: start->end loop error of a trajectory against the ICP-measured
true start->end pose (static first/last seconds, see loop_closure_check.py); prints translation [m] / rotation [deg]."""
import sys, json, numpy as np
from scipy.spatial.transform import Rotation as R
c = json.load(open(sys.argv[1])); ti = np.array(c['icp_start_to_end_t_m'])
for f in sys.argv[2:]:
    r = np.loadtxt(f)
    def pose(t):
        k = np.argmin(abs(r[:, 0] - t)); T = np.eye(4); T[:3, :3] = R.from_quat(r[k, 4:8]).as_matrix(); T[:3, 3] = r[k, 1:4]; return T
    t0, t1 = 1785481431.226597, 1785481594.528594
    T = np.linalg.inv(pose(t0 + 3.5)) @ pose(t1 - 3.5)
    seg = np.linalg.norm(np.diff(r[:, 1:4], axis=0), axis=1).sum()
    print(f'{f}: start->end {np.round(T[:3,3],3)} m, loop error {np.linalg.norm(T[:3,3]-ti):.3f} m over {seg:.1f} m path, z range {np.ptp(r[:,3]):.2f} m, poses {len(r)}')
