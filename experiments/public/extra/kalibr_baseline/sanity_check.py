#!/usr/bin/env python3
"""Check that the fold construction and scoring reproduce the existing
'ours-M2' row of Table I, and score M2 / ROS / mrcal on the Rig-A
six-image validation subset for reference."""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_kalibr import jobs, RIGA, EXP, pose_only  # noqa
import cv2
from protocol import ROS_FLAGS

out = {}
for tag, folder, train, ev, spec in jobs():
    from rap_headless import detect_all
    ents, size = detect_all(folder, spec)
    tr = [e for e in ents if e["name"] in set(train)]
    objp = spec.object_points().astype(np.float64)
    _r, K, D, _a, _b = cv2.calibrateCamera([spec.object_points()] * len(tr),
                                           [e["corners"] for e in tr], size, None, None,
                                           flags=ROS_FLAGS)
    m, _ = pose_only(K, D, objp, [e["corners"] for e in ev])
    pv = float(np.mean([pose_only(K, D, objp, [e["corners"]])[0] for e in ev]))
    out[tag] = {"M2_held": m, "M2_held_per_view_mean": pv, "n_train": len(tr)}
    print(tag, out[tag])
    if folder == RIGA:
        o = os.path.join(EXP, "out")
        ros = json.load(open(os.path.join(o, "fs39_ros.json")))
        mr = json.load(open(os.path.join(o, "fs39_mrcal.json")))
        refs = {"ROS (fs39_ros.json, %d frames)" % ros["n_used"]: (ros["K"], ros["dist"])}
        for k, v in mr.items():
            refs["%s (fs39_mrcal.json, %d frames)" % (k, v["n_frames"])] = (v["K"], v["dist"])
        for k, (Kr, Dr) in refs.items():
            pv = float(np.mean([pose_only(Kr, Dr, objp, [e["corners"]])[0] for e in ev]))
            out[tag][k] = pv
            print("  ", k, round(pv, 4))
json.dump(out, open(os.path.join(HERE, "sanity_check.json"), "w"), indent=1)
