#!/usr/bin/env python3
"""Assemble kalibr_table1.json from runs_folds_raw.json (primary run) and
runs_repeat2_folds_raw.json (repeat run; Kalibr shuffles views unseeded).
std = population std over the 4 folds (ddof=0), as in the existing Table I."""
import json, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
SESS = ["opencalib_front", "ros_wide_left", "ros_wide_right", "opencv_left", "opencv_right"]
runs = {k: json.load(open(os.path.join(HERE, f"{k}_folds_raw.json")))
        for k in ("runs", "runs_repeat2")}
out = {"protocol": "rotating 4-fold holdout (eval = entries[f::4], sorted by name), "
                   "Kalibr kalibr_calibrate_cameras pinhole-radtan at defaults on the training "
                   "frames, scored by protocol.pose_only on legacy-extractor corners; "
                   "cell = mean over folds, std ddof=0",
       "sessions": {}}
for s in SESS:
    d = {}
    for k, r in runs.items():
        f = [r[f"{s}/f{i}"] for i in range(4)]
        h = [x["held"] for x in f]
        d[k] = {"fold_held": h, "mean": float(np.mean(h)), "std": float(np.std(h)),
                "cell": "%.3f(%d)" % (np.mean(h), round(1000 * np.std(h))),
                "frames_in_bag": [x["images_in_bag"] for x in f],
                "frames_corners_extracted": [x["corners_extracted"] for x in f],
                "frames_used_after_view_selection": [x["views_used"] for x in f],
                "outlier_corners_removed": [x["outlier_corners_removed"] for x in f],
                "K": [x["K"] for x in f], "dist_k1k2p1p2k3": [x["dist"] for x in f],
                "camchain": [x["camchain"] for x in f], "failures": [x.get("error") for x in f]}
    out["sessions"][s] = {"primary": d["runs"], "repeat_run": d["runs_repeat2"]}
ra = {}
for k, r in runs.items():
    x = r["rig_a/train33"]
    ra[k] = {k2: x[k2] for k2 in ("held_per_view_mean", "held", "held_per_view", "images_in_bag",
                                  "corners_extracted", "views_used", "K", "dist")}
out["rig_a"] = {"train": "33 images (all but the six validation images)",
                "eval": "left-0000/0006/0011/0019/0024/0035, mean over views of per-view mean corner error",
                "primary": ra["runs"], "repeat_run": ra["runs_repeat2"],
                "reference_same_scoring": json.load(open(os.path.join(HERE, "sanity_check.json")))["rig_a/train33"]}
json.dump(out, open(os.path.join(HERE, "kalibr_table1.json"), "w"), indent=1)
for s in SESS:
    p = out["sessions"][s]
    print(s, p["primary"]["cell"], "repeat", p["repeat_run"]["cell"],
          "used", p["primary"]["frames_used_after_view_selection"], "/", p["primary"]["frames_in_bag"])
print("rig_a", ra["runs"]["held_per_view_mean"], ra["runs"]["views_used"], "repeat",
      ra["runs_repeat2"]["held_per_view_mean"], ra["runs_repeat2"]["views_used"])
