#!/usr/bin/env python3
"""The C1/C2/C3 controlled protocol, extended with mrcal.

Adds to protocol.py:

  C4  mrcal OPENCV4, flat board, outlier rejection ON
  C5  mrcal OPENCV4, flat board, outlier rejection OFF
  C6  mrcal OPENCV4, mrcal's shipped defaults (board flex + outliers)
  C7  mrcal OPENCV5 (adds k3), flat board, outliers ON

Every configuration sees the identical corner set on the identical training
frames, and every one is scored the same way: intrinsics frozen, six pose
parameters refitted per frame, on frames none of them ever saw.

C5 vs C2  mrcal's solver vs OpenCV's, same model, same corners, no rejection
C4 vs C5  what mrcal's per-corner outlier rejection is worth
C4 vs C3  per-corner rejection (mrcal) vs per-image selection (RAP)
C7 vs C4  mrcal's M3 vs its M2, beside RAP's own order verdict
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(HERE, "..", "..", "..", "tool")))
from fast_calib_offline_gui.camera_intrinsics import BoardSpec  # noqa: E402

import mrcal_baseline as MB                                     # noqa: E402
from protocol import pose_only, run as run_c123                 # noqa: E402
from rap_headless import detect_all                             # noqa: E402

MRCAL_LABELS = {
    "M4-flat":      "C4 mrcal OPENCV4 +outlier",
    "M4-flat-noor": "C5 mrcal OPENCV4 -outlier",
    "M4-shipped":   "C6 mrcal shipped defaults",
    "M5-flat":      "C7 mrcal OPENCV5 (+k3) ",
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--every", type=int, default=4)
    ap.add_argument("--json-out")
    args = ap.parse_args()

    allrows = []
    for folder in args.folders:
        name = os.path.basename(folder.rstrip("/"))
        r = run_c123(folder, args.every)          # C1 / C2 / C3, same split

        sess = json.load(open(os.path.join(folder, ".fastcalib_intrinsics.json")))
        b = sess["board"]
        spec = BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                         square_mm=b["square_mm"], detector="auto", extractor="legacy")
        objp = spec.object_points().astype(np.float64)
        entries, size = detect_all(folder, spec)
        entries.sort(key=lambda e: e["name"])
        held = set(r["eval_names"])
        eval_sets = [e["corners"] for e in entries if e["name"] in held]
        keep = {e["name"] for e in entries if e["name"] not in held}

        cache = {"image_size": list(size), "board": b, "entries": entries}
        outdir = os.path.join(HERE, "out", "mrcal_holdout", name)
        for key in MB.CONFIGS:
            res = MB.run_one(cache, keep, key, outdir)
            if "error" in res:
                r[key] = {"eval_mean": float("nan"), "error": res["error"]}
                continue
            m, mx = pose_only(res["K"], res["dist"], objp, eval_sets)
            res.update(eval_mean=m, eval_max=mx)
            r[key] = res
        allrows.append(r)

        print(f"\n=== {name}  ({r['n_train']} train / {r['n_eval']} held out) ===")
        order = [("C1", "C1 ROS as shipped        "),
                 ("C2", "C2 M2 no selection       "),
                 ("C3", "C3 RAP                   ")] + \
                [(k, MRCAL_LABELS[k]) for k in MB.CONFIGS]
        for key, label in order:
            c = r.get(key, {})
            if "eval_mean" not in c or c["eval_mean"] != c["eval_mean"]:
                print(f"  {label} FAILED")
                continue
            K = np.asarray(c["K"], float)
            d = (list(np.asarray(c["dist"], float).ravel()) + [0.0] * 5)[:5]
            extra = ""
            if "outlier_corners" in c:
                extra = (f" | {c['outlier_corners']}/{c['total_corners']} corners cut"
                         f" ({100*c['outlier_corners']/c['total_corners']:.1f}%)")
            elif key == "C3":
                t = c["trace"]
                extra = (f" | {len(t['stage1_dropped'])+len(t['stage2_dropped'])}"
                         f" images cut, {t['order']['chosen']}")
            print(f"  {label} fx {K[0,0]:8.2f} cx {K[0,2]:7.2f} k1 {d[0]:+.4f} "
                  f"k2 {d[1]:+.4f} k3 {d[4]:+.5f} | held-out {c['eval_mean']:.3f} px"
                  f" (max {c['eval_max']:5.2f}){extra}")

    if args.json_out:
        def clean(o):
            if isinstance(o, np.ndarray):
                return o.tolist()
            raise TypeError
        json.dump(allrows, open(args.json_out, "w"), indent=1, default=clean)


if __name__ == "__main__":
    main()
