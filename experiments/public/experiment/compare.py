#!/usr/bin/env python3
"""Score two parameter sets on identical observations.

In-sample errors are not comparable across methods: each method reports the
residual of the frames it chose to keep, and RAP keeps fewer and better ones,
so a lower number there is partly self-selection.  This is the paper's own
remedy (eq. valpose): freeze the intrinsics, refit only the six pose
parameters per frame with solvePnP, and evaluate on one common corner set.
Whatever difference survives is the intrinsics, not the bookkeeping.
"""
from __future__ import annotations

import argparse
import json

import cv2
import numpy as np


def load_cache(path):
    c = json.load(open(path))
    b = c["board"]
    objp = np.zeros((b["rows"] * b["cols"], 3), np.float64)
    objp[:, :2] = np.mgrid[0:b["cols"], 0:b["rows"]].T.reshape(-1, 2)
    objp *= b["square_mm"] / 1000.0
    entries = [(e["name"], e["source"], np.asarray(e["corners"], np.float64))
               for e in c["entries"]]
    return objp, entries, tuple(c["image_size"])


def pose_only(params, objp, entries):
    """Mean corner error with K, dist frozen and only the pose refitted."""
    K = np.asarray(params["K"], float)
    D = np.asarray((list(params["dist"]) + [0.0] * 5)[:5], float)
    per = []
    for name, _src, uv in entries:
        ok, rv, tv = cv2.solvePnP(objp, uv, K, D)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
        per.append((name, np.linalg.norm(proj.reshape(-1, 2) - uv, axis=1)))
    allerr = np.concatenate([e for _n, e in per])
    return {"mean": float(allerr.mean()),
            "max": float(allerr.max()),
            "per_image": {n: float(e.mean()) for n, e in per}}


def fmt_params(p):
    K = np.asarray(p["K"], float)
    d = (list(p["dist"]) + [0.0] * 5)[:5]
    return (f"fx {K[0,0]:8.2f}  fy {K[1,1]:8.2f}  cx {K[0,2]:8.2f}  cy {K[1,2]:8.2f}  "
            f"k1 {d[0]:+.4f}  k2 {d[1]:+.4f}  k3 {d[4]:+.5f}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", required=True)
    ap.add_argument("--ros", required=True)
    ap.add_argument("--rap", required=True)
    ap.add_argument("--eval-names", default="", help="restrict scoring to these frames")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    objp, entries, size = load_cache(args.cache)
    if args.eval_names:
        want = set(args.eval_names.split(","))
        entries = [e for e in entries if e[0] in want]

    ros = json.load(open(args.ros))
    rap = json.load(open(args.rap))

    r_ros = pose_only(ros, objp, entries)
    r_rap = pose_only(rap, objp, entries)

    print(f"\n=== {args.label or args.cache} ===")
    print(f"  scoring set: {len(entries)} frames, {size[0]}x{size[1]}")
    print(f"  ROS  {fmt_params(ros)}")
    print(f"  RAP  {fmt_params(rap)}")
    dK = np.asarray(rap["K"], float) - np.asarray(ros["K"], float)
    print(f"  dK   fx {dK[0,0]:+.2f}  fy {dK[1,1]:+.2f}  cx {dK[0,2]:+.2f}  "
          f"cy {dK[1,2]:+.2f} px")
    print(f"  in-sample  ROS {ros['mean_corner']:.3f} px on {ros['n_used']} frames | "
          f"RAP {rap['mean_corner']:.3f} px on {rap['trace']['n_final']} frames  "
          f"(not comparable)")
    print(f"  pose-only  ROS {r_ros['mean']:.3f} px (max {r_ros['max']:.2f}) | "
          f"RAP {r_rap['mean']:.3f} px (max {r_rap['max']:.2f})  "
          f"-> {100*(r_ros['mean']-r_rap['mean'])/r_ros['mean']:+.1f}%")
    t = rap["trace"]
    print(f"  RAP trace: {t['n_input']} detected, stage1 dropped "
          f"{len(t['stage1_dropped'])}, stage2 dropped {len(t['stage2_dropped'])}, "
          f"order = {t['order']['chosen']}")
    if "M2_val_mean" in t["order"]:
        o = t["order"]
        print(f"             validation: M2 {o['M2_val_mean']:.3f}/{o['M2_val_max']:.3f} px "
              f"vs M3 {o['M3_val_mean']:.3f}/{o['M3_val_max']:.3f} px (mean/max)")


if __name__ == "__main__":
    main()
