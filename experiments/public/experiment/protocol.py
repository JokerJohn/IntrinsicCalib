#!/usr/bin/env python3
"""Controlled comparison, with an evaluation set neither method ever sees.

Three configurations, so that the corner extractor and the paper's two
contributions are not confounded:

  C1  ROS as shipped     ROS detector (classic + its 0.5x-min-spacing subpix)
                         + ROS default flags (-k 2 => M2), every frame it accepts
  C2  M2, no selection   the project's detector (SB, classic fallback)
                         + the same M2 flags, every detected frame
  C3  RAP                the project's detector + Stage 1 + Stage 2
                         + validation-based distortion-order selection

  C1 vs C2  isolates the corner extractor.
  C2 vs C3  isolates image selection + distortion-order selection — the paper's
            actual contributions, with the extractor held fixed.

Scoring: intrinsics frozen, six pose parameters refitted per frame (solvePnP)
on the held-out frames' SB corners.  Held-out frames are removed before any
method runs, so nothing is scored on its own training data.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(HERE, "..", "..", "..", "tool")))
from fast_calib_offline_gui.camera_intrinsics import BoardSpec, calibrate  # noqa: E402

from rap_headless import detect_all, run_rap                               # noqa: E402

ROS_FLAGS = (cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 |
             cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6)


def pose_only(K, D, objp, sets):
    K = np.asarray(K, float)
    D = np.asarray(list(np.asarray(D, float).ravel()) + [0.0] * 5, float)[:5]
    errs = []
    for uv in sets:
        uv = np.asarray(uv, np.float64)
        ok, rv, tv = cv2.solvePnP(objp, uv, K, D)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
        errs.append(np.linalg.norm(proj.reshape(-1, 2) - uv, axis=1))
    a = np.concatenate(errs)
    return float(a.mean()), float(a.max())


def run(folder: str, every: int) -> dict:
    sess = json.load(open(os.path.join(folder, ".fastcalib_intrinsics.json")))
    b = sess["board"]
    spec = BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                     square_mm=b["square_mm"], detector="auto", extractor="legacy")
    objp = spec.object_points().astype(np.float64)

    entries, size = detect_all(folder, spec)
    entries.sort(key=lambda e: e["name"])
    eval_entries = entries[::every]
    train_entries = [e for e in entries if e not in eval_entries]
    eval_sets = [e["corners"] for e in eval_entries]
    held = ",".join(e["name"] for e in eval_entries)

    name = os.path.basename(folder.rstrip("/"))
    out = {"dataset": name, "n_total": len(entries),
           "n_train": len(train_entries), "n_eval": len(eval_entries),
           "eval_names": [e["name"] for e in eval_entries]}

    # -- C1: the real ROS calibrator, in its own interpreter --------------
    tmp = os.path.join(HERE, "out", f"{name}_ros_holdout.json")
    subprocess.run(["/usr/bin/python3", os.path.join(HERE, "ros_baseline.py"),
                    folder, "--out", tmp, "--exclude", held],
                   check=True, capture_output=True)
    c1 = json.load(open(tmp))
    out["C1"] = {"K": c1["K"], "dist": c1["dist"], "n_used": c1["n_used"]}

    # -- C2: same flags, our corners, no selection ------------------------
    sets = [e["corners"] for e in train_entries]
    rms, K, D, _rv, _tv = cv2.calibrateCamera(
        [spec.object_points()] * len(sets), sets, size, None, None, flags=ROS_FLAGS)
    out["C2"] = {"K": np.asarray(K).tolist(), "dist": np.asarray(D).ravel().tolist(),
                 "n_used": len(sets)}

    # -- C3: RAP ----------------------------------------------------------
    res = run_rap(train_entries, spec, size)
    out["C3"] = {"K": res["K"], "dist": res["dist"],
                 "n_used": len(res["used_names"]), "trace": res["trace"]}

    for key in ("C1", "C2", "C3"):
        m, mx = pose_only(out[key]["K"], out[key]["dist"], objp, eval_sets)
        out[key]["eval_mean"] = m
        out[key]["eval_max"] = mx
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--every", type=int, default=4,
                    help="hold out every Nth frame (default 4 => 25%%)")
    ap.add_argument("--json-out")
    args = ap.parse_args()

    rows = []
    for f in args.folders:
        r = run(f, args.every)
        rows.append(r)
        t = r["C3"]["trace"]
        print(f"\n=== {r['dataset']}  ({r['n_train']} train / {r['n_eval']} held out) ===")
        for key, label in (("C1", "C1 ROS as shipped   "),
                           ("C2", "C2 M2, no selection "),
                           ("C3", "C3 RAP              ")):
            c = r[key]
            K = np.asarray(c["K"], float)
            d = (list(c["dist"]) + [0.0] * 5)[:5]
            print(f"  {label} fx {K[0,0]:8.2f} cx {K[0,2]:7.2f} "
                  f"k1 {d[0]:+.4f} k2 {d[1]:+.4f} k3 {d[4]:+.5f} | "
                  f"{c['n_used']:2d} frames | held-out {c['eval_mean']:.3f} px "
                  f"(max {c['eval_max']:.2f})")
        print(f"     extractor effect  C1->C2 {100*(r['C1']['eval_mean']-r['C2']['eval_mean'])/r['C1']['eval_mean']:+.1f}%")
        print(f"     selection effect  C2->C3 {100*(r['C2']['eval_mean']-r['C3']['eval_mean'])/r['C2']['eval_mean']:+.1f}%"
              f"   [stage1 -{len(t['stage1_dropped'])}, stage2 -{len(t['stage2_dropped'])}, {t['order']['chosen']}]")

    if args.json_out:
        json.dump(rows, open(args.json_out, "w"), indent=1)


if __name__ == "__main__":
    main()
