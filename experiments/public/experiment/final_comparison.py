#!/usr/bin/env python3
"""The headline table: every method at its shipped defaults, one error number.

No baseline is modified and no baseline internals are ablated -- each tool runs
as its authors ship it, and only its final calibration is compared.  The single
ablation kept is of *this* method (M2 with no selection), because that is what
isolates the paper's own contribution.

Two error columns, and the distinction matters:

  self-reported  what each tool prints at the end of its own run, on the frames
                 and corners it chose to keep.  Quoted for transparency; NOT
                 comparable across tools, because the retained sets differ.
  held-out       intrinsics frozen, six pose parameters refitted per frame, on
                 frames no method ever saw.  This is the comparable number.

Averaged over a rotating 4-fold holdout: a single split of a 12-frame session
reverses sign between folds and is not evidence in either direction.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(HERE, "..", "..", "..", "tool")))
from fast_calib_offline_gui.camera_intrinsics import BoardSpec  # noqa: E402

import mrcal_baseline as MB                                     # noqa: E402
from protocol import ROS_FLAGS, pose_only                       # noqa: E402
from rap_headless import detect_all, run_rap                    # noqa: E402

# Table I rows: ROS-calib, mrcal (OPENCV8, its documented rational model), ours-M2, ours (Full);
# the 4- and 5-coefficient mrcal variants enter the order comparison of Section V.
METHODS = ["ROS-calib", "mrcal OPENCV4", "mrcal OPENCV5",
           "mrcal OPENCV8", "ours-M2", "ours (Full)"]


def run_ros(folder, held):
    tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False).name
    p = subprocess.run(["/usr/bin/python3", os.path.join(HERE, "ros_baseline.py"),
                        folder, "--out", tmp, "--exclude", ",".join(sorted(held))],
                       capture_output=True, text=True)
    if not os.path.exists(tmp) or os.path.getsize(tmp) == 0:
        return None
    r = json.load(open(tmp))
    os.unlink(tmp)
    return {"K": r["K"], "dist": r["dist"], "self": r["mean_corner"],
            "note": f"{r['n_used']} frames"}


def fold(folder, entries, size, spec, objp, off, every, outdir):
    ev = entries[off::every]
    tr = [e for e in entries if e not in ev]
    held = {e["name"] for e in ev}
    keep = {e["name"] for e in tr}
    eval_sets = [e["corners"] for e in ev]
    cache = {"image_size": list(size), "board": {
        "pattern": spec.pattern, "cols": spec.cols, "rows": spec.rows,
        "square_mm": spec.square_mm}, "entries": entries}

    res = {}

    r = run_ros(folder, held)
    if r:
        res["ROS-calib"] = r

    for key in MB.CONFIGS:
        m = MB.run_one(cache, keep, key, os.path.join(outdir, f"f{off}"))
        if "error" in m:
            continue
        label = "mrcal " + key.split("-")[1]
        note = ""
        if "outlier_corners" in m:
            note = (f"{100*m['outlier_corners']/m['total_corners']:.1f}% corners cut")
        res[label] = {"K": m["K"], "dist": m["dist"], "self": m["rms"], "note": note}

    sets = [e["corners"] for e in tr]
    _rms, K, D, _r, _t = cv2.calibrateCamera(
        [spec.object_points()] * len(sets), sets, size, None, None, flags=ROS_FLAGS)
    ins = np.mean([np.linalg.norm(
        cv2.projectPoints(spec.object_points(), _r[i], _t[i], K, D)[0].reshape(-1, 2)
        - sets[i], axis=1).mean() for i in range(len(sets))])
    res["ours-M2"] = {
        "K": np.asarray(K).tolist(), "dist": np.asarray(D).ravel().tolist(),
        "self": float(ins), "note": f"{len(sets)} frames"}

    rr = run_rap(tr, spec, size)
    t = rr["trace"]
    res["ours (Full)"] = {
        "K": rr["K"], "dist": rr["dist"],
        "self": float(np.mean([p["mean"] for p in rr["per_image"]])),
        "note": (f"{len(t['stage1_dropped'])+len(t['stage2_dropped'])} images cut, "
                 f"{t['order']['chosen']}")}

    for k in res:
        m, mx = pose_only(res[k]["K"], res[k]["dist"], objp, eval_sets)
        res[k]["held"] = m
        res[k]["held_max"] = mx
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--every", type=int, default=4)
    ap.add_argument("--json-out")
    args = ap.parse_args()

    table = {}
    for folder in args.folders:
        name = os.path.basename(folder.rstrip("/"))
        sess = json.load(open(os.path.join(folder, ".fastcalib_intrinsics.json")))
        b = sess["board"]
        spec = BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                         square_mm=b["square_mm"], detector="auto", extractor="legacy")
        objp = spec.object_points().astype(np.float64)
        entries, size = detect_all(folder, spec)
        entries.sort(key=lambda e: e["name"])
        outdir = os.path.join(HERE, "out", "final", name)

        acc = {}
        for off in range(args.every):
            r = fold(folder, entries, size, spec, objp, off, args.every, outdir)
            for k, v in r.items():
                acc.setdefault(k, {"held": [], "self": [], "note": []})
                acc[k]["held"].append(v["held"])
                acc[k]["self"].append(v["self"])
                acc[k]["note"].append(v["note"])
        table[name] = acc

        print(f"\n=== {name}  ({len(entries)} frames, {args.every}-fold holdout) ===")
        print(f"  {'method':16s} {'self-reported':>14s} {'held-out':>10s} {'cell':>10s}  {'vs M2':>6s}  notes")
        base = np.mean(acc["ours-M2"]["held"])
        for k in METHODS:
            if k not in acc:
                continue
            h = np.mean(acc[k]["held"])
            s = np.mean([x for x in acc[k]["self"] if x is not None])
            d = 100 * (base - h) / base
            note = max(set(acc[k]["note"]), key=acc[k]["note"].count)
            cell = f"{h:.3f}({round(1000 * np.std(acc[k]['held'])):d})"   # mean(std over folds, ddof 0, last digit)
            print(f"  {k:16s} {s:11.3f} px {h:8.3f} px {cell:>10s}  {d:+5.1f}%  {note}")

    if args.json_out:
        json.dump(table, open(args.json_out, "w"), indent=1)


if __name__ == "__main__":
    main()
