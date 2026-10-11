#!/usr/bin/env python3
"""Rotating holdout: the single split of protocol.py is too thin to trust on a
20-frame session, so every offset of the same stride is run in turn and the
held-out error averaged.  Also reports the corner-zone coverage audit, because
the paper's own reason to distrust a free k3 is extrapolation into image
corners that carry no observations.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(HERE, "..", "..", "..", "tool")))
from fast_calib_offline_gui.camera_intrinsics import BoardSpec  # noqa: E402

from protocol import ROS_FLAGS, pose_only                        # noqa: E402
from rap_headless import detect_all, run_rap                     # noqa: E402


def coverage(entries, size):
    """Corner-zone audit: how many detected points fall in each 20% corner."""
    w, h = size
    pts = np.concatenate([e["corners"] for e in entries])
    zx, zy = 0.2 * w, 0.2 * h
    return {
        "TL": int(((pts[:, 0] < zx) & (pts[:, 1] < zy)).sum()),
        "TR": int(((pts[:, 0] > w - zx) & (pts[:, 1] < zy)).sum()),
        "BL": int(((pts[:, 0] < zx) & (pts[:, 1] > h - zy)).sum()),
        "BR": int(((pts[:, 0] > w - zx) & (pts[:, 1] > h - zy)).sum()),
        "max_radius_frac": float(np.max(np.linalg.norm(
            pts - np.array([w / 2, h / 2]), axis=1)) / (0.5 * np.hypot(w, h))),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--every", type=int, default=4)
    args = ap.parse_args()

    for folder in args.folders:
        sess = json.load(open(os.path.join(folder, ".fastcalib_intrinsics.json")))
        b = sess["board"]
        spec = BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                         square_mm=b["square_mm"], detector="auto", extractor="legacy")
        objp = spec.object_points().astype(np.float64)
        entries, size = detect_all(folder, spec)
        entries.sort(key=lambda e: e["name"])

        cov = coverage(entries, size)
        print(f"\n=== {os.path.basename(folder.rstrip('/'))} "
              f"({len(entries)} frames, {size[0]}x{size[1]}) ===")
        print(f"  coverage  TL {cov['TL']:4d}  TR {cov['TR']:4d}  BL {cov['BL']:4d}  "
              f"BR {cov['BR']:4d} points in the 20% corner zones; "
              f"furthest point at {100*cov['max_radius_frac']:.0f}% of the half-diagonal")

        c2s, c3s, choices, drops = [], [], [], []
        for off in range(args.every):
            ev = entries[off::args.every]
            tr = [e for e in entries if e not in ev]
            if len(tr) < 6 or not ev:
                continue
            sets = [e["corners"] for e in tr]
            _rms, K, D, _r, _t = cv2.calibrateCamera(
                [spec.object_points()] * len(sets), sets, size, None, None,
                flags=ROS_FLAGS)
            m2, _ = pose_only(K, D, objp, [e["corners"] for e in ev])
            res = run_rap(tr, spec, size)
            m3, _ = pose_only(res["K"], res["dist"], objp, [e["corners"] for e in ev])
            c2s.append(m2); c3s.append(m3)
            choices.append(res["trace"]["order"]["chosen"])
            drops.append(len(res["trace"]["stage1_dropped"]) +
                         len(res["trace"]["stage2_dropped"]))
            print(f"  split {off}: train {len(tr):2d} / eval {len(ev):2d} | "
                  f"C2 {m2:.3f} px  C3 {m3:.3f} px  ({100*(m2-m3)/m2:+.1f}%) | "
                  f"{res['trace']['order']['chosen']}, -{drops[-1]} frames")
        c2s, c3s = np.array(c2s), np.array(c3s)
        print(f"  MEAN over {len(c2s)} splits: C2 {c2s.mean():.3f} px -> "
              f"C3 {c3s.mean():.3f} px  ({100*(c2s.mean()-c3s.mean())/c2s.mean():+.1f}%)"
              f" | wins {int((c3s < c2s).sum())}/{len(c2s)}")
        print(f"  order chosen: {', '.join(sorted(set(choices)))}")


if __name__ == "__main__":
    main()
