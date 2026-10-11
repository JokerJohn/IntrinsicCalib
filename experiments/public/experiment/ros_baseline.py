#!/usr/bin/env python3
"""ROS camera_calibration baseline — the real MonoCalibrator from
/opt/ros/noetic, not a reimplementation.

Must run under /usr/bin/python3 (the interpreter ROS Noetic's dist-packages
were built for; it carries OpenCV 4.2.0, which is what a ROS user actually
gets).  The flags reproduce `rosrun camera_calibration cameracalibrator.py`
with its defaults: -k 2, i.e. k1 k2 p1 p2 free and k3..k6 fixed at zero —
the same model the paper calls M2.

    /usr/bin/python3 ros_baseline.py <folder> --out ros.json
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import cv2                                                    # noqa: E402
import numpy as np                                            # noqa: E402
from camera_calibration.calibrator import (                    # noqa: E402
    ChessboardInfo, MonoCalibrator, Patterns,
)

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")

# cameracalibrator.py, default --k-coefficients=2
ROS_DEFAULT_FLAGS = (cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 |
                     cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--out", required=True)
    ap.add_argument("--exclude", default="")
    args = ap.parse_args()

    sess = json.load(open(os.path.join(args.folder, ".intrinsics_session.json")))
    b = sess["board"]
    held = set(x for x in args.exclude.split(",") if x)

    paths = sorted(p for p in glob.glob(os.path.join(args.folder, "*"))
                   if p.lower().endswith(IMG_EXT)
                   and os.path.basename(p) not in held)
    images = [cv2.imread(p) for p in paths]
    images = [im for im in images if im is not None]

    board = ChessboardInfo(n_cols=b["cols"], n_rows=b["rows"],
                           dim=b["square_mm"] / 1000.0)
    mc = MonoCalibrator([board], flags=ROS_DEFAULT_FLAGS,
                        pattern=Patterns.Chessboard, name="cam")

    # collect_corners is where ROS decides which frames it can use at all
    good = mc.collect_corners(images)
    mc.cal_fromcorners(good)

    K = np.asarray(mc.intrinsics, float)
    D = np.asarray(mc.distortion, float).ravel()[:5]

    # in-sample mean corner error, in the MATLAB convention the paper reports
    ipts = [g[0].reshape(-1, 2).astype(np.float64) for g in good]
    opts = [o.reshape(-1, 3).astype(np.float64) for o in mc.mk_object_points(
        [g[2] for g in good])]
    errs = []
    for o, uv in zip(opts, ipts):
        ok, rv, tv = cv2.solvePnP(o, uv, K, D)
        proj, _ = cv2.projectPoints(o, rv, tv, K, D)
        errs.append(np.linalg.norm(proj.reshape(-1, 2) - uv, axis=1))
    allerr = np.concatenate(errs)

    out = {
        "K": K.tolist(), "dist": D.tolist(),
        "image_size": [int(mc.size[0]), int(mc.size[1])],
        "n_input": len(images), "n_used": len(good),
        "mean_corner": float(allerr.mean()),
        "opencv_version": cv2.__version__,
        "flags": "k1 k2 p1 p2 free, k3..k6 fixed (-k 2, ROS default)",
    }
    json.dump(out, open(args.out, "w"), indent=1)
    print(f"ROS  {args.folder}: {len(good)}/{len(images)} used | "
          f"in-sample mean corner {out['mean_corner']:.3f} px")


if __name__ == "__main__":
    main()
