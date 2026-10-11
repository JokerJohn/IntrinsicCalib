#!/usr/bin/env python3
"""Sanity-check every staged dataset with a plain OpenCV calibration, so the
board geometry recorded in the session file is known to be right before any of
it is fed to the RAP selection.

For each folder it reports: how many images give a full inner-corner grid, the
reference OpenCV fit (fix-k3, i.e. the M2 baseline), the mean corner error in
the MATLAB convention the paper uses, and how many images sit above 2x the
median — the population Stage 2 is supposed to act on.

    ./verify_datasets.py               # every folder with a session file
    ./verify_datasets.py opencv_left   # just one
"""
from __future__ import annotations

import glob
import json
import os
import sys

import cv2
import numpy as np

SESSION_FILE = ".intrinsics_session.json"
IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")


def detect(gray, size):
    ok, c = cv2.findChessboardCornersSB(
        gray, size, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if ok:
        return c, "SB"
    ok, c = cv2.findChessboardCorners(
        gray, size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        cv2.cornerSubPix(gray, c, (11, 11), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4))
        return c, "classic"
    return None, None


def check(folder: str) -> dict | None:
    sess_path = os.path.join(folder, SESSION_FILE)
    if not os.path.exists(sess_path):
        print(f"{folder}: no {SESSION_FILE}, skipped")
        return None
    board = json.load(open(sess_path))["board"]
    size = (board["cols"], board["rows"])
    sq = board["square_mm"]

    paths = sorted(p for p in glob.glob(os.path.join(folder, "*"))
                   if p.lower().endswith(IMG_EXT))
    g0 = cv2.imread(paths[0], cv2.IMREAD_GRAYSCALE)
    h, w = g0.shape
    print(f"\n{folder}: {len(paths)} images {w}x{h}, board {size[0]}x{size[1]} @ {sq} mm")

    objp = np.zeros((size[0] * size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2) * sq
    obj, img, how = [], [], []
    for p in paths:
        g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        c, tag = detect(g, size)
        if c is not None:
            obj.append(objp); img.append(c); how.append(tag)
    print(f"  detected {len(obj)}/{len(paths)} "
          f"(SB {how.count('SB')}, classic {how.count('classic')})")
    if len(obj) < 6:
        print("  !! too few detections to calibrate")
        return None

    rms, K, d, rv, tv = cv2.calibrateCamera(obj, img, (w, h), None, None,
                                            flags=cv2.CALIB_FIX_K3)
    per = []
    for i in range(len(obj)):
        pr, _ = cv2.projectPoints(obj[i], rv[i], tv[i], K, d)
        per.append(np.linalg.norm(img[i].reshape(-1, 2) - pr.reshape(-1, 2),
                                  axis=1).mean())
    per = np.array(per)
    med = np.median(per)
    print(f"  M2 (fix k3): RMS {rms:.3f} px | mean corner {per.mean():.3f} px")
    print(f"  fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f} "
          f"| k1 {d.ravel()[0]:+.4f} k2 {d.ravel()[1]:+.4f}")
    print(f"  per-image mean: min {per.min():.3f} median {med:.3f} max {per.max():.3f} "
          f"| above 2x median: {(per > 2*med).sum()}")
    return {"folder": folder, "n": len(paths), "detected": len(obj),
            "rms": float(rms), "mean_corner": float(per.mean())}


def main() -> None:
    folders = sys.argv[1:] or sorted(
        os.path.dirname(p) for p in glob.glob(f"**/{SESSION_FILE}", recursive=True))
    for f in folders:
        try:
            check(f)
        except Exception as exc:                      # keep going across a bad set
            print(f"{f}: FAILED — {exc}")


if __name__ == "__main__":
    main()
