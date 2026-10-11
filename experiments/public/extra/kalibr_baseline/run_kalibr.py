#!/usr/bin/env python3
"""Kalibr baseline for Table I (and Rig-A), at Kalibr's shipped defaults.

For every (session, fold) the TRAINING frames of the rotating 4-fold holdout of
final_comparison.py (entries sorted by name, eval = entries[f::4]) are written
to a ROS1 bag, and the real kalibr_calibrate_cameras (docker image
kalibr:noetic, built from Dockerfile_ros1_20_04 of Kalibr's master branch,
see README.md of this directory)
is run on it with pinhole-radtan.  The resulting camchain is scored with
protocol.pose_only on the evaluation fold's legacy-extractor corners.

Stage 'run' launches the containers; stage 'score' parses and scores.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import glob
import json
import os
import re
import subprocess
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, "..", "..", "experiment"))   # experiments/public/experiment
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(EXP, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(EXP, "..", "..", "..", "tool")))
import cv2                                                      # noqa: E402
from fast_calib_offline_gui.camera_intrinsics import BoardSpec  # noqa: E402
from protocol import pose_only                                  # noqa: E402
from rap_headless import detect_all                             # noqa: E402

DATA = os.environ.get("PUBLIC_DIR", os.path.dirname(EXP))   # fetched public sessions (experiments/public)
SESSIONS = ["opencalib_front", "ros_wide_left", "ros_wide_right",
            "opencv_left", "opencv_right"]
RIGA = os.environ.get("RIG_A_DIR", os.path.join(EXP, "..", "..", "..", "data", "rig_a"))
RIGA_VAL = ["left-0000.png", "left-0006.png", "left-0011.png",
            "left-0019.png", "left-0024.png", "left-0035.png"]
IMAGE = os.environ.get("KALIBR_IMAGE", "kalibr:noetic")
EVERY = 4


DATA, RIGA = os.path.abspath(DATA), os.path.abspath(RIGA)


def spec_of(folder):
    if folder == RIGA:
        return BoardSpec(pattern="chessboard", cols=11, rows=8, square_mm=45.0,
                         detector="auto", extractor="legacy")
    b = json.load(open(os.path.join(folder, ".fastcalib_intrinsics.json")))["board"]
    return BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                     square_mm=b["square_mm"], detector="auto", extractor="legacy")


def jobs():
    """(tag, folder, train_names, eval_entries, spec) for every run."""
    out = []
    for s in SESSIONS:
        folder = os.path.join(DATA, s)
        spec = spec_of(folder)
        entries, _ = detect_all(folder, spec)
        entries.sort(key=lambda e: e["name"])
        for off in range(EVERY):
            ev = entries[off::EVERY]
            tr = [e for e in entries if e not in ev]
            out.append((f"{s}/f{off}", folder, [e["name"] for e in tr], ev, spec))
    spec = spec_of(RIGA)
    entries, _ = detect_all(RIGA, spec)
    entries.sort(key=lambda e: e["name"])
    ev = [e for e in entries if e["name"] in RIGA_VAL]
    assert len(ev) == 6
    tr = sorted(os.path.basename(p) for p in glob.glob(os.path.join(RIGA, "*.png"))
                if os.path.basename(p) not in RIGA_VAL)
    out.append(("rig_a/train33", RIGA, tr, ev, spec))
    return out


def run_one(tag, folder, train, spec):
    wd = os.path.join(HERE, os.environ.get("KALIBR_RUNS", "runs"), tag)
    os.makedirs(wd, exist_ok=True)
    sq = spec.square_mm / 1000.0
    with open(os.path.join(wd, "checkerboard.yaml"), "w") as f:
        f.write(f"target_type: 'checkerboard'\ntargetCols: {spec.cols}\n"
                f"targetRows: {spec.rows}\nrowSpacingMeters: {sq}\n"
                f"colSpacingMeters: {sq}\n")
    with open(os.path.join(wd, "train_images.txt"), "w") as f:
        f.write("\n".join(train) + "\n")
    imgs = " ".join(f"'{os.path.join(folder, n)}'" for n in train)
    cmd = (f"source /catkin_ws/devel/setup.bash && cd '{wd}' && "
           f"rm -f train.bag train-* && "
           f"python3 '{HERE}/make_bag.py' train.bag {imgs} && "
           f"rosrun kalibr kalibr_calibrate_cameras --models pinhole-radtan "
           f"--target checkerboard.yaml --bag train.bag --topics /cam0/image_raw "
           f"--dont-show-report; rc=$?; rm -f train.bag; exit $rc")
    dcmd = ["docker", "run", "--rm", "--network", "none",
            "-u", f"{os.getuid()}:{os.getgid()}",
            "-e", "HOME=/tmp", "-e", "ROS_HOME=/tmp/.ros", "-e", "MPLBACKEND=Agg",
            *[a for p in sorted({os.path.abspath(HERE), os.path.abspath(DATA), os.path.abspath(RIGA)})
              for a in ("-v", f"{p}:{p}")],
            "--entrypoint", "/bin/bash", IMAGE, "-c", cmd]
    with open(os.path.join(wd, "kalibr.log"), "w") as log:
        try:
            p = subprocess.run(dcmd, stdout=log, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, timeout=3600)
            rc = p.returncode
        except subprocess.TimeoutExpired:
            rc = "timeout"
    return tag, rc


def parse(tag):
    wd = os.path.join(HERE, os.environ.get("KALIBR_RUNS", "runs"), tag)
    log = open(os.path.join(wd, "kalibr.log"), errors="replace").read()
    info = {"log": os.path.relpath(os.path.join(wd, "kalibr.log"), HERE)}
    m = re.search(r"Extracted corners for (\d+) images \(of (\d+) images\)", log)
    if m:
        info["corners_extracted"] = int(m.group(1))
        info["images_in_bag"] = int(m.group(2))
    m = re.search(r"Processed (\d+) images with (\d+) images used", log)
    if m:
        info["views_processed"] = int(m.group(1))
        info["views_used"] = int(m.group(2))
    info["outlier_corners_removed"] = sum(
        int(x) for x in re.findall(r"Removed (\d+) outlier corners", log))
    cc = os.path.join(wd, "train-camchain.yaml")
    if not os.path.exists(cc):
        tail = [l for l in log.strip().splitlines() if l.strip()][-6:]
        info["error"] = "\n".join(tail)
        return info, None, None
    c = yaml.safe_load(open(cc))["cam0"]
    fx, fy, cx, cy = c["intrinsics"]
    k1, k2, p1, p2 = c["distortion_coeffs"]
    K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    D = [k1, k2, p1, p2, 0.0]
    info["camchain"] = os.path.relpath(cc, HERE)
    info["K"], info["dist"] = K, D
    return info, K, D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["run", "score"])
    ap.add_argument("--only", default="")
    ap.add_argument("-j", type=int, default=6)
    a = ap.parse_args()
    J = [j for j in jobs() if a.only in j[0]]

    if a.stage == "run":
        with cf.ThreadPoolExecutor(a.j) as ex:
            fs = [ex.submit(run_one, t, f, tr, sp) for t, f, tr, _ev, sp in J]
            for fu in cf.as_completed(fs):
                print(*fu.result(), flush=True)
        return

    res = {}
    for tag, folder, train, ev, spec in J:
        objp = spec.object_points().astype(np.float64)
        info, K, D = parse(tag)
        info["n_train"] = len(train)
        info["n_eval"] = len(ev)
        if K is not None:
            m, mx = pose_only(K, D, objp, [e["corners"] for e in ev])
            info["held"], info["held_max"] = m, mx
            pv = [pose_only(K, D, objp, [e["corners"]])[0] for e in ev]
            info["held_per_view"] = dict(zip([e["name"] for e in ev], pv))
            info["held_per_view_mean"] = float(np.mean(pv))
        res[tag] = info
        print(tag, {k: v for k, v in info.items()
                    if k not in ("K", "dist", "held_per_view")}, flush=True)
    json.dump(res, open(os.path.join(HERE, os.environ.get("KALIBR_RUNS", "runs") + "_folds_raw.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
