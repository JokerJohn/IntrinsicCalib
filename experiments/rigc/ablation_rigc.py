#!/usr/bin/env python3
"""Ablation rows of Table II: cross-session held-out error on Rig-C.

Rows (the labels of Table II):
  ours (Full)   Stage 1 + Stage 2 + order selection (flir_full.py, row RAP)
  ours w/o IS   image selection removed: every frame, refined corners, no
                Stage 1 and no Stage 2; order selection kept
  ours w/o OS   order selection removed: Stage 1 + Stage 2, order fixed at M2
  ours-M2       no selection, M2 (flir_full.py, row M2-R)
  Kalibr        Kalibr's published calibration of the session (row kalibr)

Every row is scored on the raw Kalibr corners of the other session with the
board pose refitted per view.  The view mask of Table II (a view is a gross
error when Kalibr, ours-M2 or ours (Full) exceeds three times its own median
on it) is applied to every row.

Reads flir_full_<cam>.json (written by flir_full.py, which must be run first
for cam0 and cam1) and the corner caches corners_<session>_<cam>.json.
Writes ablation_rigc.json beside this script.
"""
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flir_full as F  # noqa: E402

cv2.setNumThreads(1)

ROWS = ["Kalibr", "ours w/o IS", "ours w/o OS", "ours-M2", "ours (Full)"]


def _result(cam):
    p = os.path.join(F.OUT, f"flir_full_{cam}.json")
    if not os.path.exists(p):
        sys.exit(f"{p} not found: run 'python flir_full.py {cam}' first")
    return json.load(open(p))


def without_is(uvs, objs):
    """Order selection on every frame, without Stage 1 and Stage 2."""
    old = F.TMAX
    F.TMAX = 0
    try:
        K, D, info = F.rap(uvs, objs)
    finally:
        F.TMAX = old
    return K, D, dict(order=info["order"], n=info["n"])


def without_os(uvs, objs):
    """Stage 1 + Stage 2 with the order fixed at M2."""
    K, D, kept = F.stage2(uvs, objs, True)
    return K, D, dict(order="M2", kept=kept, n=len(uvs))


def main():
    out = {}
    for cam in ("cam0", "cam1"):
        R = _result(cam)
        obs = {d: F.extract(d, cam) for d in ("D1", "D2")}
        out[cam] = {}
        for train, test in (("D1", "D2"), ("D2", "D1")):
            key = f"{train}->{test}"
            tr = obs[train]
            acc = [o for o in tr if o["src"] == "rap"]          # accepted by Stage 1
            uvA, objA = F.sets(acc, "R")
            uvR, objR = F.sets(tr, "R")
            row = lambda m: (np.array(R[key]["rows"][m]["K"]), np.array(R[key]["rows"][m]["D"]), {})
            P = {"Kalibr": row("kalibr"), "ours-M2": row("M2-R"), "ours (Full)": row("RAP"),
                 "ours w/o IS": without_is(uvR, objR), "ours w/o OS": without_os(uvA, objA)}
            uvT, objT = F.sets(obs[test], "K")
            errs = {m: F.view_errors(v[0], v[1], uvT, objT) for m, v in P.items()}
            pv = np.array([[e.mean() for e in errs[m]] for m in ("Kalibr", "ours-M2", "ours (Full)")])
            gross = (pv > 3 * np.median(pv, axis=1, keepdims=True)).any(0)
            out[cam][key] = {m: dict(clean=float(np.concatenate([e for e, g in zip(errs[m], gross) if not g]).mean()),
                                     **{a: b for a, b in P[m][2].items() if a in ("order", "kept", "n")})
                             for m in ROWS}
    json.dump(out, open(os.path.join(F.OUT, "ablation_rigc.json"), "w"), indent=1)

    cols = [(c, k) for c in ("cam0", "cam1") for k in ("D1->D2", "D2->D1")]
    print("Table II, cross-session held-out mean corner error [px] on Rig-C")
    print(f"{'method':14s}" + "".join(f"{('L' if c == 'cam0' else 'R') + ' ' + k:>12s}" for c, k in cols))
    print(f"{'ROS-calib':14s}" + "".join(f"{'x':>12s}" for _ in cols))
    print(f"{'mrcal':14s}" + "".join(f"{'x':>12s}" for _ in cols))
    for m in ROWS:
        print(f"{m:14s}" + "".join(f"{out[c][k][m]['clean']:12.3f}" for c, k in cols))


if __name__ == "__main__":
    main()
