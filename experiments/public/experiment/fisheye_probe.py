#!/usr/bin/env python3
"""Does the RAP machinery transfer to a fisheye?

Run on mrcal's Samyang session, using the corners mrgingham already produced
(corners.vnl) because no OpenCV chessboard detector indexes this grid at all.
That substitution is itself the first finding, and it is what makes the rest
of the experiment possible.

Kannala-Brandt has the same kind of order hierarchy the paper exploits for
Brown-Conrady -- theta + k1 th^3 + k2 th^5 + k3 th^7 + k4 th^9 -- so the
distortion-order criterion is transplanted verbatim onto it:

  F2  k1 k2       (k3, k4 fixed at 0)
  F3  k1 k2 k3    (k4 fixed)
  F4  k1 k2 k3 k4 (OpenCV's default, and what the project's calibrate() does)

Stage 2 (residual-based rejection) is model-agnostic and is applied unchanged.
Stage 1 is NOT applicable: it is defined as "the SB detector succeeded", and
here it succeeds nowhere.
"""
from __future__ import annotations

import argparse
import os
from collections import defaultdict

import cv2
import numpy as np

PRUNE_FACTOR = 2.0
ORDERS = {
    "F2": cv2.fisheye.CALIB_FIX_K3 | cv2.fisheye.CALIB_FIX_K4,
    "F3": cv2.fisheye.CALIB_FIX_K4,
    "F4": 0,
}
BASE_FLAGS = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW


def read_vnl(path, gridn):
    """mrgingham output: `filename x y level`, one corner per line."""
    per = defaultdict(list)
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        f = line.split()
        if len(f) < 3 or f[1] == "-":
            continue
        per[f[0]].append((float(f[1]), float(f[2])))
    want = gridn * gridn
    return {k: np.asarray(v, np.float64) for k, v in per.items() if len(v) == want}


def fisheye_fit(objp, sets, size, flags):
    o = [objp.reshape(-1, 1, 3).copy() for _ in sets]
    c = [s.reshape(-1, 1, 2).copy() for s in sets]
    K = np.eye(3)
    D = np.zeros((4, 1))
    rms, K, D, rv, tv = cv2.fisheye.calibrate(
        o, c, size, K, D, None, None, BASE_FLAGS | flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-8))
    per = []
    for i, s in enumerate(sets):
        pr, _ = cv2.fisheye.projectPoints(o[i], rv[i], tv[i], K, D)
        per.append(float(np.linalg.norm(pr.reshape(-1, 2) - s, axis=1).mean()))
    return {"K": K, "D": D, "rms": float(rms), "per_image": np.asarray(per)}


def fisheye_pose_only(K, D, objp, sets):
    """Freeze K,D; refit the six pose parameters per frame.

    cv2 has no fisheye solvePnP: unproject to normalised rays with
    fisheye.undistortPoints, solve there with an identity camera, then measure
    the error back in pixels through fisheye.projectPoints.
    """
    errs = []
    for uv in sets:
        norm = cv2.fisheye.undistortPoints(uv.reshape(-1, 1, 2), K, D)
        ok, rv, tv = cv2.solvePnP(objp, norm.reshape(-1, 1, 2),
                                  np.eye(3), np.zeros(5))
        if not ok:
            continue
        pr, _ = cv2.fisheye.projectPoints(objp.reshape(-1, 1, 3), rv, tv, K, D)
        errs.append(np.linalg.norm(pr.reshape(-1, 2) - uv, axis=1))
    a = np.concatenate(errs)
    return float(a.mean()), float(a.max())


def stage2(objp, sets, names, size, flags):
    """The project's Stage 2, unchanged, on top of a fisheye fit."""
    r = fisheye_fit(objp, sets, size, flags)
    cs, kept = list(sets), list(names)
    for _ in range(3):
        med = float(np.median(r["per_image"]))
        keep = r["per_image"] <= PRUNE_FACTOR * med
        if keep.all() or int(keep.sum()) < 8:
            break
        cs = [c for c, k in zip(cs, keep) if k]
        kept = [n for n, k in zip(kept, keep) if k]
        r = fisheye_fit(objp, cs, size, flags)
    r["kept"] = kept
    r["sets"] = cs
    return r


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vnl", required=True)
    ap.add_argument("--images", required=True)
    ap.add_argument("--gridn", type=int, default=14)
    ap.add_argument("--spacing-m", type=float, default=0.0588)
    args = ap.parse_args()

    per = read_vnl(args.vnl, args.gridn)
    names = sorted(per)
    sets = [per[n] for n in names]
    g = cv2.imread(os.path.join(args.images, names[0]), cv2.IMREAD_GRAYSCALE)
    size = (g.shape[1], g.shape[0])

    n = args.gridn
    objp = np.zeros((n * n, 3), np.float64)
    objp[:, :2] = np.mgrid[0:n, 0:n].T.reshape(-1, 2) * args.spacing_m

    w, h = size
    pts = np.concatenate(sets)
    halfdiag = 0.5 * np.hypot(w, h)
    rmax = np.max(np.linalg.norm(pts - np.array([w / 2, h / 2]), axis=1))
    print(f"{len(names)} frames with a complete {n}x{n} grid, {w}x{h}")
    print(f"observations reach {100*rmax/halfdiag:.0f}% of the half-diagonal")

    # estimation / validation split, exactly the project's rule
    val_idx = list(range(0, len(names), 5))
    tr_idx = [i for i in range(len(names)) if i not in val_idx]
    tr_sets = [sets[i] for i in tr_idx]
    tr_names = [names[i] for i in tr_idx]
    val_sets = [sets[i] for i in val_idx]
    print(f"estimation {len(tr_idx)} / validation {len(val_idx)}\n")

    print(f"{'order':6s} {'in-sample':>10s} {'stage2':>8s} {'val mean':>10s} {'val max':>9s}   K")
    results = {}
    for key, flags in ORDERS.items():
        r = stage2(objp, tr_sets, tr_names, size, flags)
        vm, vx = fisheye_pose_only(r["K"], r["D"], objp, val_sets)
        results[key] = (r, vm, vx)
        K = r["K"]
        print(f"{key:6s} {r['per_image'].mean():9.3f}p {len(tr_sets)-len(r['kept']):7d} "
              f"{vm:9.3f}p {vx:8.2f}p   fx {K[0,0]:7.1f} cx {K[0,2]:7.1f} "
              f"D {np.round(r['D'].ravel(), 4)}")

    # the paper's rule, transplanted: adopt the richer order only if the
    # validation frames confirm it (mean <= 1.02x and max <= 1.5x)
    print("\ndistortion-order verdict (paper's criterion, applied to Kannala-Brandt):")
    base = "F2"
    for cand in ("F3", "F4"):
        bm, bx = results[base][1], results[base][2]
        cm, cx = results[cand][1], results[cand][2]
        ok = cm <= bm * 1.02 and cx <= bx * 1.5
        print(f"  {base} -> {cand}: val mean {bm:.3f} -> {cm:.3f}, "
              f"max {bx:.2f} -> {cx:.2f}  =>  {'ADOPT' if ok else 'reject'}")
        if ok:
            base = cand
    print(f"  selected: {base}")

    # extrapolation check: where do the selected and the base model disagree?
    print("\nmodel divergence along the diagonal (undistorted-point distance):")
    K2, D2 = results["F2"][0]["K"], results["F2"][0]["D"]
    Kb, Db = results[base][0]["K"], results[base][0]["D"]
    ang = np.arctan2(h, w)
    for frac in (0.2, 0.4, 0.6, 0.8, 1.0):
        r = frac * halfdiag
        p = np.array([[[w / 2 + r * np.cos(ang), h / 2 + r * np.sin(ang)]]], np.float64)
        u2 = cv2.fisheye.undistortPoints(p, K2, D2, P=K2).ravel()
        ub = cv2.fisheye.undistortPoints(p, Kb, Db, P=Kb).ravel()
        tag = "observed" if frac <= rmax / halfdiag else "EXTRAPOLATED"
        print(f"  r = {frac:4.2f} half-diag: |F2 - {base}| = "
              f"{np.linalg.norm(u2-ub):8.2f} px   {tag}")


if __name__ == "__main__":
    main()
