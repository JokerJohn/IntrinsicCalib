#!/usr/bin/env python3
"""Headless RAP run: the orchestration of IntrinsicsDialog._calibrate, lifted
out of the Qt dialog so it can be scripted, calling the project's own
detect_corners_ex() and calibrate() unmodified.

Ported line-for-line from camera_intrinsics.py::_calibrate (2026-07-28 state):

  Stage 1  SB-precision subset      used = sb  if len(sb) >= max(8, len(used)//2)
  split    C / V                    V = used[::5]  when len(used) >= 15
  Stage 2  iterative robust reject  drop mean > 2.0 x median, <=3 rounds, floor 8
  order    fix-k3 vs free-k3        pose-only (solvePnP) on V,
                                    adopt free iff mean <= 1.0x and max <= 1.0x
  final    retrain winner on all retained frames, Stage 2 applied again

Writes a JSON result plus a corner cache so the comparison can score any
parameter set on identical observations.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(HERE, "..", "..", "..", "tool")))

import cv2  # noqa: E402
from intrinsic_calib.camera_intrinsics import (  # noqa: E402
    PRUNE_FACTOR, BoardSpec, calibrate, detect_corners_ex,
)

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")


def detect_all(folder: str, spec: BoardSpec):
    """The project's own detector over a folder; returns the entry list the
    dialog would hold after `Detect All`."""
    paths = sorted(p for p in glob.glob(os.path.join(folder, "*"))
                   if p.lower().endswith(IMG_EXT))
    entries, size = [], None
    for p in paths:
        g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        size = (g.shape[1], g.shape[0])
        corners, _obj, source = detect_corners_ex(g, spec)
        if corners is not None:
            entries.append({"name": os.path.basename(p), "corners": corners,
                            "source": source})
    return entries, size


def _prune_train(corner_sets, names, spec, size, model, fk3, auto_prune):
    """`train()` from _calibrate: fit, then Stage 2 rejection, up to 3 rounds."""
    r = calibrate(corner_sets, spec, size, model=model, fix_k3=fk3)
    kept = list(names)
    if auto_prune:
        cs = list(corner_sets)
        for _ in range(3):
            means = np.array([p["mean"] for p in r["per_image"]])
            med = float(np.median(means))
            keep = means <= PRUNE_FACTOR * med
            if keep.all() or int(keep.sum()) < 8:
                break
            cs = [c for c, k in zip(cs, keep) if k]
            kept = [n for n, k in zip(kept, keep) if k]
            r = calibrate(cs, spec, size, model=model, fix_k3=fk3)
    r["used_names"] = kept
    return r


def run_rap(entries, spec, size, model="plumb_bob", fix_k3=True, auto_prune=True):
    used = list(entries)
    trace = {"n_input": len(used)}

    # ---- Stage 1: keep the SB-precision subset when there is enough of it ----
    sb = [e for e in used if e["source"] == "sb"]
    stage1_dropped = []
    if spec.pattern != "aprilgrid" and len(sb) >= max(8, len(used) // 2):
        stage1_dropped = [e["name"] for e in used if e["source"] != "sb"]
        used = sb
    trace["stage1_dropped"] = stage1_dropped
    trace["n_after_stage1"] = len(used)

    # ---- estimation / validation split ----
    if len(used) >= 15:
        holdout = used[::5]
        train = [e for e in used if e not in holdout]
    else:
        holdout, train = [], used
    trace["validation_set"] = [e["name"] for e in holdout]

    names_train = [e["name"] for e in train]
    cs_train = [e["corners"] for e in train]
    full_names = [e["name"] for e in used]
    full_sets = [e["corners"] for e in used]

    res = _prune_train(cs_train, names_train, spec, size, model, fix_k3, auto_prune)

    # ---- distortion-order selection, pose-only on the validation subset ----
    if fix_k3 and model == "plumb_bob" and spec.pattern != "aprilgrid" and holdout:
        alt = _prune_train(cs_train, names_train, spec, size, model, False, auto_prune)
        objd = spec.object_points().astype(np.float64)

        def ho_err(r):
            Km = np.array(r["K"])
            Dm = np.array((list(r["dist"]) + [0.0] * 5)[:5])
            es = []
            for e in holdout:
                uv = e["corners"].astype(np.float64)
                okp, rv, tv = cv2.solvePnP(objd, uv, Km, Dm)
                if not okp:
                    continue
                proj, _ = cv2.projectPoints(objd, rv, tv, Km, Dm)
                es.append(float(np.linalg.norm(proj.reshape(-1, 2) - uv, axis=1).mean()))
            return (float(np.mean(es)), float(np.max(es))) if es else (np.inf, np.inf)

        base_mean, base_max = ho_err(res)
        alt_mean, alt_max = ho_err(alt)
        use_free = alt_mean <= base_mean * 1.0 and alt_max <= base_max * 1.0
        trace["order"] = {
            "M2_val_mean": base_mean, "M2_val_max": base_max,
            "M3_val_mean": alt_mean, "M3_val_max": alt_max,
            "chosen": "M3 (free k3)" if use_free else "M2 (fix k3)",
        }
        # winner retrained on every retained frame
        res = _prune_train(full_sets, full_names, spec, size, model,
                           not use_free, auto_prune)
    else:
        trace["order"] = {"chosen": "M2 (fix k3)",
                          "reason": f"no validation split ({len(used)} < 15 images)"}

    trace["stage2_dropped"] = [n for n in full_names if n not in set(res["used_names"])]
    trace["n_final"] = len(res["used_names"])
    res["trace"] = trace
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--out", required=True)
    ap.add_argument("--corner-cache")
    ap.add_argument("--exclude", default="", help="comma-separated names to withhold")
    args = ap.parse_args()

    sess = json.load(open(os.path.join(args.folder, ".intrinsics_session.json")))
    b = sess["board"]
    spec = BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                     square_mm=b["square_mm"], detector="auto", extractor="legacy")

    entries, size = detect_all(args.folder, spec)
    if args.corner_cache:
        json.dump({"image_size": list(size), "board": b,
                   "entries": [{"name": e["name"], "source": e["source"],
                                "corners": e["corners"].tolist()} for e in entries]},
                  open(args.corner_cache, "w"))

    held = set(x for x in args.exclude.split(",") if x)
    run_on = [e for e in entries if e["name"] not in held]
    res = run_rap(run_on, spec, size)

    out = {k: res[k] for k in ("model", "rms", "K", "dist", "image_size")}
    out["trace"] = res["trace"]
    out["used_names"] = res["used_names"]
    out["mean_corner"] = float(np.mean([p["mean"] for p in res["per_image"]]))
    json.dump(out, open(args.out, "w"), indent=1)
    t = res["trace"]
    print(f"RAP  {args.folder}: {t['n_input']} detected -> stage1 {t['n_after_stage1']} "
          f"-> final {t['n_final']} | {t['order']['chosen']} | "
          f"in-sample mean corner {out['mean_corner']:.3f} px")


if __name__ == "__main__":
    main()
