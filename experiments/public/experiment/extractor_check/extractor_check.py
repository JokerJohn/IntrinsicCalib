#!/usr/bin/env python3
"""Do the paper's results survive a change of corner extractor?

Runs the RAP pipeline unchanged (rap_headless.run_rap) with the legacy
FAST-Calib chessboard extractor (the paper) and with Kalibr's checkerboard
extractor at Kalibr's defaults, and reports the quantities quoted in the paper:

  Rig-A  Fig. 3b   four image sets under M2, scored on the six validation
                   images of the paper (pose-only)
         Sec. V-C  the order decision of the pipeline on its own split
  public Table I   4-fold held-out mean of the M2 ablation and of RAP

Each pipeline is scored on the corners of its own extractor (self-consistent),
and additionally on the legacy corners of the same held-out frames, so that a
change in the error can be attributed to the parameters rather than to the
corners they are scored on.  Nothing under ../out is written.
"""
from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(EXP, "_qtstub"))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(EXP, "..", "..", "..", "tool")))
from fast_calib_offline_gui.camera_intrinsics import BoardSpec  # noqa: E402

from protocol import ROS_FLAGS, pose_only                       # noqa: E402
from rap_headless import _prune_train, detect_all, run_rap      # noqa: E402

RIG_A = os.environ.get("RIG_A_DIR", os.path.join(EXP, "..", "..", "..", "data", "rig_a"))
PAPER_V = ["left-0000.png", "left-0006.png", "left-0011.png", "left-0019.png",
           "left-0024.png", "left-0035.png"]
PUBLIC = ["opencalib_front", "ros_wide_left", "ros_wide_right",
          "opencv_left", "opencv_right"]
ROOT = os.path.dirname(EXP)   # experiments/public
EXTRACTORS = ["legacy", "kalibr"]


def spec_for(b, extractor):
    return BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                     square_mm=b["square_mm"], detector="auto", extractor=extractor)


def m2_fit(entries, spec, size):
    sets = [e["corners"] for e in entries]
    _r, K, D, _rv, _tv = cv2.calibrateCamera(
        [spec.object_points()] * len(sets), sets, size, None, None, flags=ROS_FLAGS)
    return K, D.ravel()


def per_view_mean(K, D, objp, sets):
    """Mean over views of the per-view mean corner error (Fig. 3b metric)."""
    K = np.asarray(K, float)
    D = np.asarray((list(np.asarray(D, float).ravel()) + [0.0] * 5)[:5])
    v = []
    for uv in sets:
        uv = np.asarray(uv, np.float64)
        ok, rv, tv = cv2.solvePnP(objp, uv, K, D)
        p, _ = cv2.projectPoints(objp, rv, tv, K, D)
        v.append(np.linalg.norm(p.reshape(-1, 2) - uv, axis=1).mean())
    return float(np.mean(v))


def rig_a(out):
    b = dict(pattern="chessboard", cols=11, rows=8, square_mm=45.0)
    ref_spec = spec_for(b, "legacy")
    objp = ref_spec.object_points().astype(np.float64)
    leg_entries, size = detect_all(RIG_A, ref_spec)
    leg = {e["name"]: e for e in leg_entries}
    for ex in EXTRACTORS:
        spec = spec_for(b, ex)
        entries, size = detect_all(RIG_A, spec)
        entries.sort(key=lambda e: e["name"])
        names = [e["name"] for e in entries]
        sb = [e for e in entries if e["source"] == "sb"]
        s1 = sb if (ex == "legacy" and len(sb) >= max(8, len(entries) // 2)) else entries
        V = [e for e in entries if e["name"] in PAPER_V]
        Vsets = [e["corners"] for e in V]
        Vleg = [leg[e["name"]]["corners"] for e in V]
        rows = {}
        for label, base, prune in (("all", entries, False), ("S2", entries, True),
                                   ("S1", s1, False), ("S1+S2", s1, True)):
            tr = [e for e in base if e["name"] not in PAPER_V]
            r = _prune_train([e["corners"] for e in tr], [e["name"] for e in tr],
                             spec, size, "plumb_bob", True, prune)
            cal = float(np.mean([p["mean"] for p in r["per_image"]]))
            rows[label] = dict(n=len(r["used_names"]), calib=cal,
                               valid=per_view_mean(r["K"], r["dist"], objp, Vsets),
                               valid_on_legacy=per_view_mean(r["K"], r["dist"], objp, Vleg))
        res = run_rap(entries, spec, size)
        t = res["trace"]
        out.setdefault("rig_a", {})[ex] = dict(
            detected=len(entries), not_detected=sorted(set(leg) - set(names)),
            V_found=[e["name"] for e in V], fig3b=rows,
            pipeline=dict(stage1_dropped=len(t["stage1_dropped"]),
                          stage2_dropped=len(t["stage2_dropped"]), n_final=t["n_final"],
                          validation_set=t["validation_set"], order=t["order"],
                          K=res["K"], dist=res["dist"],
                          in_sample=float(np.mean([p["mean"] for p in res["per_image"]]))))
        print(f"\n=== Rig-A, {ex} extractor: {len(entries)}/39 detected; "
              f"paper V found {len(V)}/6 ===")
        for k, v in rows.items():
            print(f"  {k:6s} n={v['n']:2d}  calib {v['calib']:.3f}  valid {v['valid']:.3f}"
                  f"  (on legacy V corners {v['valid_on_legacy']:.3f})")
        o = t["order"]
        print(f"  pipeline: stage1 -{len(t['stage1_dropped'])}, stage2 -{len(t['stage2_dropped'])}, "
              f"final {t['n_final']} | {o.get('chosen')} | "
              + (f"M2 {o['M2_val_mean']:.3f}/{o['M2_val_max']:.3f}  "
                 f"M3 {o['M3_val_mean']:.3f}/{o['M3_val_max']:.3f}" if "M2_val_mean" in o else o.get("reason", ""))
              + f" | fx {res['K'][0][0]:.2f}")


def public(out, every=4):
    for name in PUBLIC:
        folder = os.path.join(ROOT, name)
        b = json.load(open(os.path.join(folder, ".fastcalib_intrinsics.json")))["board"]
        ref_spec = spec_for(b, "legacy")
        objp = ref_spec.object_points().astype(np.float64)
        leg_entries, _ = detect_all(folder, ref_spec)
        leg = {e["name"]: e for e in leg_entries}
        for ex in EXTRACTORS:
            spec = spec_for(b, ex)
            entries, size = detect_all(folder, spec)
            entries.sort(key=lambda e: e["name"])
            acc = {"abl": [], "rap": [], "abl_leg": [], "rap_leg": [], "order": [], "cut": []}
            for off in range(every):
                ev = entries[off::every]
                tr = [e for e in entries if e not in ev]
                evs = [e["corners"] for e in ev]
                evl = [leg[e["name"]]["corners"] for e in ev if e["name"] in leg]
                K, D = m2_fit(tr, spec, size)
                rr = run_rap(tr, spec, size)
                t = rr["trace"]
                acc["abl"].append(pose_only(K, D, objp, evs)[0])
                acc["rap"].append(pose_only(rr["K"], rr["dist"], objp, evs)[0])
                acc["abl_leg"].append(pose_only(K, D, objp, evl)[0])
                acc["rap_leg"].append(pose_only(rr["K"], rr["dist"], objp, evl)[0])
                acc["order"].append(t["order"]["chosen"][:2])
                acc["cut"].append(len(t["stage1_dropped"]) + len(t["stage2_dropped"]))
            out.setdefault("public", {}).setdefault(name, {})[ex] = dict(
                detected=len(entries), **{k: v for k, v in acc.items()})
            print(f"  {name:16s} {ex:6s} det {len(entries):2d} | abl {np.mean(acc['abl']):.3f} "
                  f"RAP {np.mean(acc['rap']):.3f} | on legacy corners: abl "
                  f"{np.mean(acc['abl_leg']):.3f} RAP {np.mean(acc['rap_leg']):.3f} | "
                  f"orders {''.join(o[1] for o in acc['order'])} cut {acc['cut']}")


def main():
    out = {}
    rig_a(out)
    print("\n=== public sessions, 4-fold held-out mean corner error [px] ===")
    public(out)
    json.dump(out, open(os.path.join(HERE, "extractor_check.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
