#!/usr/bin/env python3
"""FLIR cam0: is our final calibration really worse than Kalibr's?

Two sessions of the same camera: D1 = 2026-02-22 calib bag, D2 = 2026-02-23.
Kalibr published one pinhole-radtan calibration per session (in-sample on all
frames of that session, with its own view selection and corner outlier
rejection).  Every method below is trained on one session and scored on the
OTHER session, so no method has seen the frames it is scored on:

  kalibr       Kalibr's published parameters of the training session
  M2-all       OpenCV, k1 k2 p1 p2 (= Kalibr radtan), every detected frame
  M3-all       same with k3 free
  M2-3sigma    M2-all + Kalibr-style per-corner 3-sigma rejection (refit)
  RAP          our pipeline: Stage 2 (2x median, <=3 rounds, floor 8) inside
               each order, order chosen on a 1-in-5 held-out split, final
               refit on all frames

Scoring: intrinsics frozen, pose refitted per view (solvePnP + LM), on the
Kalibr-extracted corners of the other session.  Reported both as the mean
corner error over all views and over the views that every method fits below
3x the median (gross views excluded identically for all methods).
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(_HERE, "..", "..", "tool")))
from fast_calib_offline_gui.kalibr_extractor import detect_aprilgrid_kalibr  # noqa: E402

OUT = os.path.dirname(os.path.abspath(__file__))
# Dataset root (the Rig-C bags, see data/rigc/MANIFEST.md); the Kalibr files ship with the repository.
D = os.environ.get("RIGC_DIR", os.path.join(_HERE, "..", "..", "data", "rigc")).rstrip("/") + "/"
_KAL = os.path.join(_HERE, "..", "..", "data", "rigc")
BAGS = {"D1": D + "calib_D1.bag",
        "D2": D + "calib_D2.bag"}
KALIBR = {  # cam0, pinhole-radtan, from calib-camchain.yaml of each session
    "D1": ([484.01330131237745, 484.26375135220377, 721.7959323905397, 558.0335639805312],
           [-0.29170656681869545, 0.06664015869901406, 2.6003216596430283e-05, 0.0004704173814457229]),
    "D2": ([476.90725234274964, 476.95921523826524, 720.5493015512368, 557.5868494955138],
           [-0.2908350629887621, 0.0724987249850815, 0.0005710081088912554, 0.0005936548939904376]),
}
SIZE = (1440, 1080)
TR, TC, TS, SP = 3, 3, 0.115, 0.3


def frames(day):
    cache = os.path.join(OUT, f"cam0_frames_{day}.npz")
    if day == "D2" and os.path.exists(os.path.join(OUT, "cam0_frames.npz")):
        cache = os.path.join(OUT, "cam0_frames.npz")
    if not os.path.exists(cache):
        fr = []
        with AnyReader([Path(BAGS[day])]) as rd:
            con = [c for c in rd.connections if c.topic == "/sensors/cam0/compressed"]
            for c, ts, raw in rd.messages(connections=con):
                m = rd.deserialize(raw, c.msgtype)
                fr.append(cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE))
        np.savez_compressed(cache, frames=np.stack(fr))
    return np.load(cache)["frames"]


def extract(day, fr, refine=None):
    obs = []
    for i, g in enumerate(fr):
        uv, obj, _ = detect_aprilgrid_kalibr(g, TR, TC, TS, SP)
        if uv is None:
            continue
        uv = uv.astype(np.float64)
        if refine is not None:
            uv = refine(g, uv)
        obs.append(dict(i=i, uv=uv, obj=obj.astype(np.float64)))
    return obs


def KD(k, d):
    K = np.array([[k[0], 0, k[2]], [0, k[1], k[3]], [0, 0, 1.0]])
    return K, np.array((list(d) + [0.0] * 5)[:5])


def fit(obs, fix_k3=True):
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    K0 = np.array([[480.0, 0, 720], [0, 480, 540], [0, 0, 1]])
    rms, K, Dd, rv, tv = cv2.calibrateCamera(
        [o["obj"].astype(np.float32) for o in obs], [o["uv"].astype(np.float32) for o in obs],
        SIZE, K0, np.zeros(5), flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
    per = []
    for o, r, t in zip(obs, rv, tv):
        p, _ = cv2.projectPoints(o["obj"], r, t, K, Dd)
        per.append(p.reshape(-1, 2) - o["uv"])
    return K, Dd.ravel(), per


def view_errors(K, Dd, obs):
    out = []
    for o in obs:
        ok, rv, tv = cv2.solvePnP(o["obj"], o["uv"], K, Dd)
        rv, tv = cv2.solvePnPRefineLM(o["obj"], o["uv"], K, Dd, rv, tv)
        p, _ = cv2.projectPoints(o["obj"], rv, tv, K, Dd)
        out.append(np.linalg.norm(p.reshape(-1, 2) - o["uv"], axis=1))
    return out


def stage2(obs, fix_k3):
    cur = list(obs)
    K, Dd, per = fit(cur, fix_k3)
    for _ in range(3):
        m = np.array([np.linalg.norm(r, axis=1).mean() for r in per])
        keep = m <= 2.0 * np.median(m)
        if keep.all() or keep.sum() < 8:
            break
        cur = [o for o, k in zip(cur, keep) if k]
        K, Dd, per = fit(cur, fix_k3)
    return K, Dd, cur


def rap(obs):
    V = obs[::5]
    C = [o for o in obs if o not in V]
    res = {}
    for name, fk3 in (("M2", True), ("M3", False)):
        K, Dd, _ = stage2(C, fk3)
        e = view_errors(K, Dd, V)
        pv = np.array([x.mean() for x in e])
        res[name] = (pv.mean(), pv.max())
    use_m3 = res["M3"][0] <= res["M2"][0] and res["M3"][1] <= res["M2"][1]
    K, Dd, kept = stage2(obs, not use_m3)
    return K, Dd, dict(order="M3" if use_m3 else "M2", val=res, kept=len(kept), n=len(obs))


def three_sigma(obs):
    cur = [dict(o) for o in obs]
    for _ in range(5):
        K, Dd, per = fit(cur, True)
        allr = np.vstack(per)
        s = allr.std(0)
        new = []
        for o, r in zip(cur, per):
            m = (np.abs(r) <= 3 * s).all(1)
            if m.sum() >= 8:
                new.append(dict(o, uv=o["uv"][m], obj=o["obj"][m]))
        if sum(len(o["uv"]) for o in new) == sum(len(o["uv"]) for o in cur):
            break
        cur = new
    K, Dd, _ = fit(cur, True)
    return K, Dd


def summarize(errs_by_method, gross_mask):
    rows = {}
    for k, e in errs_by_method.items():
        allc = np.concatenate(e)
        clean = np.concatenate([x for x, g in zip(e, gross_mask) if not g])
        rows[k] = dict(mean=float(allc.mean()), median=float(np.median(allc)),
                       mean_clean=float(clean.mean()), median_clean=float(np.median(clean)))
    return rows


import functools
print = functools.partial(print, flush=True)


def main():
    obs = {d: extract(d, frames(d)) for d in ("D1", "D2")}
    # OpenCV's dense LM is O(n_views^3): train on every 3rd frame (still ~130-190
    # views per session), score on every frame of the other session.
    train_obs = {d: obs[d][::3] for d in obs}
    for d in obs:
        n = [len(o["uv"]) for o in obs[d]]
        print(f"{d}: {len(obs[d])} frames with >=4 tags; corners/frame median {np.median(n):.0f}, "
              f"full 36: {sum(x == 36 for x in n)}")
    report = {}
    for train, test in (("D2", "D1"), ("D1", "D2")):
        tr, te = train_obs[train], obs[test]
        params = {"kalibr": KD(*KALIBR[train])}
        K, Dd, _ = fit(tr, True); params["M2-all"] = (K, Dd)
        K, Dd, _ = fit(tr, False); params["M3-all"] = (K, Dd)
        params["M2-3sigma"] = three_sigma(tr)
        K, Dd, info = rap(tr); params["RAP"] = (K, Dd)
        errs = {k: view_errors(K_, D_, te) for k, (K_, D_) in params.items()}
        pv = np.array([[x.mean() for x in errs[k]] for k in errs])        # methods x views
        gross = (pv > 3 * np.median(pv, axis=1, keepdims=True)).any(0)
        rows = summarize(errs, gross)
        print(f"\n=== train {train} -> test {test}  ({len(te)} views, {gross.sum()} gross views "
              f"excluded in *_clean) | RAP: {info['order']}, kept {info['kept']}/{info['n']}, "
              f"val M2 {info['val']['M2'][0]:.3f}/{info['val']['M2'][1]:.3f} "
              f"M3 {info['val']['M3'][0]:.3f}/{info['val']['M3'][1]:.3f}")
        for k, r in rows.items():
            Km, Dm = params[k]
            print(f"  {k:10s} fx {Km[0,0]:7.2f} cx {Km[0,2]:7.2f} cy {Km[1,2]:7.2f} | "
                  f"mean {r['mean']:.3f} median {r['median']:.3f} | clean mean {r['mean_clean']:.3f} "
                  f"median {r['median_clean']:.3f}")
        report[f"{train}->{test}"] = dict(rows=rows, rap=info,
                                          params={k: dict(K=v[0].tolist(), D=list(map(float, v[1])))
                                                  for k, v in params.items()})
    json.dump(report, open(os.path.join(OUT, "flir_compare.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
