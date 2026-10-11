#!/usr/bin/env python3
"""Cross-session evaluation on the FLIR stereo pair, all frames of both sessions.

Sessions of the same two cameras: D1 = 2026-02-22 calib bag, D2 = 2026-02-23.
Every method is estimated on all frames of one session and scored on all
frames of the other (intrinsics frozen, board pose refitted per view), so no
method sees the frames it is scored on.  The yardstick is the same for every
method: the raw Kalibr tag corners of the test session.

Methods (K = raw Kalibr corners, R = refined corners of the proposed extractor)
  kalibr        Kalibr's published pinhole-radtan calibration of the session
  M2-K          OpenCV k1 k2 p1 p2 (= Kalibr's model), every Kalibr frame
  M3-K          + k3
  M2-K-3sigma   M2-K with Kalibr-style per-corner 3-sigma rejection
  RAP-K         Stage 2 + order selection on raw Kalibr corners (no Stage 1)
  M2-R          proposed extractor, every frame (Stage 1 rejects included
                through their raw corners), no selection: the ablation
  RAP           the proposed pipeline: Stage 1 (corner acceptance, >= 4
                complete tags), Stage 2 inside each order, order selected on
                the 1-in-5 held-out split, final refit on all accepted frames

Writes flir_full_<cam>.json (parameters, per-view errors) and one intrinsics
yaml per (cam, session, method) under intrinsics/ for the SLAM experiments.
"""
from __future__ import annotations

import argparse
import functools
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(_HERE, "..", "..", "tool")))
from fast_calib_offline_gui.aprilgrid_refine import detect_aprilgrid_rap   # noqa: E402
from fast_calib_offline_gui.kalibr_extractor import detect_aprilgrid_kalibr  # noqa: E402

print = functools.partial(print, flush=True)
cv2.setNumThreads(1)
OUT = os.path.dirname(os.path.abspath(__file__))
# Dataset root (the Rig-C bags, see data/rigc/MANIFEST.md); the Kalibr files ship with the repository.
D = os.environ.get("RIGC_DIR", os.path.join(_HERE, "..", "..", "data", "rigc")).rstrip("/") + "/"
_KAL = os.path.join(_HERE, "..", "..", "data", "rigc")
BAGS = {"D1": D + "calib_D1.bag",
        "D2": D + "calib_D2.bag"}
CAMCHAIN = {"D1": os.path.join(_KAL, "kalibr_D1", "calib-camchain.yaml"),
            "D2": os.path.join(_KAL, "kalibr_D2", "calib-camchain.yaml")}
SIZE = (1440, 1080)
TR, TC, TS, SP = 3, 3, 0.115, 0.3
LU = cv2.CALIB_USE_LU
KAPPA, NMIN, NSTOP, TMAX = 2.0, 15, 8, 3


# ----------------------------------------------------------------- data
def frames(day, cam):
    cache = os.path.join(OUT, f"frames_{day}_{cam}.npz")
    legacy = {("D2", "cam0"): "cam0_frames.npz", ("D1", "cam0"): "cam0_frames_D1.npz"}
    if (day, cam) in legacy and os.path.exists(os.path.join(OUT, legacy[(day, cam)])):
        cache = os.path.join(OUT, legacy[(day, cam)])
    if not os.path.exists(cache):
        fr = []
        with AnyReader([Path(BAGS[day])]) as rd:
            con = [c for c in rd.connections if c.topic == f"/sensors/{cam}/compressed"]
            for c, ts, raw in rd.messages(connections=con):
                m = rd.deserialize(raw, c.msgtype)
                fr.append(cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE))
        np.savez_compressed(cache, frames=np.stack(fr))
    return np.load(cache)["frames"]


def extract(day, cam):
    cache = os.path.join(OUT, f"corners_{day}_{cam}.json")
    if os.path.exists(cache):
        d = json.load(open(cache))
        return [dict(i=o["i"], src=o["src"], uvK=np.array(o["uvK"]), objK=np.array(o["objK"]),
                     uvR=np.array(o["uvR"]), objR=np.array(o["objR"])) for o in d]
    fr = frames(day, cam)
    obs = []
    for i, g in enumerate(fr):
        uvK, objK, _ = detect_aprilgrid_kalibr(g, TR, TC, TS, SP)
        if uvK is None:
            continue
        uvR, objR, src, _msg = detect_aprilgrid_rap(g, TR, TC, TS, SP)
        obs.append(dict(i=i, src=src, uvK=uvK.astype(np.float64), objK=objK.astype(np.float64),
                        uvR=uvR.astype(np.float64), objR=objR.astype(np.float64)))
    json.dump([dict(i=o["i"], src=o["src"], uvK=o["uvK"].tolist(), objK=o["objK"].tolist(),
                    uvR=o["uvR"].tolist(), objR=o["objR"].tolist()) for o in obs],
              open(cache, "w"))
    return obs


def kalibr_published(day, cam):
    c = yaml.safe_load(open(CAMCHAIN[day]))[cam]
    k, d = c["intrinsics"], c["distortion_coeffs"]
    return KD(k, d)


def KD(k, d):
    K = np.array([[k[0], 0, k[2]], [0, k[1], k[3]], [0, 0, 1.0]])
    return K, np.array((list(d) + [0.0] * 5)[:5])


# ----------------------------------------------------------------- estimation
def sets(obs, which):
    return [o["uv" + which] for o in obs], [o["obj" + which] for o in obs]


_NFIT = [0]


def fit(uvs, objs, fix_k3=True):
    t = time.time()
    r = _fit(uvs, objs, fix_k3)
    _NFIT[0] += 1
    print(f"    fit #{_NFIT[0]} n={len(uvs)} k3={'free' if not fix_k3 else 'fix'} {time.time()-t:.1f}s")
    return r


def _fit(uvs, objs, fix_k3=True):
    flags = (cv2.CALIB_FIX_K3 if fix_k3 else 0) | LU | cv2.CALIB_USE_INTRINSIC_GUESS
    K0 = np.array([[480.0, 0, SIZE[0] / 2], [0, 480.0, SIZE[1] / 2], [0, 0, 1]])
    rms, K, Dd, rv, tv = cv2.calibrateCamera(
        [o.astype(np.float32) for o in objs], [u.astype(np.float32) for u in uvs],
        SIZE, K0, np.zeros(5), flags=flags)
    per = []
    for u, o, r, t in zip(uvs, objs, rv, tv):
        p, _ = cv2.projectPoints(o, r, t, K, Dd)
        per.append(np.linalg.norm(p.reshape(-1, 2) - u, axis=1))
    return K, Dd.ravel(), per


def view_errors(K, Dd, uvs, objs):
    out = []
    for u, o in zip(uvs, objs):
        ok, rv, tv = cv2.solvePnP(o, u, K, Dd)
        rv, tv = cv2.solvePnPRefineLM(o, u, K, Dd, rv, tv)
        p, _ = cv2.projectPoints(o, rv, tv, K, Dd)
        out.append(np.linalg.norm(p.reshape(-1, 2) - u, axis=1))
    return out


def stage2(uvs, objs, fix_k3):
    """Iterative median-scaled rejection, eqs. (2)-(3): tau = kappa * median."""
    uvs, objs = list(uvs), list(objs)
    K, Dd, per = fit(uvs, objs, fix_k3)
    for _ in range(TMAX):
        m = np.array([e.mean() for e in per])
        keep = m <= KAPPA * np.median(m)
        if keep.all() or keep.sum() < NSTOP:
            break
        uvs = [u for u, k in zip(uvs, keep) if k]
        objs = [o for o, k in zip(objs, keep) if k]
        K, Dd, per = fit(uvs, objs, fix_k3)
    return K, Dd, len(uvs)


def rap(uvs, objs):
    """Split 1-in-5, Stage 2 inside each order, criterion (6), final refit."""
    n = len(uvs)
    if n < NMIN:
        K, Dd, kept = stage2(uvs, objs, True)
        return K, Dd, dict(order="M2", reason="no split", kept=kept, n=n)
    V = list(range(0, n, 5))
    C = [i for i in range(n) if i not in set(V)]
    val = {}
    for name, fk3 in (("M2", True), ("M3", False)):
        K, Dd, _ = stage2([uvs[i] for i in C], [objs[i] for i in C], fk3)
        e = view_errors(K, Dd, [uvs[i] for i in V], [objs[i] for i in V])
        pv = np.array([x.mean() for x in e])
        val[name] = (float(pv.mean()), float(pv.max()))
    use_m3 = val["M3"][0] <= val["M2"][0] and val["M3"][1] <= val["M2"][1]
    K, Dd, kept = stage2(uvs, objs, not use_m3)
    return K, Dd, dict(order="M3" if use_m3 else "M2", val=val, kept=kept, n=n, nV=len(V))


def three_sigma(uvs, objs):
    uvs, objs = list(uvs), list(objs)
    for _ in range(5):
        K, Dd, per = fit(uvs, objs, True)
        res = []
        for u, o in zip(uvs, objs):
            ok, rv, tv = cv2.solvePnP(o, u, K, Dd)
            p, _ = cv2.projectPoints(o, rv, tv, K, Dd)
            res.append(p.reshape(-1, 2) - u)
        s = np.vstack(res).std(0)
        n_before = sum(len(u) for u in uvs)
        nu, no = [], []
        for u, o, r in zip(uvs, objs, res):
            m = (np.abs(r) <= 3 * s).all(1)
            if m.sum() >= 8:
                nu.append(u[m]); no.append(o[m])
        uvs, objs = nu, no
        if sum(len(u) for u in uvs) == n_before:
            break
    K, Dd, _ = fit(uvs, objs, True)
    return K, Dd


# ----------------------------------------------------------------- driver
def write_yaml(path, K, Dd, note):
    d = dict(cam_model="Pinhole", cam_width=SIZE[0], cam_height=SIZE[1], scale=1.0,
             cam_fx=float(K[0, 0]), cam_fy=float(K[1, 1]), cam_cx=float(K[0, 2]),
             cam_cy=float(K[1, 2]), cam_d0=float(Dd[0]), cam_d1=float(Dd[1]),
             cam_d2=float(Dd[2]), cam_d3=float(Dd[3]), cam_d4=float(Dd[4]),
             intrinsics=[float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])],
             distortion_coeffs=[float(x) for x in Dd], distortion_model="radtan", note=note)
    yaml.safe_dump(d, open(path, "w"), sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cam", choices=["cam0", "cam1"])
    args = ap.parse_args()
    cam = args.cam
    t0 = time.time()
    obs = {d: extract(d, cam) for d in ("D1", "D2")}
    for d in obs:
        src = [o["src"] for o in obs[d]]
        print(f"{cam} {d}: {len(src)} Kalibr frames, Stage 1 accepts {src.count('rap')} "
              f"({time.time()-t0:.0f}s)")
    os.makedirs(os.path.join(OUT, "intrinsics"), exist_ok=True)
    report = {}
    for train, test in (("D1", "D2"), ("D2", "D1")):
        tr, te = obs[train], obs[test]
        uvK, objK = sets(tr, "K")
        uvR, objR = sets(tr, "R")
        acc = [o for o in tr if o["src"] == "rap"]
        uvA, objA = sets(acc, "R")
        P, info = {}, {}
        P["kalibr"] = kalibr_published(train, cam)
        K, Dd, _ = fit(uvK, objK, True); P["M2-K"] = (K, Dd)
        K, Dd, _ = fit(uvK, objK, False); P["M3-K"] = (K, Dd)
        P["M2-K-3sigma"] = three_sigma(uvK, objK)
        K, Dd, i = rap(uvK, objK); P["RAP-K"] = (K, Dd); info["RAP-K"] = i
        K, Dd, _ = fit(uvR, objR, True); P["M2-R"] = (K, Dd)
        if len(acc) >= max(NSTOP, len(tr) // 2):
            K, Dd, i = rap(uvA, objA)
            i["stage1_dropped"] = len(tr) - len(acc)
        else:
            K, Dd, i = rap(uvR, objR)
            i["stage1_dropped"] = 0; i["stage1"] = "fallback: too few accepted"
        P["RAP"] = (K, Dd); info["RAP"] = i
        print(f"  fits done ({time.time()-t0:.0f}s)")

        uvT, objT = sets(te, "K")
        errs = {k: view_errors(K_, D_, uvT, objT) for k, (K_, D_) in P.items()}
        pv = np.array([[x.mean() for x in errs[k]] for k in P])
        gross = (pv > 3 * np.median(pv, axis=1, keepdims=True)).any(0)
        key = f"{train}->{test}"
        rows = {}
        print(f"\n=== {cam}: train {train} ({len(tr)} frames) -> test {test} "
              f"({len(te)} views, {int(gross.sum())} gross excluded in clean) ===")
        for k, (K_, D_) in P.items():
            allc = np.concatenate(errs[k])
            clean = np.concatenate([e for e, g in zip(errs[k], gross) if not g])
            rows[k] = dict(mean=float(allc.mean()), median=float(np.median(allc)),
                           mean_clean=float(clean.mean()), median_clean=float(np.median(clean)),
                           pv_mean=float(pv[list(P).index(k)].mean()),
                           K=K_.tolist(), D=[float(x) for x in D_], info=info.get(k))
            tag = ""
            if k in info:
                i = info[k]
                tag = f"| {i['order']}, kept {i['kept']}/{i['n']}"
                if "val" in i:
                    tag += (f", val M2 {i['val']['M2'][0]:.3f}/{i['val']['M2'][1]:.3f} "
                            f"M3 {i['val']['M3'][0]:.3f}/{i['val']['M3'][1]:.3f}")
                if "stage1_dropped" in i:
                    tag += f", S1 -{i['stage1_dropped']}"
            print(f"  {k:12s} fx {K_[0,0]:7.2f} fy {K_[1,1]:7.2f} cx {K_[0,2]:7.2f} cy {K_[1,2]:7.2f} "
                  f"k1 {D_[0]:+.4f} k2 {D_[1]:+.4f} k3 {D_[4]:+.4f} | mean {rows[k]['mean']:.3f} "
                  f"median {rows[k]['median']:.3f} | clean {rows[k]['mean_clean']:.3f}/"
                  f"{rows[k]['median_clean']:.3f} {tag}")
            write_yaml(os.path.join(OUT, "intrinsics", f"{cam}_{train}_{k}.yaml"), K_, D_,
                       f"{cam}, estimated on session {train}, method {k}")
        report[key] = dict(rows=rows, gross=int(gross.sum()), n_test=len(te), n_train=len(tr))
    # cross-session parameter repeatability
    rep = {}
    print(f"\n=== {cam}: |D1 - D2| of the estimated parameters [px] ===")
    for k in report["D1->D2"]["rows"]:
        a = np.array(report["D1->D2"]["rows"][k]["K"]); b = np.array(report["D2->D1"]["rows"][k]["K"])
        rep[k] = dict(dfx=abs(a[0, 0] - b[0, 0]), dfy=abs(a[1, 1] - b[1, 1]),
                      dcx=abs(a[0, 2] - b[0, 2]), dcy=abs(a[1, 2] - b[1, 2]))
        print(f"  {k:12s} dfx {rep[k]['dfx']:6.2f} dfy {rep[k]['dfy']:6.2f} "
              f"dcx {rep[k]['dcx']:6.2f} dcy {rep[k]['dcy']:6.2f}")
    report["repeatability"] = rep
    json.dump(report, open(os.path.join(OUT, f"flir_full_{cam}.json"), "w"), indent=1)
    print(f"done in {time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
