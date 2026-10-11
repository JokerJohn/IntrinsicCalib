#!/usr/bin/env python3
"""Table II numbers: cross-session held-out mean corner error on Rig-C for the
methods that support the AprilGrid target natively (Kalibr, ours-M2 = M2-R, ours (Full) = RAP).
Gross views are identified over these three methods only (a view is gross when
any of them exceeds three times its own median on it).

Reads flir_full_<cam>.json (written by flir_full.py; the shipped copy in
figures/data/ is used when it has not been run) and the corner caches
corners_<session>_<cam>.json, writes table2_rigc.json beside this script."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flir_full as F
import cv2
cv2.setNumThreads(1)
def _result(cam):
    p = os.path.join(F.OUT, f"flir_full_{cam}.json")
    if not os.path.exists(p):
        p = os.path.join(F.OUT, "..", "..", "figures", "data", f"flir_full_{cam}.json")
    return json.load(open(p))


out = {}
for cam in ("cam0", "cam1"):
    R = _result(cam)
    obs = {d: F.extract(d, cam) for d in ("D1", "D2")}
    out[cam] = {}
    for key in ("D1->D2", "D2->D1"):
        test = key.split("->")[1]
        uvT, objT = F.sets(obs[test], "K")
        res = {}
        for m in ("kalibr", "M2-R", "RAP"):
            row = R[key]["rows"][m]
            K = np.array(row["K"]); D = np.array(row["D"])
            res[m] = F.view_errors(K, D, uvT, objT)
        pv = np.array([[e.mean() for e in res[m]] for m in res])
        gross = (pv > 3 * np.median(pv, axis=1, keepdims=True)).any(0)
        out[cam][key] = {m: dict(clean=float(np.concatenate([e for e, g in zip(res[m], gross) if not g]).mean()),
                                 all=float(np.concatenate(res[m]).mean()),
                                 median=float(np.median(np.concatenate(res[m]))))
                         for m in res}
        out[cam][key]["n_views"] = len(uvT); out[cam][key]["n_gross"] = int(gross.sum())
        print(cam, key, len(uvT), int(gross.sum()), {m: round(v["clean"], 3) for m, v in out[cam][key].items() if isinstance(v, dict)})
json.dump(out, open(os.path.join(F.OUT, "table2_rigc.json"), "w"), indent=1)
