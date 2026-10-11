#!/usr/bin/env python3
"""Why our AprilGrid corners fit worse than Kalibr's on the FLIR cam0 session.

Separates (i) the error definition (Kalibr prints the per-axis mean +- std of
the residual vector, we report the mean corner distance) from (ii) the
extractor (Kalibr: tag corners in the raw image; legacy: undistort with a rough
model, complete the grid through a homography, refine, map back)."""
import json, os, sys
import numpy as np, cv2
from pathlib import Path
from rosbags.highlevel import AnyReader
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(_HERE, "..", "..", "tool")))
from fast_calib_offline_gui.kalibr_extractor import detect_aprilgrid_kalibr
from fast_calib_offline_gui.feature_detector import detect_aprilgrid_corners
from fast_calib_offline_gui.models import BoardConfig, CameraIntrinsics

OUT = os.path.dirname(os.path.abspath(__file__))
BAG = os.path.join(os.environ.get("RIGC_DIR", os.path.join(_HERE, "..", "..", "data", "rigc")), "calib_D2.bag")
TR, TC, TS, SP = 3, 3, 0.115, 0.3
SIZE = (1440, 1080)
KAL_K = np.array([[476.90725234274964, 0, 720.5493015512368], [0, 476.95921523826524, 557.5868494955138], [0, 0, 1.]])
KAL_D = np.array([-0.2908350629887621, 0.0724987249850815, 0.0005710081088912554, 0.0005936548939904376])

cache = os.path.join(OUT, "cam0_frames.npz")
if not os.path.exists(cache):
    frames = []
    with AnyReader([Path(BAG)]) as rd:
        con = [c for c in rd.connections if c.topic == "/sensors/cam0/compressed"]
        for c, ts, raw in rd.messages(connections=con):
            m = rd.deserialize(raw, c.msgtype)
            frames.append(cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE))
    np.savez_compressed(cache, frames=np.stack(frames))
frames = np.load(cache)["frames"]
print("frames", len(frames))

# ---- Kalibr extractor on every frame ------------------------------------
kal = []
for i, g in enumerate(frames):
    uv, obj, msg = detect_aprilgrid_kalibr(g, TR, TC, TS, SP)
    if uv is not None:
        kal.append((i, uv.astype(np.float64), obj.astype(np.float64)))
print("Kalibr extractor: valid frames", len(kal), "corners/frame median", np.median([len(u) for _, u, _ in kal]))

def residuals(K, D, sets, refit=True):
    res = []
    for _, uv, obj in sets:
        ok, rv, tv = cv2.solvePnP(obj, uv, K, D)
        rv, tv = cv2.solvePnPRefineLM(obj, uv, K, D, rv, tv)
        p, _ = cv2.projectPoints(obj, rv, tv, K, D)
        res.append(p.reshape(-1, 2) - uv)
    return np.vstack(res)

def report(tag, r):
    n = np.linalg.norm(r, axis=1)
    print(f"  {tag:42s} per-axis mean {r.mean(0).round(4)} std {r.std(0).round(3)} | "
          f"mean |e| {n.mean():.3f}  RMS |e| {np.sqrt((n**2).mean()):.3f}  N={len(n)}")

def calib(sets, flags):
    rms, K, D, rv, tv = cv2.calibrateCamera([o.astype(np.float32) for _, _, o in sets],
                                           [u.astype(np.float32) for _, u, _ in sets], SIZE,
                                           KAL_K.copy(), np.zeros(5), flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
    return K, D.ravel()

print("\n(1) error definition, on Kalibr-extracted corners (in-sample, all frames)")
report("Kalibr's published intrinsics", residuals(KAL_K, KAL_D, kal))
K4, D4 = calib(kal, cv2.CALIB_FIX_K3)
report("OpenCV fit, radtan k1 k2 p1 p2 (= Kalibr model)", residuals(K4, D4, kal))
print("   fx fy cx cy", K4[0, 0].round(2), K4[1, 1].round(2), K4[0, 2].round(2), K4[1, 2].round(2), "D", D4.round(4))

print("\n(2) legacy extractor (two-pass completion with Kalibr's intrinsics as the rough model)")
cam = CameraIntrinsics(fx=KAL_K[0, 0], fy=KAL_K[1, 1], cx=KAL_K[0, 2], cy=KAL_K[1, 2],
                       k1=KAL_D[0], k2=KAL_D[1], p1=KAL_D[2], p2=KAL_D[3])
board = BoardConfig(tag_rows=TR, tag_cols=TC, tag_size=TS, tag_spacing=SP)
leg = []
for i, uv, obj in kal:
    u2, o2, _ = detect_aprilgrid_corners(frames[i], board, camera=cam, complete_grid=True)
    if u2 is not None:
        leg.append((i, np.asarray(u2, np.float64).reshape(-1, 2), np.asarray(o2, np.float64).reshape(-1, 3)))
report("legacy corners, Kalibr intrinsics", residuals(KAL_K, KAL_D, leg))
KL, DL = calib(leg, cv2.CALIB_FIX_K3)
report("legacy corners, own radtan fit", residuals(KL, DL, leg))

# matched corners: same frame, same object point
d, rad = [], []
legd = {i: (u, o) for i, u, o in leg}
for i, uv, obj in kal:
    if i not in legd: continue
    u2, o2 = legd[i]
    for p, o in zip(uv, obj):
        j = np.where(np.linalg.norm(o2 - o, axis=1) < 1e-6)[0]
        if len(j):
            d.append(u2[j[0]] - p); rad.append(np.linalg.norm(p - KAL_K[:2, 2]))
d, rad = np.array(d), np.array(rad)
dn = np.linalg.norm(d, axis=1)
print(f"\n(3) legacy - Kalibr, matched corners: N={len(dn)} median {np.median(dn):.3f} px, mean {dn.mean():.3f}, p95 {np.percentile(dn,95):.3f}, >1px {100*(dn>1).mean():.1f}%")
for lo, hi in [(0, 200), (200, 400), (400, 600), (600, 900)]:
    m = (rad >= lo) & (rad < hi)
    if m.any(): print(f"   radius {lo:3d}-{hi:3d} px: median {np.median(dn[m]):.3f}  mean {dn[m].mean():.3f}  N={m.sum()}")
json.dump({"kalibr_frames": [i for i, _, _ in kal]}, open(os.path.join(OUT, "diag_extractor.json"), "w"))
