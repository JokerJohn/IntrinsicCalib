import numpy as np, cv2, sys
import os
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("CALIB_TOOL_DIR", os.path.join(_HERE, "..", "..", "tool")))
exec(open("diag_extractor.py").read().split("# ---- Kalibr extractor")[0].split("cache = ")[0])
frames = np.load("cam0_frames.npz")["frames"]
from fast_calib_offline_gui.kalibr_extractor import detect_aprilgrid_kalibr
kal=[]
for i,g in enumerate(frames):
    uv,obj,_=detect_aprilgrid_kalibr(g,TR,TC,TS,SP)
    if uv is not None: kal.append((i,uv.astype(np.float64),obj.astype(np.float64)))
def per_view(K,D,sets):
    out=[]
    for i,uv,obj in sets:
        ok,rv,tv=cv2.solvePnP(obj,uv,K,D); rv,tv=cv2.solvePnPRefineLM(obj,uv,K,D,rv,tv)
        p,_=cv2.projectPoints(obj,rv,tv,K,D); out.append(p.reshape(-1,2)-uv)
    return out
r=per_view(KAL_K,KAL_D,kal)
pv=np.array([np.linalg.norm(x,axis=1).mean() for x in r])
print("per-view mean |e| percentiles 10/50/90/99/max:",np.percentile(pv,[10,50,90,99,100]).round(3))
print("views > 2x median:",(pv>2*np.median(pv)).sum(),"of",len(pv))
allr=np.vstack(r); n=np.linalg.norm(allr,axis=1)
print("corner |e| median %.3f, p90 %.3f, p99 %.3f, max %.2f"%(np.median(n),*np.percentile(n,[90,99,100])))
# Kalibr-style: iteratively drop corners beyond 3 sigma (per axis), report std
m=np.ones(len(n),bool)
for _ in range(5):
    s=allr[m].std(0); m=(np.abs(allr)<=3*s).all(1)
print("after 3-sigma corner rejection: kept %.1f%%, per-axis std %s, mean |e| %.3f"%(100*m.mean(),allr[m].std(0).round(3),n[m].mean()))
keep=pv<=2*np.median(pv)
r2=np.vstack([x for x,k in zip(r,keep) if k]); n2=np.linalg.norm(r2,axis=1)
print("after view rejection (2x median): per-axis std %s, mean |e| %.3f"%(r2.std(0).round(3),n2.mean()))
# blur vs error
sharp=np.array([cv2.Laplacian(frames[i],cv2.CV_64F).var() for i,_,_ in kal])
print("corr(per-view err, sharpness) = %.2f"%np.corrcoef(pv,sharp)[0,1], " err of sharpest/blurriest quartile: %.3f / %.3f"%(pv[sharp>=np.percentile(sharp,75)].mean(), pv[sharp<=np.percentile(sharp,25)].mean()))
