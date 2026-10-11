#!/usr/bin/env python3
"""mrcal as a baseline, fed the corners the RAP run already produced.

mrcal is the only tool surveyed that automatically discards bad calibration
observations, so it is the real competitor to Stage 2 -- but it discards
individual *corners*, where Stage 2 discards whole *images*.  Feeding it the
identical corner set on the identical frames isolates that difference from
everything else.

Four mrcal configurations:

  M4-flat      LENSMODEL_OPENCV4, flat board, outlier rejection ON
               -- same model class as C2/C3 (k1 k2 p1 p2), so only the solver
                  and the per-corner rejection differ
  M4-flat-noor same, --skip-outlier-rejection
               -- isolates the rejection from mrcal's solver
  M4-shipped   LENSMODEL_OPENCV4 with mrcal's default board-flex solve
               -- mrcal as a user would actually run it
  M5-flat      LENSMODEL_OPENCV5 (adds k3), flat board, outliers ON
               -- mrcal's M3 equivalent, to sit beside RAP's order choice
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import subprocess
import tempfile

import numpy as np

# Every entry runs mrcal at its defaults -- board-flex solve on, outlier
# rejection on.  Only --lensmodel varies, and only because mrcal has no default
# for it: it is a required argument the user must supply.  OPENCV4 matches the
# model class the paper and the ROS calibrator use; OPENCV8 is the model mrcal's
# own documentation uses in its example invocation.
CONFIGS = {
    "mrcal-OPENCV4": ("LENSMODEL_OPENCV4", []),
    "mrcal-OPENCV5": ("LENSMODEL_OPENCV5", []),
    "mrcal-OPENCV8": ("LENSMODEL_OPENCV8", []),
}


def write_vnl(path, entries, names_keep):
    """mrcal corners.vnl: one corner per line, `filename x y level`.

    Corner order must be the board's row-major grid, which is exactly what
    findChessboardCorners / SB return, so the cache is written straight out.
    level 0 = detected at full resolution.
    """
    with open(path, "w") as fh:
        fh.write("# filename x y level\n")
        for e in entries:
            if e["name"] not in names_keep:
                continue
            for x, y in e["corners"]:
                fh.write(f"{e['name']} {x:.6f} {y:.6f} 0\n")


def read_cameramodel(path):
    txt = open(path).read()
    # mrcal writes `'intrinsics': [ fx, fy, cx, cy, d0, d1, ... ],`
    m = re.search(r"'intrinsics'\s*:\s*[\[(]\s*([^\])]*)[\])]", txt, re.S)
    vals = [float(v) for v in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?",
                                         m.group(1))]
    fx, fy, cx, cy = vals[:4]
    K = [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]
    return K, vals[4:]


def run_one(cache, names_keep, key, outdir):
    lensmodel, extra = CONFIGS[key]
    b = cache["board"]
    w, h = cache["image_size"]
    entries = cache["entries"]
    ext = os.path.splitext(entries[0]["name"])[1]

    # absolute: the solve runs with cwd=d, so relative paths would resolve
    # against d itself
    d = os.path.abspath(os.path.join(outdir, key))
    os.makedirs(d, exist_ok=True)
    vnl = os.path.join(d, "corners.vnl")
    write_vnl(vnl, entries, names_keep)

    # mrcal's tools carry `#!/usr/bin/env python3`, which on this machine
    # resolves to anaconda's 3.10 -- where python3-mrcal (built for 3.8) is
    # not importable.  Invoke the system interpreter explicitly.
    cmd = ["/usr/bin/python3", "/usr/bin/mrcal-calibrate-cameras",
           "--corners-cache", vnl,
           "--lensmodel", lensmodel,
           "--focal", str(int(0.9 * w)),          # rough seed, as mrcal advises
           "--object-spacing", str(b["square_mm"] / 1000.0),
           "--object-width-n", str(b["cols"]),
           "--object-height-n", str(b["rows"]),
           "--imagersize", str(w), str(h),
           "--outdir", d, "--jobs", "4"] + extra + [f"*{ext}"]
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=d)
    model = os.path.join(d, "camera-0.cameramodel")
    if not os.path.exists(model):
        return {"error": (p.stderr or p.stdout)[-400:]}

    K, dist = read_cameramodel(model)
    log = p.stdout + p.stderr
    rms = re.findall(r"RMS reprojection error:\s*([0-9.]+)", open(model).read())
    nout = re.findall(r"Noutliers:\s*(\d+)\s*out of\s*(\d+)", open(model).read())
    out = {"K": K, "dist": dist, "lensmodel": lensmodel,
           "n_frames": len(names_keep),
           "rms": float(rms[0]) if rms else None}
    if nout:
        out["outlier_corners"] = int(nout[0][0])
        out["total_corners"] = int(nout[0][1])
    warn = re.findall(r"WARNING[^\n]*", log)
    if warn:
        out["warnings"] = warn[:3]
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", required=True)
    ap.add_argument("--exclude", default="", help="held-out frame names")
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    cache = json.load(open(args.cache))
    cache["entries"] = [{"name": e["name"], "source": e["source"],
                         "corners": np.asarray(e["corners"], float)}
                        for e in cache["entries"]]
    held = set(x for x in args.exclude.split(",") if x)
    keep = [e["name"] for e in cache["entries"] if e["name"] not in held]

    res = {}
    for key in CONFIGS:
        res[key] = run_one(cache, set(keep), key, args.outdir)
        r = res[key]
        if "error" in r:
            print(f"  {key:13s} FAILED: {r['error'][:120]}")
        else:
            o = (f"{r['outlier_corners']}/{r['total_corners']} corners out"
                 if "outlier_corners" in r else "no outlier stats")
            print(f"  {key:13s} rms {r['rms']} px | {o}")
    json.dump(res, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    main()
