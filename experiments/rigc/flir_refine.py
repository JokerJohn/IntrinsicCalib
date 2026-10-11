#!/usr/bin/env python3
"""AprilGrid corner refinement and a Stage-1 criterion for the FLIR sessions.

Kalibr refines each tag corner with cornerSubPix on a fixed 5x5 window
(half-size 2), whatever the corner's scale, blur or obliquity.  Candidates:

  kalibr   Kalibr as is
  sp-ad    cornerSubPix, half-window adapted to the local corner spacing
  saddle   saddle-point fit: weighted quadratic surface on the smoothed image,
           re-centred until convergence (Placht/ROCHADE-style), window adapted
           to the corner spacing; returns a per-corner quality (saddle test,
           convergence shift)

Stage 1 analogue ("complete high-accuracy detection" for a partial-view
target): a frame is accepted only if every corner of every decoded tag
passes the saddle test and converges.

Scoring is extractor-independent: train on one session, freeze intrinsics, and
report (a) the cross-session parameter difference |fx_D1 - fx_D2| etc. and
(b) the held-out error on the other session.
"""
from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flir_compare as FC  # noqa: E402


def local_spacing(uv):
    d = np.linalg.norm(uv[:, None, :] - uv[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    return d.min(1)


def refine_subpix_adaptive(g, uv):
    sp = local_spacing(uv)
    out = uv.copy()
    for k, (p, s) in enumerate(zip(uv, sp)):
        h = int(np.clip(round(0.25 * s), 2, 12))
        c = cornerSubPix_one(g, p, h)
        out[k] = c
    return out


def cornerSubPix_one(g, p, h):
    pts = np.array([[p]], np.float32)
    cv2.cornerSubPix(g, pts, (h, h), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
    return pts[0, 0].astype(np.float64)


_BLUR = {}


def saddle_one(gs, p, h, iters=6):
    """Fit I = a x^2 + b xy + c y^2 + d x + e y + f on a Gaussian-weighted
    (2h+1)^2 window of the smoothed image gs; move to the stationary point."""
    H, W = gs.shape
    q = p.copy()
    yy, xx = np.mgrid[-h:h + 1, -h:h + 1].astype(np.float64)
    w = np.exp(-(xx ** 2 + yy ** 2) / (2 * (0.6 * h) ** 2)).ravel()
    A = np.stack([xx.ravel() ** 2, (xx * yy).ravel(), yy.ravel() ** 2,
                  xx.ravel(), yy.ravel(), np.ones(xx.size)], 1)
    Aw = A * w[:, None]
    for _ in range(iters):
        x0, y0 = q
        if not (h + 1 <= x0 < W - h - 2 and h + 1 <= y0 < H - h - 2):
            return q, False, np.inf
        patch = cv2.getRectSubPix(gs, (2 * h + 1, 2 * h + 1), (float(x0), float(y0))).astype(np.float64)
        coef, *_ = np.linalg.lstsq(Aw, patch.ravel() * w, rcond=None)
        a, b, c, d, e, _f = coef
        det = 4 * a * c - b * b
        if det >= 0:                       # not a saddle
            return q, False, np.inf
        dx = (b * e - 2 * c * d) / det
        dy = (b * d - 2 * a * e) / det
        step = np.hypot(dx, dy)
        if step > h:
            return q, False, np.inf
        q = q + np.array([dx, dy])
        if step < 1e-3:
            break
    return q, True, float(np.linalg.norm(q - p))


def refine_saddle(g, uv, return_quality=False):
    key = id(g)
    gs = cv2.GaussianBlur(g.astype(np.float32), (0, 0), 1.0)
    sp = local_spacing(uv)
    out = uv.copy()
    ok = np.zeros(len(uv), bool)
    shift = np.zeros(len(uv))
    for k, (p, s) in enumerate(zip(uv, sp)):
        h = int(np.clip(round(0.25 * s), 3, 10))
        q, good, sh = saddle_one(gs, p, h)
        out[k], ok[k], shift[k] = q, good, sh
    if return_quality:
        return out, ok, shift
    return out


import functools
print = functools.partial(print, flush=True)


def main():
    fr = {d: FC.frames(d) for d in ("D1", "D2")}
    base = {d: FC.extract(d, fr[d]) for d in fr}
    variants = {}
    variants["kalibr"] = base
    variants["sp-ad"] = {d: [dict(o, uv=refine_subpix_adaptive(fr[d][o["i"]], o["uv"])) for o in base[d]]
                         for d in base}
    sad, qual = {}, {}
    for d in base:
        sad[d], qual[d] = [], []
        for o in base[d]:
            uv, ok, sh = refine_saddle(fr[d][o["i"]], o["uv"], True)
            m = ok & (sh < 1.5)
            sad[d].append(dict(o, uv=uv[m] if m.sum() >= 8 else uv, obj=o["obj"][m] if m.sum() >= 8 else o["obj"]))
            qual[d].append(dict(all_ok=bool(m.all()), frac=float(m.mean()), n=len(m)))
    variants["saddle"] = sad
    # Stage 1 analogue on the saddle corners: every corner of every decoded tag passes
    variants["saddle+S1"] = {d: [o for o, q in zip(sad[d], qual[d]) if q["all_ok"]] for d in sad}
    for d in qual:
        print(f"{d}: saddle Stage-1 accepts {sum(q['all_ok'] for q in qual[d])}/{len(qual[d])} frames")

    report = {}
    print(f"\n{'variant':10s} {'method':7s} | {'|dfx|':>6s} {'|dcx|':>6s} {'|dcy|':>6s} | "
          f"held-out mean/median  D2->D1        D1->D2   | orders")
    for name, obs in variants.items():
        for meth in ("M2-all", "RAP"):
            P, held, orders = {}, {}, []
            for train, test in (("D2", "D1"), ("D1", "D2")):
                if meth == "M2-all":
                    K, Dd, _ = FC.fit(obs[train][::3], True)
                else:
                    K, Dd, info = FC.rap(obs[train][::3]); orders.append(f"{info['order']}({info['kept']}/{info['n']})")
                P[train] = (K, Dd)
                # score on the Kalibr corners of the other session: same yardstick for every variant
                e = np.concatenate(FC.view_errors(K, Dd, base[test]))
                held[f"{train}->{test}"] = (float(e.mean()), float(np.median(e)))
            dK = np.abs(P["D1"][0] - P["D2"][0])
            report[f"{name}/{meth}"] = dict(dfx=dK[0, 0], dcx=dK[0, 2], dcy=dK[1, 2], held=held, orders=orders)
            h1, h2 = held["D2->D1"], held["D1->D2"]
            print(f"{name:10s} {meth:7s} | {dK[0,0]:6.2f} {dK[0,2]:6.2f} {dK[1,2]:6.2f} | "
                  f"{h1[0]:.3f}/{h1[1]:.3f}   {h2[0]:.3f}/{h2[1]:.3f} | {' '.join(orders)}")
    kd = {d: FC.KD(*FC.KALIBR[d]) for d in ("D1", "D2")}
    dK = np.abs(kd["D1"][0] - kd["D2"][0])
    print(f"{'Kalibr':10s} {'pub.':7s} | {dK[0,0]:6.2f} {dK[0,2]:6.2f} {dK[1,2]:6.2f} |")
    json.dump(report, open(os.path.join(FC.OUT, "flir_refine.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
