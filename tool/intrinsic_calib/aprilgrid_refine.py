"""AprilGrid corner extraction for the calibration pipeline.

Tag decoding and corner identification are Kalibr's (kalibr_extractor, the
ethz_apriltag2 detector with Kalibr's default options).  What follows is the
part that Kalibr does not do: each tag corner is relocated by a saddle-point
fit in a window scaled to the local corner spacing, and every corner carries
an acceptance decision, so that a frame enters the estimation only through the
corners whose localisation succeeded.

Stage 1 for a partially visible target therefore has two levels:
  corner  the Gaussian-weighted quadratic surface fitted around the corner is
          a saddle (negative Hessian determinant), the iteration converged and
          moved the corner by less than `max_shift` px;
  frame   at least `min_tags` tags keep all four corners.
Frames that fail the frame rule are still returned with their raw Kalibr
corners (source "kalibr") so that they stay visible as rejected images.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from .kalibr_extractor import KalibrAprilgridOptions, detect_aprilgrid_kalibr


@dataclass
class AprilRefineOptions:
    window_frac: float = 0.25    # half-window = window_frac x local corner spacing
    window_min: int = 3
    window_max: int = 10
    sigma_img: float = 1.0       # Gaussian smoothing before the surface fit
    max_shift: float = 1.5       # [px] corner rejected if moved further
    iters: int = 6
    min_tags: int = 4            # frame rule: complete tags required


def _local_spacing(uv: np.ndarray) -> np.ndarray:
    d = np.linalg.norm(uv[:, None, :] - uv[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    return d.min(1)


def _saddle_one(gs: np.ndarray, p: np.ndarray, h: int, iters: int):
    H, W = gs.shape
    q = p.astype(np.float64).copy()
    yy, xx = np.mgrid[-h:h + 1, -h:h + 1].astype(np.float64)
    w = np.exp(-(xx ** 2 + yy ** 2) / (2 * (0.6 * h) ** 2)).ravel()
    A = np.stack([xx.ravel() ** 2, (xx * yy).ravel(), yy.ravel() ** 2,
                  xx.ravel(), yy.ravel(), np.ones(xx.size)], 1)
    Aw = A * w[:, None]
    for _ in range(iters):
        x0, y0 = q
        if not (h + 1 <= x0 < W - h - 2 and h + 1 <= y0 < H - h - 2):
            return q, False
        patch = cv2.getRectSubPix(gs, (2 * h + 1, 2 * h + 1),
                                  (float(x0), float(y0))).astype(np.float64)
        coef, *_ = np.linalg.lstsq(Aw, patch.ravel() * w, rcond=None)
        a, b, c, d, e, _f = coef
        det = 4 * a * c - b * b
        if det >= 0:
            return q, False
        dx = (b * e - 2 * c * d) / det
        dy = (b * d - 2 * a * e) / det
        step = float(np.hypot(dx, dy))
        if step > h:
            return q, False
        q = q + np.array([dx, dy])
        if step < 1e-3:
            break
    return q, True


def refine_saddle(gray: np.ndarray, uv: np.ndarray,
                  opt: Optional[AprilRefineOptions] = None):
    """Returns (refined uv (N,2) float64, accepted (N,) bool, shift (N,) px)."""
    o = opt or AprilRefineOptions()
    gs = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), o.sigma_img)
    sp = _local_spacing(uv)
    out = uv.astype(np.float64).copy()
    ok = np.zeros(len(uv), bool)
    shift = np.full(len(uv), np.inf)
    for k, (p, s) in enumerate(zip(uv, sp)):
        h = int(np.clip(round(o.window_frac * s), o.window_min, o.window_max))
        q, good = _saddle_one(gs, p, h, o.iters)
        if good:
            sh = float(np.linalg.norm(q - p))
            out[k], shift[k] = q, sh
            ok[k] = sh <= o.max_shift
    return out, ok, shift


def detect_aprilgrid_rap(gray: np.ndarray, tag_rows: int, tag_cols: int,
                         tag_size: float, tag_spacing: float,
                         opt: Optional[AprilRefineOptions] = None,
                         kalibr_opt: Optional[KalibrAprilgridOptions] = None
                         ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str, str]:
    """Returns (uv, object points, source, message).

    source "rap":    accepted corners of a frame that passes the frame rule
    source "kalibr": raw Kalibr corners of a frame that fails it (Stage 1 reject)
    (None, None, None, msg) when Kalibr itself rejects the frame."""
    o = opt or AprilRefineOptions()
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    uv, obj, msg = detect_aprilgrid_kalibr(gray, tag_rows, tag_cols, tag_size,
                                           tag_spacing, kalibr_opt)
    if uv is None:
        return None, None, None, msg
    ref, ok, _shift = refine_saddle(gray, uv, o)
    # a tag is complete when its four corners are accepted; corners are
    # identified by their object point, four per tag
    tag_id = _tag_ids(obj, tag_size, tag_spacing, tag_cols)
    complete = [t for t in np.unique(tag_id) if ok[tag_id == t].sum() == 4]
    n_tags = len(np.unique(tag_id))
    if len(complete) < o.min_tags:
        return (uv.astype(np.float32), obj.astype(np.float32), "kalibr",
                f"{msg} refine: {len(complete)}/{n_tags} complete tags < {o.min_tags}.")
    m = ok
    return (ref[m].astype(np.float32), obj[m].astype(np.float32), "rap",
            f"{msg} refine: {int(m.sum())}/{len(m)} corners, {len(complete)}/{n_tags} complete tags.")


def _tag_ids(obj: np.ndarray, tag_size: float, tag_spacing: float, tag_cols: int):
    pitch = (1.0 + tag_spacing) * tag_size
    c = np.floor(obj[:, 0] / pitch + 1e-6).astype(int)
    r = np.floor(obj[:, 1] / pitch + 1e-6).astype(int)
    return r * tag_cols + c
