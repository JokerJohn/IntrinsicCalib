"""Kalibr's calibration-target corner extractors, with Kalibr's default options.

* AprilGrid: Kalibr's own C++ (ethz_apriltag2 tag detector + the
  computeObservation logic of GridCalibrationTargetAprilgrid), compiled on
  first use into a small shared library and called through ctypes.  Corners
  are the four outer corners of every decoded tag, refined by cornerSubPix in
  the raw image; no grid completion and no undistortion, so the extracted
  corners do not depend on any intrinsics guess.
* Checkerboard: GridCalibrationTargetCheckerboard::computeObservation, which
  is plain OpenCV (findChessboardCorners + cornerSubPix) and therefore ported
  directly to Python.

Both follow kalibr master, commit 1f60227 (2024-03-08).
"""
from __future__ import annotations

import ctypes
import os
import subprocess
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, "kalibr_aprilgrid.cpp")
_TP = os.path.join(_HERE, "third_party", "ethz_apriltag2")
_LIB = os.path.join(_HERE, "libkalibr_aprilgrid.so")
_lock = threading.Lock()
_lib = None


@dataclass
class KalibrAprilgridOptions:
    """AprilgridOptions() defaults of Kalibr."""
    do_subpix_refinement: bool = True
    max_subpix_displacement2: float = 1.5   # [px^2]
    min_tags_for_valid_obs: int = 4
    min_border_distance: float = 4.0         # [px]
    black_tag_border: int = 2


@dataclass
class KalibrCheckerboardOptions:
    """CheckerboardOptions() defaults of Kalibr."""
    use_adaptive_threshold: bool = True
    normalize_image: bool = True
    perform_fast_check: bool = True
    filter_quads: bool = False
    do_subpixel_refinement: bool = True
    window_width: int = 11   # cornerSubPix half-window, cv::Size(w, w)


def _sources():
    srcs = [_SRC] + [os.path.join(_TP, "src", f)
                     for f in sorted(os.listdir(os.path.join(_TP, "src")))
                     if f.endswith(".cc")]
    return srcs


def _build() -> None:
    srcs = _sources()
    if os.path.exists(_LIB) and all(os.path.getmtime(_LIB) >= os.path.getmtime(s)
                                    for s in srcs):
        return
    cflags = subprocess.check_output(["pkg-config", "--cflags", "opencv4", "eigen3"],
                                     text=True).split()
    libs = subprocess.check_output(["pkg-config", "--libs", "opencv4"], text=True).split()
    keep = {"-lopencv_core", "-lopencv_imgproc", "-lopencv_calib3d", "-lopencv_features2d", "-lopencv_flann"}
    libs = [x for x in libs if not x.startswith("-l") or x in keep]
    cmd = (["g++", "-O3", "-fPIC", "-shared", "-std=c++14", "-w",
            "-I" + os.path.join(_TP, "include")] + cflags + srcs
           + ["-o", _LIB + ".tmp"] + libs)
    subprocess.check_call(cmd)
    os.replace(_LIB + ".tmp", _LIB)


def _load():
    global _lib
    with _lock:
        if _lib is None:
            _build()
            lib = ctypes.CDLL(_LIB)
            fn = lib.kalibr_aprilgrid_detect
            fn.restype = ctypes.c_int
            fn.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
                           ctypes.c_int, ctypes.c_int, ctypes.c_double,
                           ctypes.c_int, ctypes.c_double, ctypes.c_int,
                           ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p,
                           ctypes.POINTER(ctypes.c_int)]
            _lib = lib
    return _lib


def aprilgrid_object_points_kalibr(tag_rows: int, tag_cols: int, tag_size: float,
                                   tag_spacing: float) -> np.ndarray:
    """GridCalibrationTargetAprilgrid::createGridPoints, (4*rows*cols, 3) [m]."""
    rows, cols = 2 * tag_rows, 2 * tag_cols
    pts = np.zeros((rows * cols, 3), np.float64)
    for r in range(rows):
        for c in range(cols):
            pts[r * cols + c, 0] = (c // 2) * (1 + tag_spacing) * tag_size + (c % 2) * tag_size
            pts[r * cols + c, 1] = (r // 2) * (1 + tag_spacing) * tag_size + (r % 2) * tag_size
    return pts


def detect_aprilgrid_kalibr(gray: np.ndarray, tag_rows: int, tag_cols: int,
                            tag_size: float, tag_spacing: float,
                            options: Optional[KalibrAprilgridOptions] = None
                            ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
    """Returns (uv (M,2) float32, object points (M,3) float32, message) for the
    observed corners, or (None, None, message) when Kalibr would reject the
    frame.  The full grid has 4*tag_rows*tag_cols corners."""
    o = options or KalibrAprilgridOptions()
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    gray = np.ascontiguousarray(gray, dtype=np.uint8)
    n = 4 * tag_rows * tag_cols
    pts = np.zeros((n, 2), np.float64)
    obs = np.zeros(n, np.uint8)
    ntags = ctypes.c_int(0)
    rc = _load().kalibr_aprilgrid_detect(
        gray.ctypes.data, gray.shape[1], gray.shape[0], tag_rows, tag_cols,
        o.max_subpix_displacement2, o.min_tags_for_valid_obs,
        o.min_border_distance, o.black_tag_border, int(o.do_subpix_refinement),
        pts.ctypes.data, obs.ctypes.data, ctypes.byref(ntags))
    total = tag_rows * tag_cols
    if rc == -2:
        return None, None, "Kalibr AprilGrid: duplicate tag ids (foreign tag in view)."
    if rc != 1:
        return None, None, f"Kalibr AprilGrid: {ntags.value}/{total} tags (< {o.min_tags_for_valid_obs})."
    m = obs.astype(bool)
    obj = aprilgrid_object_points_kalibr(tag_rows, tag_cols, tag_size, tag_spacing)
    return (pts[m].astype(np.float32), obj[m].astype(np.float32),
            f"Kalibr AprilGrid: {ntags.value}/{total} tags, {int(m.sum())}/{n} corners.")


def detect_checkerboard_kalibr(gray: np.ndarray, cols: int, rows: int,
                               options: Optional[KalibrCheckerboardOptions] = None
                               ) -> Optional[np.ndarray]:
    """GridCalibrationTargetCheckerboard::computeObservation: (cols*rows, 2)
    float32 corners in OpenCV order, or None when the grid is not found."""
    o = options or KalibrCheckerboardOptions()
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    flags = 0
    if o.perform_fast_check:
        flags += cv2.CALIB_CB_FAST_CHECK
    if o.use_adaptive_threshold:
        flags += cv2.CALIB_CB_ADAPTIVE_THRESH
    if o.normalize_image:
        flags += cv2.CALIB_CB_NORMALIZE_IMAGE
    if o.filter_quads:
        flags += cv2.CALIB_CB_FILTER_QUADS
    ok, centers = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
    if not ok:
        return None
    centers = centers.astype(np.float32)
    if o.do_subpixel_refinement:
        centers = cv2.cornerSubPix(
            gray, centers, (o.window_width, o.window_width), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
    return centers.reshape(-1, 2).astype(np.float32)
