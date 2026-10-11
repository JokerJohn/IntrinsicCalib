"""AprilGrid corner detection with full-grid completion (Kalibr layout)."""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

from .models import BoardConfig, CameraIntrinsics


def _clip_quad(quad: Sequence[Sequence[float]], width: int, height: int) -> np.ndarray:
    pts = np.asarray(quad, dtype=np.float32).reshape(-1, 2)
    if pts.shape != (4, 2):
        raise ValueError("Image board ROI must contain exactly four points.")
    pts[:, 0] = np.clip(pts[:, 0], 0, width - 1)
    pts[:, 1] = np.clip(pts[:, 1], 0, height - 1)
    return pts


def aprilgrid_object_points(board: BoardConfig) -> Dict[int, np.ndarray]:
    """Kalibr aprilgrid layout: tag id = row * cols + col (start id 0), each
    tag contributes its 4 outer corners. tag_spacing is the gap/tag-size
    RATIO (kalibr tagSpacing). Returns {tag_id: (4,3) corner points, metres},
    corner order matching cv2.aruco detections (TL, TR, BR, BL in tag frame
    mapped to the grid plane with +x right, +y up per kalibr convention)."""
    size = float(board.tag_size)
    step = size * (1.0 + float(board.tag_spacing))
    rows = int(board.tag_rows)
    points: Dict[int, np.ndarray] = {}
    # Kalibr convention, verified empirically on the real 3x3 target (a full
    # 9-tag frame ranks this hypothesis at 8 px homography residual vs 55 px
    # for the next best): tag id 0 sits at the BOTTOM-left (ids increase
    # upward row-major in the physical y-up frame), and cv2.aruco's corner
    # order maps to the grid with a cyclic shift of 2. Expressed in an
    # image-like y-down grid frame below (row 0 at the top).
    for row in range(rows):
        for col in range(int(board.tag_cols)):
            tag_id = row * int(board.tag_cols) + col
            x0 = col * step
            y0 = (rows - 1 - row) * step
            base = [(x0, y0), (x0 + size, y0),
                    (x0 + size, y0 + size), (x0, y0 + size)]
            base = base[2:] + base[:2]
            points[tag_id] = np.array([[x, y, 0.0] for x, y in base], dtype=np.float32)
    return points


def _radtan_distort_points(pts_und: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """Map undistorted pixel coords back to raw distorted pixels (exact
    forward radtan model; inverse of cv2.undistort's mapping)."""
    k1, k2, p1, p2 = float(D[0]), float(D[1]), float(D[2]), float(D[3])
    k3 = float(D[4]) if len(D) > 4 else 0.0
    x = (pts_und[:, 0] - K[0, 2]) / K[0, 0]
    y = (pts_und[:, 1] - K[1, 2]) / K[1, 1]
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.column_stack((xd * K[0, 0] + K[0, 2], yd * K[1, 1] + K[1, 2]))


def detect_aprilgrid_corners(
    image_path: Path,
    board: BoardConfig,
    image_roi_quad: Optional[Sequence[Sequence[float]]] = None,
    camera: Optional["CameraIntrinsics"] = None,
    complete_grid: bool = True,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
    """Kalibr AprilGrid (36h11) detection.

    Kalibr grids join the tags' black borders to the corner squares, which
    breaks plain quad detectors (cv2.aruco recovers only a few tags per
    frame; apriltag3 none). So, like kalibr itself, tags are used only for
    ID/initialization: the image is undistorted (known intrinsics if given,
    else a small k1-guess sweep), a plane homography is fitted to the >= 2
    detected tags, ALL grid corners are predicted through it, refined with
    cornerSubPix on the checkerboard-like junctions, and mapped back to raw
    distorted pixel coordinates through the exact forward model.
    Returns (corners_uv raw (N,2), object_points (N,3), message).
    """
    image = cv2.imread(str(image_path)) if not isinstance(image_path, np.ndarray) else image_path
    if image is None:
        return None, None, f"Could not read image: {image_path}"
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if image_roi_quad:
        quad = _clip_quad(image_roi_quad, gray.shape[1], gray.shape[0]).astype(np.int32)
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [quad], 255)
        gray = np.where(mask > 0, gray, np.uint8(255)).astype(np.uint8)
    h, w = gray.shape

    if camera is not None:
        guesses = [(camera.camera_matrix().astype(float),
                    np.array([camera.k1, camera.k2, camera.p1, camera.p2, camera.k3], float))]
    else:
        f = 0.33 * w
        base_k = np.array([[f, 0.0, w / 2.0], [0.0, f, h / 2.0], [0.0, 0.0, 1.0]])
        guesses = [(base_k, np.array([k1g, 0.06 if k1g < -0.2 else 0.0, 0.0, 0.0, 0.0]))
                   for k1g in (0.0, -0.15, -0.25, -0.32)]

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.polygonalApproxAccuracyRate = 0.08
    params.maxErroneousBitsInBorderRate = 0.6
    params.errorCorrectionRate = 1.0
    # Bit-decode robustness is the real bottleneck on blurred kalibr targets:
    # with the default 4 px/cell sampling and 0.13 ignored margin, motion blur
    # bleeds tag borders into the bit cells and the quad is rejected at decode
    # (parameter sweep on 40 real fisheye frames: defaults 114 tags / 12 empty
    # frames -> these values 318 tags / 0 empty frames).
    params.perspectiveRemovePixelPerCell = 8
    params.perspectiveRemoveIgnoredMarginPerCell = 0.30
    params.adaptiveThreshWinSizeMax = 93
    params.adaptiveThreshWinSizeStep = 4
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    layout = aprilgrid_object_points(board)

    best = None  # (n_tags, und, K, D, matches)
    for K, D in guesses:
        und = cv2.undistort(gray, K, D)
        # Scale escalation: an obliquely viewed tag has too few pixels per bit
        # cell to decode at native resolution (observed on the fisheye tail:
        # 0 tags at 1.0x, 2-4 tags at 1.5-2.0x). Upsampling costs nothing on
        # the frames that already decode because we stop at the first scale
        # that yields enough tags.
        matches = []
        for scale in (1.0, 1.5, 2.0):
            probe = und if scale == 1.0 else cv2.resize(
                und, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            corners, ids, _ = detector.detectMarkers(probe)
            found = []
            if ids is not None:
                for tag_corners, tag_id in zip(corners, ids.ravel()):
                    obj = layout.get(int(tag_id))
                    if obj is not None:
                        found.append((np.asarray(tag_corners, np.float32).reshape(4, 2) / scale,
                                      obj))
            if len(found) > len(matches):
                matches = found
            if len(matches) >= 3:
                break
        if best is None or len(matches) > best[0]:
            best = (len(matches), und, K, D, matches)
        if camera is not None and len(matches) >= 3:
            break
    n_tags, und, K, D, matches = best
    total = int(board.tag_rows) * int(board.tag_cols)
    if not complete_grid:
        # Tags-only mode: return just the id-verified tag corners (mapped back
        # to raw pixels). Used as the robust first stage when no intrinsics
        # are known yet: without true undistortion the grid-completion
        # homography can snap sub-pixel refinement onto neighbouring
        # junctions, so completion is deferred to a second pass.
        if n_tags < 1:
            return None, None, "AprilGrid: no tags decoded."
        uv_und = np.vstack([m[0] for m in matches])
        obj = np.vstack([m[1] for m in matches])
        uv_raw = _radtan_distort_points(uv_und.astype(np.float64), K, D)
        return (uv_raw.astype(np.float32), obj.astype(np.float32),
                f"AprilGrid: {n_tags}/{total} tags (tag corners only).")
    min_tags = 2 if camera is not None else 3
    if n_tags < min_tags:
        return None, None, (
            f"AprilGrid: only {n_tags} tag(s) decoded (need >= {min_tags} for a "
            "reliable grid homography). Board too oblique/blurred in this frame."
        )
    uv_seen = np.vstack([m[0] for m in matches])
    xy_seen = np.vstack([m[1][:, :2] for m in matches])
    H, _ = cv2.findHomography(xy_seen, uv_seen, 0)
    if H is None:
        return None, None, "AprilGrid: homography estimation failed."
    all_obj = np.vstack([layout[i] for i in sorted(layout)])
    margin = 8
    # Corner-response map: a refined point is accepted only if it sits on a
    # real saddle. Reference level = the decoded tag corners themselves
    # (known-true saddles). Without this, predictions that land in blank
    # white gaps have zero gradient, never move in cornerSubPix, and would
    # pass any shift gate while being pure ghosts.
    response = cv2.cornerMinEigenVal(und, 5)
    seen_ij = np.clip(np.rint(uv_seen).astype(int), 0,
                      [und.shape[1] - 1, und.shape[0] - 1])
    ref_resp = float(np.median(response[seen_ij[:, 1], seen_ij[:, 0]]))

    def _refine_round(h_mat):
        pred_r = cv2.perspectiveTransform(
            all_obj[:, :2].reshape(-1, 1, 2).astype(np.float64), h_mat).reshape(-1, 2)
        inside_r = ((pred_r[:, 0] > margin) & (pred_r[:, 0] < und.shape[1] - margin)
                    & (pred_r[:, 1] > margin) & (pred_r[:, 1] < und.shape[0] - margin))
        refined_r = pred_r.astype(np.float32).reshape(-1, 1, 2).copy()
        idx_r = np.where(inside_r)[0]
        if len(idx_r) == 0:
            return pred_r, inside_r, refined_r
        sub_r = refined_r[idx_r]
        cv2.cornerSubPix(und, sub_r, (8, 8), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3))
        refined_r[idx_r] = sub_r
        return pred_r, inside_r, refined_r

    # Iterative expansion: the initial homography extrapolates from only the
    # decoded tags, so far-away junctions land off-target and get gated out.
    # Each round refits the homography on the refined inliers and re-predicts,
    # recovering the corners the previous round rejected.
    keep = None
    refined = None
    for _round in range(3):
        pred, inside, refined = _refine_round(H)
        ref2 = refined.reshape(-1, 2)
        shift = np.linalg.norm(ref2 - pred, axis=1)
        ij = np.clip(np.rint(ref2).astype(int), 0,
                     [und.shape[1] - 1, und.shape[0] - 1])
        resp_ok = response[ij[:, 1], ij[:, 0]] > 0.15 * ref_resp
        # Wide shift gate (junction spacing ~16 px at 1 m, so 10 px cannot
        # reach a neighbouring saddle) - large recoveries are exactly the
        # point of the expansion; ghosts are killed by resp_ok instead.
        keep = inside & (shift < 10.0) & resp_ok
        if keep.sum() < 12:
            break
        ref_kept = refined.reshape(-1, 2)[keep]
        H2, inl = cv2.findHomography(all_obj[keep][:, :2], ref_kept, cv2.RANSAC, 2.5)
        if H2 is None or inl is None or int(inl.sum()) < 12:
            break
        inl = inl.ravel().astype(bool)
        keep_idx = np.where(keep)[0][inl]
        keep = np.zeros_like(keep)
        keep[keep_idx] = True
        H = H2
    if keep is None or keep.sum() < 12:
        return None, None, "AprilGrid: corner refinement rejected most junctions."
    uv_raw = _radtan_distort_points(refined.reshape(-1, 2)[keep], K, D)
    ok_bounds = ((uv_raw[:, 0] >= 0) & (uv_raw[:, 0] < w)
                 & (uv_raw[:, 1] >= 0) & (uv_raw[:, 1] < h))
    uv_raw = uv_raw[ok_bounds]
    obj_keep = all_obj[keep][ok_bounds]
    return (uv_raw.astype(np.float32), obj_keep.astype(np.float32),
            f"AprilGrid: {n_tags}/{total} tags decoded, "
            f"{len(uv_raw)}/{total * 4} corners via grid homography.")


    # ===== END CHANGE: project board circles as image ellipses =====


