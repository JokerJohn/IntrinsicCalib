from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
try:
    import open3d as o3d  # only needed by the LiDAR feature functions below
except ImportError:  # pragma: no cover
    o3d = None

from .models import BOARD_TYPE_APRILGRID, BoardConfig, CameraIntrinsics


@dataclass
class ImageDetection:
    centers_cam: List[List[float]] = field(default_factory=list)
    centers_uv: List[List[float]] = field(default_factory=list)
    circles_uv: List[Dict[str, float]] = field(default_factory=list)
    marker_ids: List[int] = field(default_factory=list)
    markers: List[Dict[str, object]] = field(default_factory=list)
    debug_image: Optional[np.ndarray] = None
    message: str = ""
    ok: bool = False


@dataclass
class LidarDetection:
    centers_lidar: List[List[float]] = field(default_factory=list)
    # FS mode only: refined centers BEFORE the rectangle projection. The
    # projected centers form a perfect rectangle and would trivially match the
    # camera-side rectangle (RMSE ~ 0), so geometric cross-checks must use
    # these instead.
    centers_lidar_prerect: List[List[float]] = field(default_factory=list)
    holes: List[Dict[str, object]] = field(default_factory=list)
    active_roi: Dict[str, float] = field(default_factory=dict)
    filtered_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    plane_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    edge_points_aligned: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    debug_image: Optional[np.ndarray] = None
    message: str = ""
    ok: bool = False


@dataclass
class FeatureDetectionSettings:
    image_hole_search_scale: float = 1.35
    image_hole_min_support_pixels: int = 40
    image_hole_max_fit_error_px: float = 8.0
    image_hole_max_center_drift_scale: float = 0.25
    image_hole_max_axis_error: float = 0.25
    min_aruco_markers: int = 3
    lidar_voxel_size: float = 0.005
    lidar_plane_distance_threshold: float = 0.012
    lidar_ransac_iterations: int = 300
    lidar_max_points: int = 180000
    lidar_circle_radius_min_scale: float = 0.55
    lidar_circle_radius_max_scale: float = 1.55
    lidar_circle_pattern_max_score: float = 3.0
    lidar_ring_support_abs_threshold: float = 0.015
    lidar_ring_support_radius_scale: float = 0.20
    lidar_min_hole_support_points: int = 12
    # "mid360" keeps the original raster-circle pipeline untouched; "fs" appends
    # an annulus-based refinement tuned for area-array lidars whose hole rims
    # are eroded by mixed-pixel returns (see _refine_fs_holes).
    lidar_type: str = "fs"
    fs_intensity_min: float = 30.0
    # Geometric priors, toggleable from the GUI (Point Cloud View):
    #  - use_circle_prior: score candidate holes by |radius - circle_radius| and
    #    let the FS annulus refinement fit at the nominal radius.
    #  - use_rect_template: require the 4 selected holes to fit the known
    #    delta_width_circles x delta_height_circles rectangle (hard gate during
    #    candidate selection) and project the final centers onto it.
    use_circle_prior: bool = True
    use_rect_template: bool = True
    # Ablation control for the radius prior. 0.0 keeps the published behaviour:
    # the annulus refinement fits every center at the fixed nominal radius. A
    # positive value instead estimates center AND radius per hole and clips the
    # radius to [circle_radius - tol, circle_radius + tol], which reproduces the
    # way prior four-hole pipelines consume the same CAD radius, as a tolerance
    # band around a free circle model rather than as a hard constraint.
    radius_tolerance_m: float = 0.0
    # Uncertainty-weighted solve (WLS): per-hole covariances from the camera
    # board-pose propagation and the LiDAR range/incidence model replace the
    # equal-weight Kabsch in Solve/Estimate/Optimize when enabled.
    use_wls_weights: bool = False
    wls_sigma_rot_deg: float = 0.1   # shared board-pose rotation noise
    wls_cam_st_mm: float = 1.0       # shared board-pose translation noise
    wls_sd_mm: float = 4.0           # s0: LiDAR base edge noise (paper formula 14)
    wls_sw_mrad: float = 1.0         # s1 x 1e-3: range-growth coefficient


def sort_pattern_centers(points: Sequence[Sequence[float]], axis_mode: str = "camera") -> List[List[float]]:
    pts = np.asarray(points, dtype=float).reshape((-1, 3))
    if len(pts) != 4:
        return pts.tolist()
    work = pts.copy()
    if axis_mode == "lidar":
        work = np.column_stack((-pts[:, 1], -pts[:, 2], pts[:, 0]))

    center = work.mean(axis=0)
    rel = work - center
    angles = np.arctan2(rel[:, 1], rel[:, 0])
    order = np.argsort(angles)
    sorted_work = work[order].copy()
    sorted_original = pts[order].copy()
    v01 = sorted_work[1, :2] - sorted_work[0, :2]
    v12 = sorted_work[2, :2] - sorted_work[1, :2]
    if np.cross(np.r_[v01, 0.0], np.r_[v12, 0.0])[2] > 0:
        sorted_original[[1, 3]] = sorted_original[[3, 1]]
    return sorted_original.tolist()


def _board_geometry(board: BoardConfig) -> Tuple[List[np.ndarray], np.ndarray, np.ndarray]:
    board_corners: List[np.ndarray] = []
    board_circle_centers: List[np.ndarray] = []
    width = board.delta_width_qr_center
    height = board.delta_height_qr_center
    circle_width = board.delta_width_circles / 2.0
    circle_height = board.delta_height_circles / 2.0
    for i in range(4):
        x_qr_center = -1 if (i % 3) == 0 else 1
        y_qr_center = 1 if i < 2 else -1
        x_center = x_qr_center * width
        y_center = y_qr_center * height
        board_circle_centers.append(np.array([x_qr_center * circle_width, y_qr_center * circle_height, 0.0]))
        corners = []
        for j in range(4):
            x_qr = -1 if (j % 3) == 0 else 1
            y_qr = 1 if j < 2 else -1
            corners.append([x_center + x_qr * board.marker_size / 2.0, y_center + y_qr * board.marker_size / 2.0, 0.0])
        board_corners.append(np.asarray(corners, dtype=np.float32))
    ids = np.asarray([1, 2, 4, 3], dtype=np.int32)
    return board_corners, ids, np.asarray(board_circle_centers, dtype=np.float64)


def _aruco_detect(image: np.ndarray, dictionary: cv2.aruco_Dictionary) -> Tuple[List[np.ndarray], np.ndarray]:
    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(image)
        return corners, np.asarray([], dtype=np.int32) if ids is None else ids.reshape(-1)

    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    return corners, np.asarray([], dtype=np.int32) if ids is None else ids.reshape(-1)


def _clip_quad(quad: Sequence[Sequence[float]], width: int, height: int) -> np.ndarray:
    pts = np.asarray(quad, dtype=np.float32).reshape(-1, 2)
    if pts.shape != (4, 2):
        raise ValueError("Image board ROI must contain exactly four points.")
    pts[:, 0] = np.clip(pts[:, 0], 0, width - 1)
    pts[:, 1] = np.clip(pts[:, 1], 0, height - 1)
    return pts


def detect_board_corners(image_path, board: BoardConfig,
                        image_roi_quad=None, camera=None):
    """Unified image-side board detection for the non-four-hole targets.

    Returns (corners_uv (N,2), object_points (N,3) in the board frame, message)
    for BOTH checkerboard and aprilgrid, so callers (batch detect, readiness,
    plane-based extrinsic, ROI projection) never branch on the board type.
    """
    if board.board_type == BOARD_TYPE_APRILGRID:
        return detect_aprilgrid_corners(image_path, board, image_roi_quad, camera=camera)
    corners, pattern, msg = detect_checkerboard_corners(image_path, board, image_roi_quad)
    if corners is None:
        return None, None, msg
    cols, rows = int(pattern[0]), int(pattern[1])
    square = float(board.square_size) or 0.05
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return corners.astype(np.float32), (grid * square).astype(np.float32), msg


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


def detect_checkerboard_corners(
    image_path: Path,
    board: BoardConfig,
    image_roi_quad: Optional[Sequence[Sequence[float]]] = None,
) -> Tuple[Optional[np.ndarray], Tuple[int, int], str]:
    """Detect checkerboard interior corners for the annotation-only aid.

    Returns (corners_uv (N,2) or None, pattern_size, message). The board's
    checkerboard_cols/rows are treated as interior-corner counts; the squares
    interpretation (counts - 1) and the transposed layout are tried as
    fallbacks so a loosely specified board_config still detects. When an image
    ROI quad is given, the search is restricted to it to avoid background
    matches. Nothing here is persisted; the caller renders it as an overlay.
    """
    image = cv2.imread(str(image_path))
    if image is None:
        return None, (0, 0), f"Could not read image: {image_path}"
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    search = gray
    if image_roi_quad:
        quad = _clip_quad(image_roi_quad, gray.shape[1], gray.shape[0]).astype(np.int32)
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [quad], 255)
        # Blank the outside to white so it reads as a quiet-zone border.
        search = np.where(mask > 0, gray, np.uint8(255)).astype(np.uint8)

    cols, rows = int(board.checkerboard_cols), int(board.checkerboard_rows)
    raw = [(cols, rows), (cols - 1, rows - 1)]
    patterns: List[Tuple[int, int]] = []
    for a, b in raw:
        for pat in ((a, b), (b, a)):
            if pat[0] >= 3 and pat[1] >= 3 and pat not in patterns:
                patterns.append(pat)
    if not patterns:
        return None, (0, 0), "Set checkerboard_cols/checkerboard_rows in board_config.yaml first."

    sb_flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    for pat in patterns:
        found, corners = False, None
        try:
            found, corners = cv2.findChessboardCornersSB(search, pat, flags=sb_flags)
        except Exception:
            found = False
        if not found:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCorners(search, pat, flags=flags)
        if found and corners is not None:
            return corners.reshape(-1, 2).astype(float), pat, f"Detected {pat[0]}x{pat[1]} corners."
    return None, (0, 0), f"Checkerboard not found (tried {patterns})."


def _mask_image_to_quad(image: np.ndarray, quad: Optional[Sequence[Sequence[float]]]) -> np.ndarray:
    if not quad:
        return image
    h, w = image.shape[:2]
    pts = _clip_quad(quad, w, h).astype(np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(mask, pts, 255)
    masked = np.zeros_like(image)
    masked[mask > 0] = image[mask > 0]
    return masked


def _draw_board_roi(debug: np.ndarray, quad: Optional[Sequence[Sequence[float]]]) -> None:
    if not quad:
        return
    h, w = debug.shape[:2]
    pts = _clip_quad(quad, w, h).astype(np.int32)
    dark = debug.copy()
    cv2.fillConvexPoly(dark, pts, (0, 0, 0))
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(mask, pts, 255)
    outside = mask == 0
    debug[outside] = cv2.addWeighted(debug, 0.25, np.zeros_like(debug), 0.75, 0)[outside]
    cv2.polylines(debug, [pts], True, (0, 220, 255), 3, cv2.LINE_AA)
    for idx, point in enumerate(pts, start=1):
        cv2.circle(debug, tuple(point), 5, (0, 220, 255), -1, cv2.LINE_AA)
        cv2.putText(debug, f"ROI {idx}", tuple(point + np.array([7, -7])), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)


def _marker_records(corners: Sequence[np.ndarray], ids: np.ndarray) -> List[Dict[str, object]]:
    records: List[Dict[str, object]] = []
    for marker_corners, marker_id in zip(corners, ids):
        pts = np.asarray(marker_corners, dtype=float).reshape(4, 2)
        records.append({"id": int(marker_id), "corners": pts.tolist()})
    return records


def _sort_payload_by_pattern(points: np.ndarray, payload: Sequence[np.ndarray], axis_mode: str) -> Tuple[np.ndarray, List[np.ndarray]]:
    sorted_points = np.asarray(sort_pattern_centers(points, axis_mode), dtype=float)
    used: set[int] = set()
    sorted_payload: List[np.ndarray] = []
    for point in sorted_points:
        order = np.argsort(np.linalg.norm(points - point.reshape(1, 3), axis=1))
        match = next(int(idx) for idx in order if int(idx) not in used)
        used.add(match)
        sorted_payload.append(np.asarray(payload[match], dtype=float))
    return sorted_points, sorted_payload


def _project_hole_outline(
    center_board: np.ndarray,
    radius: float,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera: CameraIntrinsics,
) -> np.ndarray:
    # ===== BEGIN CHANGE: project board circles as image ellipses =====
    # A circle on a tilted calibration board is not generally a circle in the
    # image. Sampling the 3D board circle and projecting the samples preserves
    # perspective and distortion, so the overlay matches what the camera sees.
    angles = np.linspace(0.0, 2.0 * np.pi, 96, endpoint=False)
    circle_board = np.column_stack(
        (
            center_board[0] + radius * np.cos(angles),
            center_board[1] + radius * np.sin(angles),
            np.full(len(angles), center_board[2]),
        )
    ).astype(np.float32)
    projected, _ = cv2.projectPoints(circle_board.reshape(-1, 1, 3), rvec, tvec, camera.camera_matrix(), camera.dist_coeffs())
    return projected.reshape(-1, 2)
    # ===== END CHANGE: project board circles as image ellipses =====


def _draw_explicit_markers(debug: np.ndarray, markers: Sequence[Dict[str, object]]) -> None:
    for marker in markers:
        pts = np.asarray(marker.get("corners", []), dtype=np.int32).reshape(-1, 2)
        if pts.shape != (4, 2):
            continue
        marker_id = marker.get("id", "?")
        cv2.polylines(debug, [pts], True, (0, 255, 0), 3, cv2.LINE_AA)
        center = np.mean(pts, axis=0).astype(int)
        cv2.putText(
            debug,
            f"ID {marker_id}",
            (int(center[0]) - 24, int(center[1]) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        for corner_idx, point in enumerate(pts, start=1):
            cv2.circle(debug, tuple(point), 4, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(
                debug,
                str(corner_idx),
                (int(point[0]) + 4, int(point[1]) + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )


def _ellipse_contour(center: np.ndarray, axes: Tuple[float, float], angle_deg: float) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * np.pi, 96, endpoint=False)
    half_axes = np.asarray(axes, dtype=float) / 2.0
    points = np.column_stack((half_axes[0] * np.cos(angles), half_axes[1] * np.sin(angles)))
    theta = np.deg2rad(angle_deg)
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], dtype=float)
    return points @ rot.T + center.reshape(1, 2)


def _ellipse_fit_error_px(contour: np.ndarray, center: np.ndarray, axes: Tuple[float, float], angle_deg: float) -> float:
    pts = np.asarray(contour, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return float("nan")
    half_axes = np.maximum(np.asarray(axes, dtype=float) / 2.0, 1e-6)
    theta = -np.deg2rad(angle_deg)
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], dtype=float)
    local = (pts - center.reshape(1, 2)) @ rot.T
    normalized_radius = np.sqrt((local[:, 0] / half_axes[0]) ** 2 + (local[:, 1] / half_axes[1]) ** 2)
    return float(np.mean(np.abs(normalized_radius - 1.0)) * np.mean(half_axes))


def _fit_ellipse_axes(points_uv: np.ndarray, fallback_diameter: float) -> np.ndarray:
    pts = np.asarray(points_uv, dtype=np.float32).reshape(-1, 1, 2)
    if len(pts) < 5:
        return np.asarray([fallback_diameter, fallback_diameter], dtype=float)
    try:
        axes = np.asarray(cv2.fitEllipse(pts)[1], dtype=float)
    except cv2.error:
        axes = np.asarray([fallback_diameter, fallback_diameter], dtype=float)
    return np.sort(np.maximum(axes, 1e-6))


def _fit_local_hole_ellipse(
    image: np.ndarray,
    center_uv: np.ndarray,
    radius_px: float,
    expected_contour_uv: np.ndarray,
    settings: FeatureDetectionSettings,
) -> Dict[str, object]:
    h, w = image.shape[:2]
    search_radius = int(round(max(12.0, radius_px * settings.image_hole_search_scale)))
    cx, cy = float(center_uv[0]), float(center_uv[1])
    x0 = max(0, int(np.floor(cx - search_radius)))
    y0 = max(0, int(np.floor(cy - search_radius)))
    x1 = min(w, int(np.ceil(cx + search_radius)))
    y1 = min(h, int(np.ceil(cy + search_radius)))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return {"status": "Too Small", "support_count": 0, "fit_error_px": float("nan")}

    crop = image[y0:y1, x0:x1]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return {"status": "Not Found", "support_count": 0, "fit_error_px": float("nan")}

    expected_area = np.pi * max(radius_px, 1.0) ** 2
    expected = np.array([cx - x0, cy - y0], dtype=float)
    best_contour: Optional[np.ndarray] = None
    best_score = float("inf")
    for contour in contours:
        if len(contour) < 5:
            continue
        area = float(abs(cv2.contourArea(contour)))
        if area < max(20.0, expected_area * 0.10) or area > expected_area * 3.5:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-9:
            continue
        centroid = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=float)
        center_error = float(np.linalg.norm(centroid - expected))
        area_error = abs(area - expected_area) / max(expected_area, 1.0)
        score = center_error / max(search_radius, 1.0) + area_error
        if score < best_score:
            best_score = score
            best_contour = contour
    if best_contour is None:
        return {"status": "Not Found", "support_count": 0, "fit_error_px": float("nan")}

    ellipse = cv2.fitEllipse(best_contour)
    local_center = np.asarray(ellipse[0], dtype=float)
    axes = tuple(float(v) for v in ellipse[1])
    angle_deg = float(ellipse[2])
    fit_error = _ellipse_fit_error_px(best_contour, local_center, axes, angle_deg)
    global_center = local_center + np.array([x0, y0], dtype=float)
    contour_uv = _ellipse_contour(global_center, axes, angle_deg)
    support_count = int(round(abs(cv2.contourArea(best_contour))))
    expected_axes = _fit_ellipse_axes(expected_contour_uv, 2.0 * radius_px)
    measured_axes = np.sort(np.asarray(axes, dtype=float))
    axis_error = float(np.max(np.abs(measured_axes - expected_axes) / np.maximum(expected_axes, 1e-6)))
    center_drift = float(np.linalg.norm(global_center - np.asarray(center_uv, dtype=float)))
    center_limit = max(4.0, radius_px * settings.image_hole_max_center_drift_scale)
    status = (
        "OK"
        if support_count >= settings.image_hole_min_support_pixels
        and fit_error <= settings.image_hole_max_fit_error_px
        and center_drift <= center_limit
        and axis_error <= settings.image_hole_max_axis_error
        else "Weak"
    )
    reject_reason = ""
    if center_drift > center_limit:
        reject_reason = f"center drift {center_drift:.1f}px > {center_limit:.1f}px"
    elif axis_error > settings.image_hole_max_axis_error:
        reject_reason = f"axis mismatch {axis_error:.2f} > {settings.image_hole_max_axis_error:.2f}"
    elif fit_error > settings.image_hole_max_fit_error_px:
        reject_reason = f"fit error {fit_error:.1f}px > {settings.image_hole_max_fit_error_px:.1f}px"
    elif support_count < settings.image_hole_min_support_pixels:
        reject_reason = f"support {support_count} < {settings.image_hole_min_support_pixels}"
    return {
        "status": status,
        "measured_u": float(global_center[0]),
        "measured_v": float(global_center[1]),
        "ellipse_axes_px": [float(axes[0]), float(axes[1])],
        "ellipse_angle_deg": angle_deg,
        "measured_contour_uv": contour_uv.tolist() if status == "OK" else [],
        "support_count": support_count,
        "fit_error_px": fit_error,
        "center_drift_px": center_drift,
        "axis_error": axis_error,
        "reject_reason": reject_reason,
    }


def detect_image_features(
    image_path: Path,
    camera: CameraIntrinsics,
    board: BoardConfig,
    image_roi_quad: Optional[Sequence[Sequence[float]]] = None,
    settings: Optional[FeatureDetectionSettings] = None,
) -> ImageDetection:
    settings = settings or FeatureDetectionSettings()
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return ImageDetection(message=f"Failed to read image: {image_path}")

    debug = image.copy()
    _draw_board_roi(debug, image_roi_quad)
    detection_image = _mask_image_to_quad(image, image_roi_quad)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board_corners, board_ids, circle_centers_board = _board_geometry(board)
    corners, ids = _aruco_detect(detection_image, dictionary)
    markers = _marker_records(corners, ids)
    _draw_explicit_markers(debug, markers)
    min_markers = max(1, int(settings.min_aruco_markers))
    if len(ids) < min_markers:
        return ImageDetection(
            marker_ids=[int(x) for x in ids],
            markers=markers,
            debug_image=debug,
            message=f"Only {len(ids)} ArUco markers detected; at least {min_markers} required.",
        )

    object_points = []
    image_points = []
    corner_by_id = {int(marker_id): board_corners[idx] for idx, marker_id in enumerate(board_ids)}
    for marker_corners, marker_id in zip(corners, ids):
        marker_id_int = int(marker_id)
        if marker_id_int not in corner_by_id:
            continue
        object_points.append(corner_by_id[marker_id_int])
        image_points.append(np.asarray(marker_corners, dtype=np.float32).reshape(4, 2))
    if not object_points:
        return ImageDetection(
            marker_ids=[int(x) for x in ids],
            markers=markers,
            debug_image=debug,
            message="No known board markers were detected.",
        )

    k = camera.camera_matrix()
    dist = camera.dist_coeffs()
    object_array = np.concatenate(object_points, axis=0).astype(np.float32)
    image_array = np.concatenate(image_points, axis=0).astype(np.float32)
    try:
        ok, rvec, tvec = cv2.solvePnP(object_array, image_array, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    except cv2.error as exc:
        return ImageDetection(marker_ids=[int(x) for x in ids], markers=markers, debug_image=debug, message=str(exc))

    if not ok or rvec is None or tvec is None:
        return ImageDetection(
            marker_ids=[int(x) for x in ids],
            markers=markers,
            debug_image=debug,
            message="Board pose estimation failed.",
        )

    cv2.drawFrameAxes(debug, k, dist, rvec, tvec, 0.2)
    rot, _ = cv2.Rodrigues(rvec)
    centers_cam_raw = (rot @ circle_centers_board.T).T + np.asarray(tvec, dtype=float).reshape(1, 3)
    centers_cam, sorted_circle_centers_board = _sort_payload_by_pattern(centers_cam_raw, list(circle_centers_board), "camera")
    projected, _ = cv2.projectPoints(centers_cam.reshape(-1, 1, 3), np.zeros((3, 1)), np.zeros((3, 1)), k, dist)
    centers_uv = projected.reshape(-1, 2)
    circles_uv = []
    for idx, uv in enumerate(centers_uv, start=1):
        x, y = int(round(uv[0])), int(round(uv[1]))
        contour_uv = _project_hole_outline(sorted_circle_centers_board[idx - 1], board.circle_radius, rvec, tvec, camera)
        radius_px = max(3.0, float(np.mean(np.linalg.norm(contour_uv - uv.reshape(1, 2), axis=1))))
        local_fit = _fit_local_hole_ellipse(image, uv, radius_px, contour_uv, settings)
        cv2.polylines(debug, [np.round(contour_uv).astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2, cv2.LINE_AA)
        measured_contour = np.asarray(local_fit.get("measured_contour_uv", []), dtype=float)
        if measured_contour.ndim == 2 and measured_contour.shape[1] == 2 and len(measured_contour) >= 5:
            cv2.polylines(
                debug,
                [np.round(measured_contour).astype(np.int32).reshape(-1, 1, 2)],
                True,
                (0, 170, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.circle(debug, (x, y), 6, (255, 0, 255), -1)
        cv2.putText(debug, str(idx), (x + 7, y - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        record = {
            "u": float(uv[0]),
            "v": float(uv[1]),
            "radius_px": radius_px,
            "contour_uv": contour_uv.tolist(),
        }
        record.update(local_fit)
        circles_uv.append(record)

    return ImageDetection(
        centers_cam=centers_cam.tolist(),
        centers_uv=centers_uv.tolist(),
        circles_uv=circles_uv,
        marker_ids=[int(x) for x in ids],
        markers=markers,
        debug_image=debug,
        message=f"Detected {len(ids)} ArUco markers and 4 image hole centers.",
        ok=True,
    )


def _crop_points(points: np.ndarray, roi: Dict[str, float]) -> np.ndarray:
    mask = (
        (points[:, 0] >= roi["x_min"])
        & (points[:, 0] <= roi["x_max"])
        & (points[:, 1] >= roi["y_min"])
        & (points[:, 1] <= roi["y_max"])
        & (points[:, 2] >= roi["z_min"])
        & (points[:, 2] <= roi["z_max"])
    )
    return points[mask]


def _rotation_align_to_z(normal: np.ndarray) -> np.ndarray:
    normal = normal.astype(float)
    normal /= max(np.linalg.norm(normal), 1e-12)
    z_axis = np.array([0.0, 0.0, 1.0])
    axis = np.cross(normal, z_axis)
    dot = float(np.clip(np.dot(normal, z_axis), -1.0, 1.0))
    angle = float(np.arccos(dot))
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9 or abs(angle) < 1e-9:
        return np.eye(3)
    axis /= axis_norm
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]],
        dtype=float,
    )
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _circle_pattern_score(xy: np.ndarray, board: BoardConfig) -> float:
    sorted_xy = np.asarray(sort_pattern_centers(np.column_stack((xy, np.zeros(len(xy)))), "camera"))[:, :2]
    side_lengths = [
        np.linalg.norm(sorted_xy[0] - sorted_xy[1]),
        np.linalg.norm(sorted_xy[1] - sorted_xy[2]),
        np.linalg.norm(sorted_xy[2] - sorted_xy[3]),
        np.linalg.norm(sorted_xy[3] - sorted_xy[0]),
    ]
    pattern_a = [board.delta_width_circles, board.delta_height_circles] * 2
    pattern_b = [board.delta_height_circles, board.delta_width_circles] * 2
    err_a = sum(abs(a - b) / max(b, 1e-6) for a, b in zip(side_lengths, pattern_a))
    err_b = sum(abs(a - b) / max(b, 1e-6) for a, b in zip(side_lengths, pattern_b))
    diagonal = np.linalg.norm(sorted_xy[0] - sorted_xy[2])
    expected_diag = np.hypot(board.delta_width_circles, board.delta_height_circles)
    return min(err_a, err_b) + abs(diagonal - expected_diag) / max(expected_diag, 1e-6)


def _select_circle_pattern(
    circles_xy: np.ndarray,
    radii: np.ndarray,
    board: BoardConfig,
    settings: FeatureDetectionSettings,
) -> Tuple[np.ndarray, np.ndarray]:
    if len(circles_xy) < 4:
        return np.empty((0, 2), dtype=float), np.empty(0, dtype=float)
    nominal_r = max(float(board.circle_radius), 1e-6)
    order = np.arange(len(circles_xy))
    # Cap the combinatorics before the exhaustive 4-combo search: rank the
    # candidates by radius agreement (with the prior when enabled, otherwise
    # with the candidate median) and keep the best 14.
    if len(order) > 14:
        ref = nominal_r if settings.use_circle_prior else float(np.median(radii))
        order = order[np.argsort(np.abs(radii - ref))][:14]
    best_score = float("inf")
    best_indices: Optional[np.ndarray] = None
    for combo in combinations([int(i) for i in order], 4):
        candidate = circles_xy[list(combo)]
        if settings.use_rect_template:
            # Hard prior: the four hole centers form the known rigid rectangle.
            # Any combination that cannot fit it within 5 cm max residual is
            # rejected outright, so spurious raster candidates cannot win on a
            # lenient side-length score.
            _, rect_rms, rect_max = _fit_rectangle_template(candidate, board)
            if rect_max > 0.05:
                continue
            score = rect_rms / 0.01  # one score unit per centimetre of rms
        else:
            score = _circle_pattern_score(candidate, board)
        # The radius-agreement term is part of the ORIGINAL selection score
        # (not an added prior), so it stays on in both modes; the Radius
        # toggle governs the fs rim refinement and radius unification instead.
        score += float(np.mean(np.abs(radii[list(combo)] - nominal_r))) / nominal_r
        if score < best_score:
            best_score = score
            best_indices = np.asarray(combo, dtype=int)
    if best_indices is None:
        return np.empty((0, 2), dtype=float), np.empty(0, dtype=float)
    if not settings.use_rect_template and best_score > settings.lidar_circle_pattern_max_score:
        return np.empty((0, 2), dtype=float), np.empty(0, dtype=float)

    selected_xy = circles_xy[best_indices]
    selected_radii = radii[best_indices]
    sorted_xy = np.asarray(sort_pattern_centers(np.column_stack((selected_xy, np.zeros(4))), "camera"))[:, :2]
    sorted_radii: List[float] = []
    used: set[int] = set()
    for center in sorted_xy:
        order = np.argsort(np.linalg.norm(selected_xy - center, axis=1))
        match = next(int(idx) for idx in order if int(idx) not in used)
        used.add(match)
        sorted_radii.append(float(selected_radii[match]))
    return sorted_xy, np.asarray(sorted_radii, dtype=float)


def _detect_circles_from_plane(
    aligned_xy: np.ndarray,
    board: BoardConfig,
    settings: FeatureDetectionSettings,
) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
    if len(aligned_xy) < 50:
        return np.empty((0, 2), dtype=float), np.empty(0, dtype=float), None
    padding = board.circle_radius * 2.0
    xy_min = aligned_xy.min(axis=0) - padding
    xy_max = aligned_xy.max(axis=0) + padding
    span = np.maximum(xy_max - xy_min, 1e-9)
    resolution = max(0.004, min(0.012, board.circle_radius / 24.0), float(span.max() / 900.0))
    size = np.ceil((xy_max - xy_min) / resolution).astype(int) + 1
    if np.any(size <= 0) or np.any(size > 2400):
        return np.empty((0, 2), dtype=float), np.empty(0, dtype=float), None

    uv = np.floor((aligned_xy - xy_min) / resolution).astype(int)
    uv[:, 0] = np.clip(uv[:, 0], 0, size[0] - 1)
    uv[:, 1] = np.clip(uv[:, 1], 0, size[1] - 1)
    img = np.zeros((size[1], size[0]), dtype=np.uint8)
    img[uv[:, 1], uv[:, 0]] = 255
    img = cv2.dilate(img, np.ones((3, 3), np.uint8), iterations=1)
    close_kernel = max(3, int(round(board.circle_radius / resolution / 4.0)) | 1)
    img_closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((close_kernel, close_kernel), np.uint8), iterations=1)
    debug = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    min_radius = max(4, int((board.circle_radius * settings.lidar_circle_radius_min_scale) / resolution))
    max_radius = max(min_radius + 2, int((board.circle_radius * settings.lidar_circle_radius_max_scale) / resolution))

    contours, hierarchy = cv2.findContours(img_closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    raw_centers = []
    raw_radii = []
    if hierarchy is not None:
        hierarchy = hierarchy[0]
        for contour, rel in zip(contours, hierarchy):
            parent = rel[3]
            if parent < 0 or len(contour) < 8:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if settings.use_circle_prior or settings.use_rect_template:
                # Prefer the area-equivalent radius and the centroid: a single
                # speckle spike on a sparse rim inflates the enclosing circle
                # far past the band while barely moving area or centroid. Kept
                # behind the prior toggles so both-off reproduces the original
                # pipeline exactly.
                area = cv2.contourArea(contour)
                if area > 0:
                    radius = float(np.sqrt(area / np.pi))
                    moments = cv2.moments(contour)
                    if moments["m00"] > 1e-9:
                        cx = moments["m10"] / moments["m00"]
                        cy = moments["m01"] / moments["m00"]
            if min_radius <= radius <= max_radius:
                raw_centers.append([cx, cy])
                raw_radii.append(radius)

    if len(raw_centers) < 4:
        # Fast fallback: connected components on the empty regions inside the
        # occupied board mask. This is less precise than the fitted circle, but
        # it gives the GUI a usable first guess when the point cloud is sparse.
        flood = img_closed.copy()
        mask = np.zeros((flood.shape[0] + 2, flood.shape[1] + 2), np.uint8)
        cv2.floodFill(flood, mask, (0, 0), 255)
        holes = cv2.bitwise_not(flood) & cv2.bitwise_not(img_closed)
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(holes)
        for label in range(1, num):
            area = stats[label, cv2.CC_STAT_AREA]
            radius = np.sqrt(area / np.pi)
            if min_radius <= radius <= max_radius:
                raw_centers.append(centroids[label].tolist())
                raw_radii.append(float(radius))

    if len(raw_centers) < 4:
        return np.empty((0, 2), dtype=float), np.empty(0, dtype=float), debug

    raw_centers_np = np.asarray(raw_centers, dtype=float)
    centers_xy = np.column_stack((raw_centers_np[:, 0] * resolution + xy_min[0], raw_centers_np[:, 1] * resolution + xy_min[1]))
    radii = np.asarray(raw_radii, dtype=float) * resolution
    selected, selected_radii = _select_circle_pattern(centers_xy, radii, board, settings)
    for center, radius in zip(centers_xy, radii):
        pixel = tuple(np.round((center - xy_min) / resolution).astype(int))
        cv2.circle(debug, pixel, int(round(radius / resolution)), (0, 180, 255), 1)
    for idx, center in enumerate(selected, start=1):
        pixel = tuple(np.round((center - xy_min) / resolution).astype(int))
        cv2.circle(debug, pixel, 5, (0, 0, 255), -1)
        cv2.putText(debug, str(idx), (pixel[0] + 5, pixel[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return selected, selected_radii, debug


def _sample_points(points: np.ndarray, max_points: int) -> List[List[float]]:
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        return []
    if len(points) > max_points:
        step = int(np.ceil(len(points) / max_points))
        points = points[::step]
    return np.asarray(points, dtype=float).reshape(-1, 3).tolist()


def _sort_holes_by_lidar_geometry(holes: List[Dict[str, object]]) -> List[Dict[str, object]]:
    if len(holes) != 4:
        return holes
    centers = np.asarray([hole["center"] for hole in holes], dtype=float)
    sorted_centers = np.asarray(sort_pattern_centers(centers, "lidar"), dtype=float)
    ordered: List[Dict[str, object]] = []
    used: set[int] = set()
    for center in sorted_centers:
        order = np.argsort(np.linalg.norm(centers - center, axis=1))
        match = next(int(idx) for idx in order if int(idx) not in used)
        used.add(match)
        hole = dict(holes[match])
        hole["id"] = len(ordered) + 1
        ordered.append(hole)
    return ordered


def _build_lidar_hole_records(
    aligned_plane: np.ndarray,
    centers_xy: np.ndarray,
    radii: np.ndarray,
    avg_z: float,
    r_inv: np.ndarray,
    normal: np.ndarray,
    settings: FeatureDetectionSettings,
) -> List[Dict[str, object]]:
    holes: List[Dict[str, object]] = []
    normal = np.asarray(normal, dtype=float)
    normal /= max(float(np.linalg.norm(normal)), 1e-12)
    plane_xy = aligned_plane[:, :2]
    angles = np.linspace(0.0, 2.0 * np.pi, 96, endpoint=False)
    for idx, (center_xy, radius) in enumerate(zip(centers_xy, radii), start=1):
        radial_error = np.abs(np.linalg.norm(plane_xy - center_xy.reshape(1, 2), axis=1) - float(radius))
        support_mask = radial_error <= max(
            settings.lidar_ring_support_abs_threshold,
            float(radius) * settings.lidar_ring_support_radius_scale,
        )
        support_aligned = aligned_plane[support_mask]
        support_lidar = (r_inv @ support_aligned.T).T if len(support_aligned) else np.empty((0, 3), dtype=float)
        ring_xy = np.column_stack(
            (
                center_xy[0] + float(radius) * np.cos(angles),
                center_xy[1] + float(radius) * np.sin(angles),
            )
        )
        ring_aligned = np.column_stack((ring_xy, np.full(len(ring_xy), avg_z)))
        ring_lidar = (r_inv @ ring_aligned.T).T
        center_lidar = (r_inv @ np.array([center_xy[0], center_xy[1], avg_z], dtype=float)).tolist()
        fit_error = float(np.mean(radial_error[support_mask])) if np.any(support_mask) else float("nan")
        holes.append(
            {
                "id": idx,
                "center": [float(v) for v in center_lidar],
                "radius_m": float(radius),
                "normal": [float(v) for v in normal],
                "support_points": _sample_points(support_lidar, 1500),
                "ring_points": _sample_points(ring_lidar, 128),
                "fit_error_m": fit_error,
                "support_count": int(np.count_nonzero(support_mask)),
                "status": "OK" if np.count_nonzero(support_mask) >= settings.lidar_min_hole_support_points else "Weak",
            }
        )
    return _sort_holes_by_lidar_geometry(holes)


def _crop_mask(points: np.ndarray, roi: Dict[str, float]) -> np.ndarray:
    return (
        (points[:, 0] >= roi["x_min"])
        & (points[:, 0] <= roi["x_max"])
        & (points[:, 1] >= roi["y_min"])
        & (points[:, 1] <= roi["y_max"])
        & (points[:, 2] >= roi["z_min"])
        & (points[:, 2] <= roi["z_max"])
    )


_INTENSITY_CACHE: Dict[str, Tuple[float, np.ndarray]] = {}


def _read_pcd_intensity_cached(path: Path, expected_points: int) -> np.ndarray:
    """Read the intensity column of an ASCII PCD, row-aligned with Open3D's xyz.

    Returns an empty array when the file is binary, has no intensity field, or
    the row count does not match (callers must skip intensity gating then).
    """
    try:
        key = str(path)
        mtime = path.stat().st_mtime
        cached = _INTENSITY_CACHE.get(key)
        if cached is not None and cached[0] == mtime:
            return cached[1] if len(cached[1]) == expected_points else np.empty(0, dtype=float)
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            fields: List[str] = []
            counts: List[int] = []
            data_format = ""
            for line in f:
                if line.startswith("FIELDS"):
                    fields = line.split()[1:]
                if line.startswith("COUNT"):
                    counts = [int(v) for v in line.split()[1:]]
                if line.startswith("DATA"):
                    data_format = line.split()[1].lower()
                    break
            if counts and any(c != 1 for c in counts):
                # multi-count fields shift the column layout; bail out rather
                # than reading the wrong column as intensity
                return np.empty(0, dtype=float)
            if data_format != "ascii" or "intensity" not in fields:
                return np.empty(0, dtype=float)
            body = f.read()
        flat = np.fromstring(body, dtype=float, sep=" ")
        if flat.size == 0 or flat.size % len(fields) != 0:
            return np.empty(0, dtype=float)
        values = np.ascontiguousarray(flat.reshape(-1, len(fields))[:, fields.index("intensity")])
        # Exact row match required: more rows than Open3D points means o3d
        # dropped rows (NaN etc.) and every later intensity would be shifted
        # to the wrong point - silently corrupting the gate is worse than
        # skipping it.
        if len(values) != expected_points:
            return np.empty(0, dtype=float)
        if len(_INTENSITY_CACHE) >= 4:
            _INTENSITY_CACHE.pop(next(iter(_INTENSITY_CACHE)))
        _INTENSITY_CACHE[key] = (mtime, values)
        return values
    except Exception:
        return np.empty(0, dtype=float)


def _fit_rectangle_template(centers_xy: np.ndarray, board: BoardConfig) -> Tuple[np.ndarray, float, float]:
    """Fit the known hole-center rectangle (3 DOF: in-plane rotation+translation)
    to 4 fitted centers and return (projected_centers, rms, max_residual).

    The rectangle is reflection-symmetric, so a proper 2D rotation always
    suffices regardless of the plane-alignment handedness.
    """
    half_w = board.delta_width_circles / 2.0
    half_h = board.delta_height_circles / 2.0
    template = np.array(
        [[-half_w, -half_h], [half_w, -half_h], [half_w, half_h], [-half_w, half_h]], dtype=float
    )
    centers = np.asarray(centers_xy, dtype=float)
    centers_mean = centers.mean(axis=0)
    template_mean = template.mean(axis=0)
    b = centers - centers_mean
    best: Optional[Tuple[float, np.ndarray, float]] = None
    from itertools import permutations

    for perm in permutations(range(4)):
        a = template[list(perm)] - template_mean
        # closed-form 2D Kabsch (proper rotation only)
        theta = np.arctan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])), float(np.sum(a * b)))
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]], dtype=float)
        projected = (rot @ a.T).T + centers_mean
        residuals = np.linalg.norm(centers - projected, axis=1)
        rms = float(np.sqrt(np.mean(residuals**2)))
        if best is None or rms < best[0]:
            best = (rms, projected, float(residuals.max()))
    assert best is not None
    return best[1], best[0], best[2]


def _refine_fs_holes(
    roi_points: np.ndarray,
    roi_intensity: np.ndarray,
    plane_model: Sequence[float],
    r_align: np.ndarray,
    centers_xy: np.ndarray,
    radii: np.ndarray,
    board: BoardConfig,
    settings: FeatureDetectionSettings,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    """FS (area-array lidar) hole-center refinement: A + C + B.

    A: per-hole fixed-radius fit (Huber IRLS) to the VOID BOUNDARY: annulus
       points are bucketed into angular sectors and only each sector's
       innermost point (the hole rim sample) enters the fit. Fitting all
       annulus points instead would weight the dense plate ring outside the
       rim by area and drag the radius estimate outward. A single erosion
       offset delta_r shared by all four holes absorbs the systematic
       mixed-pixel rim erosion without letting per-hole radius noise leak
       into the centers.
    C: annulus points lying inside the rim with low intensity are rejected as
       mixed-pixel returns (the plate is white; bleed points are dark).
    B: the four refined centers are projected onto the known rigid rectangle
       (delta_width_circles x delta_height_circles, 3-DOF in-plane fit).

    Operates on the raw ROI crop (no random cap, no voxel grid) so the annulus
    keeps full point density. Falls back per-hole to the raster center when an
    annulus has too little support, and skips the rectangle projection when the
    template residual is implausibly large.
    """
    nominal_r = float(board.circle_radius)
    normal = np.asarray(plane_model[:3], dtype=float)
    norm_len = max(float(np.linalg.norm(normal)), 1e-12)
    signed = (roi_points @ (normal / norm_len)) + float(plane_model[3]) / norm_len
    slab_limit = max(0.035, 2.5 * settings.lidar_plane_distance_threshold)
    slab_mask = np.abs(signed) <= slab_limit
    slab_points = roi_points[slab_mask]
    has_intensity = len(roi_intensity) == len(roi_points)
    slab_intensity = roi_intensity[slab_mask] if has_intensity else np.empty(0, dtype=float)

    diag: Dict[str, object] = {
        "method": "fs_annulus_v1",
        "slab_points": int(len(slab_points)),
        "intensity_available": bool(has_intensity),
        "centers_raster": np.asarray(centers_xy, dtype=float).tolist(),
        "hole_band_counts": [0, 0, 0, 0],
        "hole_gated_counts": [0, 0, 0, 0],
        "hole_rim_sectors": [0, 0, 0, 0],
        "hole_refined": [False, False, False, False],
    }
    if len(slab_points) < 200:
        diag["status"] = "skipped: slab too sparse"
        diag["centers_refined"] = np.asarray(centers_xy, dtype=float).tolist()
        diag["centers_projected"] = np.asarray(centers_xy, dtype=float).tolist()
        diag["delta_r_m"] = 0.0
        diag["rect_applied"] = False
        return np.asarray(centers_xy, dtype=float), np.asarray(radii, dtype=float), diag

    aligned_xy = (r_align @ slab_points.T).T[:, :2]
    centers = np.asarray(centers_xy, dtype=float).copy()
    band_in, band_out = 0.030, 0.040
    huber_delta = 0.012
    n_sectors = 24
    min_sectors = 10
    radius_tol = max(0.0, float(getattr(settings, "radius_tolerance_m", 0.0)))
    free_radius = radius_tol > 0.0
    delta_r = float(np.clip(nominal_r - float(np.median(radii)), 0.0, 0.030)) if len(radii) else 0.0
    if free_radius:
        # The tolerance-band arm carries no shared bias: each hole keeps its own
        # radius inside the band, so there is nothing for delta_r to absorb.
        delta_r = 0.0
    hole_radii = np.clip(np.asarray(radii, dtype=float), nominal_r - radius_tol,
                         nominal_r + radius_tol) if free_radius else None

    for _outer in range(5):
        r_eff = nominal_r - delta_r
        observed_radii: List[float] = []
        for k in range(len(centers)):
            rel = aligned_xy - centers[k]
            rho = np.linalg.norm(rel, axis=1)
            sel = (rho >= r_eff - band_in) & (rho <= r_eff + band_out)
            gated = 0
            gate_active = has_intensity and settings.fs_intensity_min > 0
            if gate_active:
                # gate at the PHYSICAL hole boundary (nominal radius), not at
                # r_eff: mixed-pixel intruders live anywhere inside the true
                # rim, and gating only below r_eff makes the converged state
                # depend on the delta_r initialization
                interior_dark = sel & (rho < nominal_r + 0.005) & (slab_intensity < settings.fs_intensity_min)
                gated = int(np.count_nonzero(interior_dark))
                sel &= ~interior_dark
            sel_idx = np.where(sel)[0]
            diag["hole_band_counts"][k] = int(len(sel_idx))
            diag["hole_gated_counts"][k] = gated
            if len(sel_idx) == 0:
                continue
            # void-boundary sampling per angular sector. With the intensity
            # gate active the strict innermost point is a clean rim sample;
            # without intensity, ungated mixed pixels make the minimum an
            # outlier magnet, so fall back to the median of the 3 innermost.
            theta = np.arctan2(rel[sel_idx, 1], rel[sel_idx, 0])
            sector = (((theta + np.pi) / (2.0 * np.pi)) * n_sectors).astype(int) % n_sectors
            rho_sel = rho[sel_idx]
            rim_rows: List[int] = []
            occupied = np.zeros(n_sectors, dtype=bool)
            for sct in range(n_sectors):
                in_sector = sector == sct
                if not np.any(in_sector):
                    continue
                occupied[sct] = True
                rows = sel_idx[in_sector]
                inner_order = np.argsort(rho_sel[in_sector])
                if gate_active:
                    rim_rows.append(int(rows[inner_order[0]]))
                else:
                    pick = inner_order[: min(3, len(inner_order))]
                    rim_rows.append(int(rows[pick[len(pick) // 2]]))
            diag["hole_rim_sectors"][k] = len(rim_rows)
            if len(rim_rows) < min_sectors:
                continue
            # angular coverage gate: a contiguous missing arc > 90 deg means a
            # truncated rim whose fixed-radius fit is ill-conditioned along
            # the gap bisector; rectangle projection would smear that bias
            # onto all four holes
            gap = 0
            longest_gap = 0
            for sct in range(2 * n_sectors):
                if occupied[sct % n_sectors]:
                    gap = 0
                else:
                    gap += 1
                    longest_gap = max(longest_gap, gap)
            longest_gap = min(longest_gap, n_sectors)
            if longest_gap > n_sectors // 4:
                continue
            pts = aligned_xy[rim_rows]
            center = centers[k].copy()
            r_hole = float(hole_radii[k]) if free_radius else r_eff
            weights = np.ones(len(pts), dtype=float)
            for _inner in range(3):
                vec = pts - center
                dist = np.linalg.norm(vec, axis=1)
                safe = np.maximum(dist, 1e-9)
                unit = vec / safe[:, None]
                residual = dist - r_hole
                abs_res = np.abs(residual)
                weights = np.where(abs_res <= huber_delta, 1.0, huber_delta / np.maximum(abs_res, 1e-12))
                if free_radius:
                    # Joint center-radius Gauss-Newton: the degenerate direction
                    # of Section III is left in the normal matrix on purpose,
                    # bounded only by the tolerance band.
                    jac = np.hstack((unit, -np.ones((len(pts), 1))))
                    hess = (jac * weights[:, None]).T @ jac
                    grad = (jac * (weights * residual)[:, None]).sum(axis=0)
                else:
                    hess = (unit * weights[:, None]).T @ unit
                    grad = (unit * (weights * residual)[:, None]).sum(axis=0)
                try:
                    step = np.linalg.solve(hess, grad)
                except np.linalg.LinAlgError:
                    break
                step_norm = float(np.linalg.norm(step[:2]))
                if step_norm > 0.05:
                    step = step * (0.05 / step_norm)
                center = center + step[:2]
                if free_radius:
                    r_hole = float(np.clip(r_hole + step[2], nominal_r - radius_tol,
                                           nominal_r + radius_tol))
            centers[k] = center
            if free_radius:
                hole_radii[k] = r_hole
            diag["hole_refined"][k] = True
            final_dist = np.linalg.norm(pts - center, axis=1)
            final_abs = np.abs(final_dist - r_hole)
            final_w = np.where(final_abs <= huber_delta, 1.0, huber_delta / np.maximum(final_abs, 1e-12))
            observed_radii.append(float(np.sum(final_w * final_dist) / max(float(np.sum(final_w)), 1e-12)))
        if observed_radii and not free_radius:
            delta_r = float(np.clip(nominal_r - float(np.mean(observed_radii)), 0.0, 0.030))

    diag["delta_r_m"] = delta_r
    diag["radius_tolerance_m"] = radius_tol
    refined_radii = (np.asarray(hole_radii, dtype=float) if free_radius
                     else np.full(len(centers), nominal_r - delta_r, dtype=float))
    diag["centers_refined"] = centers.tolist()

    rect_applied = False
    if settings.use_rect_template and len(centers) == 4 and all(diag["hole_refined"]):
        projected, rect_rms, rect_max = _fit_rectangle_template(centers, board)
        diag["rect_rms_m"] = rect_rms
        diag["rect_max_m"] = rect_max
        if rect_max <= 0.035:
            centers = projected
            rect_applied = True
    diag["rect_applied"] = rect_applied
    diag["centers_projected"] = centers.tolist()
    diag["status"] = "ok"
    return centers, refined_radii, diag


def detect_lidar_features(
    cloud_path: Path,
    board: BoardConfig,
    roi: Optional[Dict[str, float]] = None,
    settings: Optional[FeatureDetectionSettings] = None,
) -> LidarDetection:
    settings = settings or FeatureDetectionSettings()
    pcd = o3d.io.read_point_cloud(str(cloud_path))
    points = np.asarray(pcd.points, dtype=float)
    if points.size == 0:
        return LidarDetection(message=f"PCD is empty: {cloud_path}")
    active_roi = dict(board.lidar_roi)
    if roi:
        active_roi.update({key: float(value) for key, value in roi.items()})
    filtered = _crop_points(points, active_roi)
    if len(filtered) < 100:
        return LidarDetection(
            active_roi=active_roi,
            filtered_points=filtered,
            message=f"ROI contains too few points: {len(filtered)}",
        )

    if len(filtered) > settings.lidar_max_points:
        rng = np.random.default_rng(7)
        filtered = filtered[rng.choice(len(filtered), size=settings.lidar_max_points, replace=False)]

    # Keep the dense pre-voxel crop: the hole-candidate raster needs it (the
    # 12 mm voxel cloud is far too sparse on a 5 mm grid for area-array lidars).
    roi_raw = filtered
    filtered_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(filtered))
    filtered_pcd = filtered_pcd.voxel_down_sample(voxel_size=settings.lidar_voxel_size)
    filtered = np.asarray(filtered_pcd.points, dtype=float)
    if len(filtered) < 100:
        return LidarDetection(
            active_roi=active_roi,
            filtered_points=filtered,
            message=f"Voxel cloud contains too few points: {len(filtered)}",
        )

    try:
        # Seed Open3D's RNG so RANSAC plane segmentation (and everything
        # downstream of its inlier set) is reproducible run-to-run.
        try:
            o3d.utility.random.seed(7)
        except AttributeError:
            pass
        plane_model, inliers = filtered_pcd.segment_plane(
            distance_threshold=settings.lidar_plane_distance_threshold,
            ransac_n=3,
            num_iterations=settings.lidar_ransac_iterations,
        )
    except RuntimeError as exc:
        return LidarDetection(active_roi=active_roi, filtered_points=filtered, message=str(exc))
    if len(inliers) < 50:
        return LidarDetection(
            active_roi=active_roi,
            filtered_points=filtered,
            message=f"Plane segmentation has too few inliers: {len(inliers)}",
        )

    plane = filtered[np.asarray(inliers, dtype=int)]
    normal = np.asarray(plane_model[:3], dtype=float)
    r_align = _rotation_align_to_z(normal)
    aligned = (r_align @ plane.T).T
    avg_z = float(np.mean(aligned[:, 2]))
    # Rasterize the DENSE pre-voxel slab (all ROI points near the fitted
    # plane), not just the voxelized RANSAC inliers: on area-array lidars the
    # 12 mm voxel cloud is so sparse on the 5 mm raster grid that hole
    # contours merge with background speckle and their radii blow past the
    # candidate band, dropping true holes from the selection.
    if settings.use_circle_prior or settings.use_rect_template:
        norm_len = max(float(np.linalg.norm(normal)), 1e-12)
        signed = (roi_raw @ (normal / norm_len)) + float(plane_model[3]) / norm_len
        slab = roi_raw[np.abs(signed) <= max(0.02, 1.5 * settings.lidar_plane_distance_threshold)]
        raster_xy = (r_align @ slab.T).T[:, :2] if len(slab) >= 100 else aligned[:, :2]
    else:
        # Both priors off = faithful original pipeline (voxel-inlier raster),
        # so results are directly comparable with the stock FAST-Calib flow.
        raster_xy = aligned[:, :2]
    centers_xy, radii, debug = _detect_circles_from_plane(raster_xy, board, settings)
    if len(centers_xy) != 4:
        return LidarDetection(
            active_roi=active_roi,
            filtered_points=filtered,
            plane_points=plane,
            debug_image=debug,
            message="Automatic LiDAR circle detection did not recover four hole centers.",
        )
    message = "Detected 4 LiDAR hole centers."
    fs_diag: Optional[Dict[str, object]] = None
    if settings.lidar_type == "fs" and not settings.use_circle_prior:
        # The FS annulus refinement is a fixed-nominal-radius fit, i.e. it IS
        # the circle prior; without the prior fall through to the generic path.
        message += " (FS refine skipped: circle prior off.)"
    if settings.lidar_type == "fs" and settings.use_circle_prior:
        try:
            roi_mask = _crop_mask(points, active_roi)
            raw_roi = points[roi_mask]
            intensity_full = _read_pcd_intensity_cached(cloud_path, len(points))
            raw_intensity = (
                intensity_full[roi_mask] if len(intensity_full) == len(points) else np.empty(0, dtype=float)
            )
            centers_xy, radii, fs_diag = _refine_fs_holes(
                raw_roi, raw_intensity, plane_model, r_align, centers_xy, radii, board, settings
            )
            delta_mm = float(fs_diag.get("delta_r_m", 0.0)) * 1000.0
            rect_note = (
                f", rect rms {float(fs_diag['rect_rms_m']) * 1000.0:.1f} mm"
                if "rect_rms_m" in fs_diag
                else ""
            )
            intensity_note = (
                ""
                if fs_diag.get("intensity_available")
                else " WARNING: no usable intensity - rim bleed not gated, centers may be biased."
            )
            message = (
                f"Detected 4 LiDAR hole centers (FS refine: {fs_diag.get('status', '?')}, "
                f"rim offset {delta_mm:.1f} mm{rect_note}).{intensity_note}"
            )
        except Exception as exc:  # degrade to the raster result, never abort
            fs_diag = None
            message = f"Detected 4 LiDAR hole centers (FS refine failed, raster centers kept: {exc})."
    if fs_diag is None:
        if settings.use_circle_prior and len(radii):
            # The prior also pins the output radius: raster radii are per-hole
            # area-equivalent estimates that wobble with rim sparsity, so the
            # rings would render at four different sizes. All physical holes
            # share the board's nominal radius.
            radii = np.full(len(radii), float(board.circle_radius), dtype=float)
        if settings.use_rect_template:
            # Generic path (mid360, or fs without the circle prior / after
            # refine failure): project the raster centers onto the known
            # rectangle so the template prior helps every lidar type.
            projected, rect_rms, rect_max = _fit_rectangle_template(centers_xy, board)
            if rect_max <= 0.035:
                centers_xy = projected
                message += f" Rect template applied (rms {rect_rms * 1000.0:.1f} mm)."
            else:
                message += f" Rect template rejected (max residual {rect_max * 1000.0:.0f} mm) - check the ROI/centers."

    r_inv = np.linalg.inv(r_align)
    holes = _build_lidar_hole_records(aligned, centers_xy, radii, avg_z, r_inv, normal, settings)
    centers_prerect: List[List[float]] = []
    if fs_diag is not None:
        for cx, cy in np.asarray(fs_diag.get("centers_refined", []), dtype=float).reshape(-1, 2):
            centers_prerect.append([float(v) for v in (r_inv @ np.array([cx, cy, avg_z], dtype=float))])
        if len(centers_prerect) == 4:
            # match the sorted order used for centers_lidar so downstream
            # cross-checks keep point correspondences
            centers_prerect = sort_pattern_centers(centers_prerect, "lidar")
        refined_xy = np.asarray(fs_diag["centers_projected"], dtype=float)
        for hole in holes:
            center_aligned = (r_align @ np.asarray(hole["center"], dtype=float))[:2]
            nearest = int(np.argmin(np.linalg.norm(refined_xy - center_aligned, axis=1)))
            hole["fs_refine"] = {
                "method": fs_diag.get("method"),
                "refined": bool(fs_diag["hole_refined"][nearest]),
                "band_count": int(fs_diag["hole_band_counts"][nearest]),
                "rim_sectors": int(fs_diag["hole_rim_sectors"][nearest]),
                "intensity_gated": int(fs_diag["hole_gated_counts"][nearest]),
                "delta_r_m": fs_diag.get("delta_r_m"),
                "rect_applied": fs_diag.get("rect_applied"),
                "rect_rms_m": fs_diag.get("rect_rms_m"),
            }
    centers_lidar = np.asarray([hole["center"] for hole in holes], dtype=float)
    return LidarDetection(
        centers_lidar=centers_lidar.tolist(),
        centers_lidar_prerect=centers_prerect,
        holes=holes,
        active_roi=active_roi,
        filtered_points=filtered,
        plane_points=plane,
        debug_image=debug,
        message=message,
        ok=True,
    )


def detect_sample_features(
    image_path: Path,
    cloud_path: Path,
    camera: CameraIntrinsics,
    board: BoardConfig,
    roi: Optional[Dict[str, float]] = None,
    image_roi_quad: Optional[Sequence[Sequence[float]]] = None,
    settings: Optional[FeatureDetectionSettings] = None,
) -> Tuple[ImageDetection, LidarDetection]:
    settings = settings or FeatureDetectionSettings()
    image_result = detect_image_features(image_path, camera, board, image_roi_quad, settings)
    lidar_result = detect_lidar_features(cloud_path, board, roi, settings)
    if image_result.ok and lidar_result.ok:
        from .fast_calib_runner import solve_rigid_transform

        # In FS mode the exported centers are rectangle-projected and would
        # match the camera rectangle with ~zero RMSE by construction; gate on
        # the pre-projection centers so the cross-check keeps its meaning.
        check_centers = (
            lidar_result.centers_lidar_prerect
            if len(lidar_result.centers_lidar_prerect) == 4
            else lidar_result.centers_lidar
        )
        _, rmse, _ = solve_rigid_transform(
            np.asarray(check_centers, dtype=float),
            np.asarray(image_result.centers_cam, dtype=float),
        )
        if rmse > 0.05:
            lidar_result.ok = False
            lidar_result.message = (
                f"LiDAR circle candidates failed the image/LiDAR geometry cross-check "
                f"(single-sample RMSE {rmse:.3f} m > 0.050 m). Review or edit manually."
            )
    return image_result, lidar_result
