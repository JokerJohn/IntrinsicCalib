"""Standalone camera-intrinsics calibration module (chessboard, ROS-style).

Design: INTRINSICS_MODULE_DESIGN.md. The core (detection / calibration /
error statistics) is plain-function so it can run headless for validation;
the dialog wraps it with a list-view-panel layout mirroring the main window.

Charts are rendered with matplotlib Agg into QPixmaps (no Qt backend
dependency); heavy work runs in a QThread.
"""
from __future__ import annotations

import io
import json
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDoubleSpinBox, QFileDialog, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QMessageBox,
    QPushButton, QScrollArea, QSpinBox, QSplitter, QTabWidget, QVBoxLayout,
    QWidget,
)

SESSION_FILE = ".fastcalib_intrinsics.json"
PRUNE_FACTOR = 2.0   # flag images above PRUNE_FACTOR x median error
RESIDUAL_SCALE = 30  # residual-vector display magnification


# --------------------------- headless core ---------------------------

@dataclass
class BoardSpec:
    pattern: str = "chessboard"  # or "aprilgrid" (kalibr 36h11)
    cols: int = 11          # inner corners (chessboard) / tag cols (aprilgrid)
    rows: int = 8           # inner corners (chessboard) / tag rows (aprilgrid)
    square_mm: float = 45.0  # square size / tag size [mm]; does NOT affect K/dist
    tag_spacing: float = 0.3  # aprilgrid only: gap/tag-size ratio (kalibr)
    detector: str = "auto"  # legacy chessboard: auto = SB first, classic fallback
    # Corner extractor: "kalibr" = Kalibr's own extractors with Kalibr's
    # default options (kalibr_extractor/); "legacy" = the FAST-Calib extractors
    # used before 2026-10 (chessboard: `detector`; aprilgrid: tags-only, then
    # grid completion with a rough intrinsics guess at Calibrate time);
    # "rap" = AprilGrid: Kalibr decoding + saddle refinement + per-corner
    # acceptance (aprilgrid_refine.py); "auto" = rap for AprilGrid, legacy (SB
    # high-accuracy) for chessboard.
    # Chessboard stays on SB because Kalibr's checkerboard extractor (classic
    # findChessboardCorners + 11 px half-window cornerSubPix) is less accurate
    # and removes the SB/classic distinction that Stage 1 relies on
    # (experiment/extractor_check: Rig-A held-out 0.316 -> 0.458 px).
    extractor: str = "auto"

    def resolved_extractor(self) -> str:
        if self.extractor == "auto":
            return "rap" if self.pattern == "aprilgrid" else "legacy"
        if self.extractor == "rap" and self.pattern != "aprilgrid":
            return "legacy"
        return self.extractor

    def object_points(self) -> np.ndarray:
        grid = np.zeros((self.rows * self.cols, 3), np.float32)
        grid[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        return grid * (self.square_mm / 1000.0)

    def to_board_config(self):
        from .models import BoardConfig
        return BoardConfig(tag_rows=self.rows, tag_cols=self.cols,
                           tag_size=self.square_mm / 1000.0,
                           tag_spacing=self.tag_spacing)


def detect_corners_ex(gray: np.ndarray, spec: BoardSpec):
    """Returns (corners (N,2) or None, object_points (N,3) or None, source).

    source is "sb" / "classic" / "april" - the calibration step prefers the
    sub-pixel-exact SB detections when enough of them exist (mixing in the
    coarser classic fallback measurably degrades accuracy: 0.36 -> 0.47 px
    mean corner error on the Rig-A set)."""
    if spec.pattern == "aprilgrid" and spec.resolved_extractor() == "rap":
        from .aprilgrid_refine import detect_aprilgrid_rap
        uv, obj, src, _msg = detect_aprilgrid_rap(gray, spec.rows, spec.cols,
                                                  spec.square_mm / 1000.0,
                                                  spec.tag_spacing)
        if uv is not None and len(uv) >= 8:
            return uv, obj, src
        return None, None, None
    if spec.pattern == "aprilgrid" and spec.resolved_extractor() == "kalibr":
        from .kalibr_extractor import detect_aprilgrid_kalibr
        uv, obj, _msg = detect_aprilgrid_kalibr(gray, spec.rows, spec.cols,
                                                spec.square_mm / 1000.0,
                                                spec.tag_spacing)
        if uv is not None and len(uv) >= 8:
            return uv, obj, "kalibr"
        return None, None, None
    if spec.pattern == "aprilgrid":
        from .feature_detector import detect_aprilgrid_corners

        # Tags-only at detection time: without intrinsics the grid-completion
        # homography is unreliable (see feature_detector). The Calibrate step
        # runs the two-stage bootstrap (rough tags-only calibration, then
        # full-grid completion with the rough intrinsics).
        uv, obj, _msg = detect_aprilgrid_corners(gray, spec.to_board_config(),
                                                 complete_grid=False)
        if uv is not None and len(uv) >= 8:
            return uv, obj, "april"
        return None, None, None
    corners, source = detect_corners(gray, spec, with_source=True)
    return corners, None, source


def detect_corners(gray: np.ndarray, spec: BoardSpec, with_source: bool = False):
    """Return (N,2) float32 corners in row-major board order, or None.

    IMPORTANT: the pattern size must be the FULL inner-corner grid. A smaller
    pattern can match different sub-grids in different images and silently
    destroy the calibration (observed on the Rig-A data with 7x5 vs 11x8).
    """
    if spec.resolved_extractor() == "kalibr":
        from .kalibr_extractor import detect_checkerboard_kalibr
        result = detect_checkerboard_kalibr(gray, spec.cols, spec.rows)
        source = "kalibr" if result is not None else None
        return (result, source) if with_source else result
    pat = (spec.cols, spec.rows)
    ok = False
    corners = None
    source = None
    if spec.detector in ("auto", "sb"):
        # CALIB_CB_ACCURACY: sharper sub-pixel model inside the SB detector.
        ok, corners = cv2.findChessboardCornersSB(
            gray, pat, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            source = "sb"
    if not ok and spec.detector in ("auto", "classic"):
        # ROS camera_calibration equivalent: classic detector + sub-pixel refine
        # with a window adapted to the actual corner spacing (a fixed 11x11 is
        # too small for large squares and too large for small ones).
        ok, corners = cv2.findChessboardCorners(
            gray, pat,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            grid = corners.reshape(spec.rows, spec.cols, 2)
            spacing = float(np.median(np.linalg.norm(np.diff(grid, axis=1), axis=2)))
            win = int(np.clip(spacing * 0.35, 5, 25))
            corners = cv2.cornerSubPix(
                gray, corners, (win, win), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4))
            source = "classic"
    result = corners.reshape(-1, 2).astype(np.float32) if ok else None
    return (result, source) if with_source else result


def calibrate(corner_sets: List[np.ndarray], spec: BoardSpec,
              image_size: Tuple[int, int], model: str = "plumb_bob",
              fix_k3: bool = True,
              object_sets: Optional[List[np.ndarray]] = None) -> dict:
    """model in {plumb_bob, rational, fisheye}; returns result dict.
    object_sets: per-image (N_i,3) object points (aprilgrid); when None the
    chessboard grid from `spec` is used for every image."""
    if object_sets is not None:
        objs = [np.asarray(o, np.float32) for o in object_sets]
    else:
        obj = spec.object_points()
        objs = [obj] * len(corner_sets)
    if model == "fisheye":
        o = [x.reshape(-1, 1, 3).astype(np.float64) for x in objs]
        c = [x.reshape(-1, 1, 2).astype(np.float64) for x in corner_sets]
        K = np.eye(3)
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
                 cv2.fisheye.CALIB_FIX_SKEW)
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            o, c, image_size, K, D, None, None, flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-8))
        dist = D.ravel().tolist()
    else:
        flags = 0
        if model == "rational":
            flags |= cv2.CALIB_RATIONAL_MODEL
        elif fix_k3:
            flags |= cv2.CALIB_FIX_K3
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            objs, corner_sets, image_size, None, None, flags=flags)
        dist = D.ravel().tolist()
    per_image = []
    for i in range(len(corner_sets)):
        if model == "fisheye":
            proj, _ = cv2.fisheye.projectPoints(
                objs[i].reshape(-1, 1, 3).astype(np.float64), rvecs[i], tvecs[i], K, D)
        else:
            proj, _ = cv2.projectPoints(objs[i], rvecs[i], tvecs[i], K, D)
        res = corner_sets[i] - proj.reshape(-1, 2)
        per_image.append(dict(mean=float(np.linalg.norm(res, axis=1).mean()),
                              max=float(np.linalg.norm(res, axis=1).max()),
                              residuals=res.tolist()))
    return dict(model=model, rms=float(rms), K=np.asarray(K).tolist(),
                dist=dist, image_size=list(image_size), per_image=per_image,
                rvecs=[r.ravel().tolist() for r in rvecs],
                tvecs=[t.ravel().tolist() for t in tvecs])


def load_reference_yaml(path: str) -> Optional[dict]:
    """Accepts ROS ost.yaml or flat GUI yaml; returns {fx,fy,cx,cy,dist[]}."""
    data = yaml.safe_load(open(path))
    if not isinstance(data, dict):
        return None
    if "camera_matrix" in data:
        k = np.array(data["camera_matrix"]["data"], float).reshape(3, 3)
        dist = list(map(float, data.get("distortion_coefficients", {}).get("data", [])))
        return dict(fx=k[0, 0], fy=k[1, 1], cx=k[0, 2], cy=k[1, 2], dist=dist)
    if "fx" in data:
        dist = [float(data.get(k, 0.0)) for k in ("k1", "k2", "p1", "p2", "k3")]
        return dict(fx=float(data["fx"]), fy=float(data["fy"]),
                    cx=float(data["cx"]), cy=float(data["cy"]), dist=dist)
    return None


def result_to_gui_yaml(result: dict) -> dict:
    K = np.array(result["K"])
    d = list(result["dist"]) + [0.0] * 5
    return dict(fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]),
                cy=float(K[1, 2]), k1=float(d[0]), k2=float(d[1]),
                p1=float(d[2]), p2=float(d[3]), k3=float(d[4]))


def result_to_ost_yaml(result: dict, name: str = "camera") -> dict:
    K = np.array(result["K"])
    d = list(result["dist"]) + [0.0] * 5
    w, h = result["image_size"]
    return dict(
        image_width=int(w), image_height=int(h), camera_name=name,
        camera_matrix=dict(rows=3, cols=3, data=[float(x) for x in K.ravel()]),
        distortion_model="plumb_bob",
        distortion_coefficients=dict(rows=1, cols=5, data=[float(x) for x in d[:5]]),
        rectification_matrix=dict(rows=3, cols=3,
                                  data=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
        projection_matrix=dict(rows=3, cols=4, data=[
            float(K[0, 0]), 0.0, float(K[0, 2]), 0.0,
            0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
            0.0, 0.0, 1.0, 0.0]),
    )


# --------------------------- chart rendering ---------------------------

def _fig_to_pixmap(fig) -> QPixmap:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    import matplotlib.pyplot as plt
    plt.close(fig)
    pix = QPixmap()
    pix.loadFromData(buf.getvalue(), "PNG")
    return pix


def render_error_chart(names: List[str], means: List[float],
                       statuses: Optional[List[str]] = None) -> QPixmap:
    """Per-image mean error bars over EVERY detected frame. Colors show the
    role each frame played in the solve: blue = used, orange = auto-pruned
    (error too high), gray = excluded (classic-fallback detection, shown for
    reference but never optimized)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if statuses is None:
        statuses = ["used"] * len(means)
    used_means = [m for m, st in zip(means, statuses) if st == "used"]
    med = float(np.median(used_means)) if used_means else 0.0
    palette = {"used": "#4a86c8", "pruned": "#e07b00", "excluded": "#b0b0b0"}
    colors = [palette.get(st, "#4a86c8") for st in statuses]
    fig, ax = plt.subplots(figsize=(7.6, 3.2))
    ax.bar(range(len(means)), means, color=colors)
    # Errors span two orders of magnitude once rejected frames are shown
    # (0.1 px used vs 15 px excluded) - a linear axis flattens the used bars
    # into invisibility, so use a log axis with the prune line still visible.
    ax.set_yscale("log")
    lo = max(min(m for m in means if m > 0) * 0.7, 1e-3) if means else 1e-2
    ax.set_ylim(lo, max(max(means) * 1.6, PRUNE_FACTOR * med * 2 if med else 1))
    for x, (m, st) in enumerate(zip(means, statuses)):
        if med and m > 4 * PRUNE_FACTOR * med:
            ax.text(x, m * 1.05, f"{m:.1f}", ha="center", va="bottom", fontsize=5.5,
                    color="#555")
    ax.axhline(med, color="#666", lw=0.8, ls="--", label=f"median (used) {med:.3f}px")
    ax.axhline(PRUNE_FACTOR * med, color="#c0392b", lw=0.8, ls=":",
               label=f"{PRUNE_FACTOR:g}x median (prune line)")
    ax.grid(axis="y", which="both", alpha=0.25, lw=0.4)
    from matplotlib.patches import Patch
    handles = [Patch(color=palette["used"], label=f"used ({statuses.count('used')})")]
    if "pruned" in statuses:
        handles.append(Patch(color=palette["pruned"],
                             label=f"auto-pruned ({statuses.count('pruned')})"))
    if "excluded" in statuses:
        handles.append(Patch(color=palette["excluded"],
                             label=f"excluded ({statuses.count('excluded')})"))
    handles += ax.get_lines()
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=90, fontsize=5)
    ax.set_ylabel("mean reproj err (px, log)")
    ax.legend(handles=handles, fontsize=6.5, loc="upper right")
    fig.tight_layout()
    return _fig_to_pixmap(fig)


def render_residual_scatter(per_image: List[dict], rms: float) -> QPixmap:
    """Kalibr-report style: every corner's reprojection residual (dx, dy) as a
    scatter, colored per image, with the RMS circle. Pattern-agnostic (works
    for chessboard and aprilgrid alike)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5.6, 5.2))
    cmap = plt.get_cmap("turbo")
    n_img = max(len(per_image), 1)
    all_res = []
    for i, entry in enumerate(per_image):
        res = np.asarray(entry.get("residuals", []), float)
        if res.size == 0:
            continue
        ax.scatter(res[:, 0], res[:, 1], s=6, alpha=0.6,
                   color=cmap(i / n_img), linewidths=0)
        all_res.append(res)
    theta = np.linspace(0, 2 * np.pi, 120)
    ax.plot(rms * np.cos(theta), rms * np.sin(theta), "k--", lw=1.0,
            label=f"RMS {rms:.3f} px")
    ax.axhline(0, color="#999", lw=0.5)
    ax.axvline(0, color="#999", lw=0.5)
    # Axis limits from the residual DISTRIBUTION, not the extremes: one
    # outlier at 15 px would otherwise shrink the useful cluster to a dot.
    if all_res:
        flat = np.abs(np.vstack(all_res))
        lim = float(np.percentile(flat, 99)) * 1.4
        n_out = int((flat.max(axis=1) > lim).sum())
        if n_out:
            ax.text(0.02, 0.02, f"{n_out} corner(s) outside view",
                    transform=ax.transAxes, fontsize=7, color="#888")
    else:
        lim = 0.5
    lim = max(lim, rms * 3.0, 0.5)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_xlabel("residual x (px)")
    ax.set_ylabel("residual y (px)")
    ax.set_title(f"reprojection residuals - {len(per_image)} images (color = image)",
                 fontsize=9)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    return _fig_to_pixmap(fig)


def render_coverage(corner_sets: List[np.ndarray], image_size) -> QPixmap:
    """Corner scatter plus an explicit image-corner audit: uncovered corner
    zones are the main reason a distortion model cannot be tightened (a free
    k3 extrapolates wildly where no corner has ever been), so their status
    must be visible, not inferred."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    w, h = image_size
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    for c in corner_sets:
        ax.scatter(c[:, 0], c[:, 1], s=3, alpha=0.45)
    allc = (np.vstack(corner_sets) if corner_sets else np.empty((0, 2)))
    missing = []
    for name, (x0, x1, y0, y1) in {
        "TL": (0, .2 * w, 0, .2 * h), "TR": (.8 * w, w, 0, .2 * h),
        "BL": (0, .2 * w, .8 * h, h), "BR": (.8 * w, w, .8 * h, h),
    }.items():
        n = int(((allc[:, 0] >= x0) & (allc[:, 0] < x1)
                 & (allc[:, 1] >= y0) & (allc[:, 1] < y1)).sum()) if len(allc) else 0
        ok = n >= 10
        if not ok:
            missing.append(name)
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=1.2,
                               edgecolor="#2a9d2a" if ok else "#d62728",
                               linestyle="-" if ok else "--"))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, f"{name}\n{n}",
                ha="center", va="center", fontsize=8,
                color="#2a9d2a" if ok else "#d62728")
    title = f"corner coverage ({len(corner_sets)} images)"
    if missing:
        title += f"  -  UNCOVERED: {', '.join(missing)} (tilt the board into these corners!)"
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=8.5,
                 color="#d62728" if missing else "#222222")
    fig.tight_layout()
    return _fig_to_pixmap(fig)


# --------------------------- worker thread ---------------------------

class _Worker(QThread):
    progressed = Signal(int, int)      # done, total
    detected = Signal(str, object)     # path, corners or None
    finished_calib = Signal(object)    # result dict or Exception

    def __init__(self, job, parent=None):
        super().__init__(parent)
        self._job = job

    def run(self):  # noqa: D102
        self._job(self)


# --------------------------- dialog ---------------------------

@dataclass
class _ImageEntry:
    path: Path
    checked: bool = True
    corners: Optional[np.ndarray] = None
    objpoints: Optional[np.ndarray] = None  # aprilgrid: matched 3D points
    source: Optional[str] = None       # "rap" / "kalibr" / "sb" / "classic" / "april"
    detected: Optional[bool] = None    # None = not attempted
    calib_index: Optional[int] = None  # row in last calibration


class IntrinsicsDialog(QDialog):
    """Standalone chessboard intrinsic calibration with error visualization."""

    def __init__(self, parent=None, current_dataset_dir: Optional[str] = None,
                 log_fn=None):
        super().__init__(parent)
        self._log_fn = log_fn
        self._frame_report = None
        self.setWindowTitle("Camera Intrinsics Calibration")
        self.setModal(False)
        self.resize(1280, 760)
        self.entries: List[_ImageEntry] = []
        self.image_size: Optional[Tuple[int, int]] = None
        self.result: Optional[dict] = None
        self.session_dir: Optional[Path] = None
        self.current_dataset_dir = current_dataset_dir
        self._worker: Optional[_Worker] = None
        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self):
        split = QSplitter(Qt.Horizontal, self)
        # left: image list
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(4, 4, 4, 4)
        self.all_box = QCheckBox("Select all")
        self.all_box.setTristate(True)
        self.all_box.clicked.connect(self._toggle_all)
        lv.addWidget(self.all_box)
        self.listw = QListWidget()
        self.listw.currentRowChanged.connect(lambda _i: self._show_current())
        self.listw.itemChanged.connect(self._item_changed)
        lv.addWidget(self.listw, 1)
        row = QHBoxLayout()
        for text, fn in (("Add Folder", self._add_folder),
                         ("Add Images", self._add_images),
                         ("Remove", self._remove_selected),
                         ("Reset", self.reset_selection)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            row.addWidget(b)
        lv.addLayout(row)

        # center: tabs
        self.tabs = QTabWidget()
        self.img_label = QLabel("Add chessboard images to begin.")
        self.img_label.setAlignment(Qt.AlignCenter)
        sc = QScrollArea()
        sc.setWidget(self.img_label)
        sc.setWidgetResizable(True)
        sc.setToolTip("Mouse wheel: zoom - double-click: fit to view")
        self._img_scroll = sc
        self._img_zoom = None      # None = fit-to-view
        self._current_bgr = None
        sc.viewport().installEventFilter(self)
        sc.installEventFilter(self)
        self.tabs.addTab(sc, "Image")
        self.cov_label = QLabel("Run Detect All first.")
        self.cov_label.setAlignment(Qt.AlignCenter)
        self.tabs.addTab(self.cov_label, "Coverage")
        self.err_label = QLabel("Run Calibrate first.")
        self.err_label.setAlignment(Qt.AlignCenter)
        self.tabs.addTab(self.err_label, "Errors")
        self.scatter_label = QLabel("Run Calibrate first.")
        self.scatter_label.setAlignment(Qt.AlignCenter)
        self.tabs.addTab(self.scatter_label, "Residuals")
        self.und_label = QLabel("Run Calibrate first.")
        self.und_label.setAlignment(Qt.AlignCenter)
        sc2 = QScrollArea()
        sc2.setWidget(self.und_label)
        sc2.setWidgetResizable(True)
        self.tabs.addTab(sc2, "Undistort")
        self.tabs.currentChanged.connect(lambda _i: self._refresh_views())

        # right: control panel
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(4, 4, 4, 4)

        board = QGroupBox("Board")
        bg = QGridLayout(board)
        self.pattern_combo = QComboBox()
        self.pattern_combo.addItems(["Chessboard", "AprilGrid (kalibr 36h11)"])
        self.cols_spin = QSpinBox(); self.cols_spin.setRange(1, 30); self.cols_spin.setValue(11)
        self.rows_spin = QSpinBox(); self.rows_spin.setRange(1, 30); self.rows_spin.setValue(8)
        self.sq_spin = QDoubleSpinBox(); self.sq_spin.setRange(1, 500); self.sq_spin.setValue(45.0)
        self.sq_spin.setSuffix(" mm")
        self.sq_spin.setToolTip("Square/tag size does not affect K / distortion;\n"
                                "it only scales the board poses.")
        self.spacing_spin = QDoubleSpinBox()
        self.spacing_spin.setRange(0.05, 2.0); self.spacing_spin.setSingleStep(0.05)
        self.spacing_spin.setValue(0.3)
        self.spacing_spin.setToolTip("AprilGrid tagSpacing: gap / tag size ratio (kalibr).")
        self.ext_combo = QComboBox()
        self.ext_combo.addItems(["Auto (refined AprilGrid / SB chessboard)",
                                 "Kalibr", "Legacy (FAST-Calib)"])
        self.ext_combo.setToolTip(
            "Auto (default): AprilGrid = Kalibr tag decoding + saddle-point\n"
            "refinement with per-corner acceptance (Stage 1 = corner test,\n"
            "then >= 4 complete tags per frame); chessboard = SB high-accuracy.\n"
            "Kalibr: Kalibr's checkerboard / AprilGrid extractors with Kalibr's\n"
            "default options (corners from the raw image, no intrinsics guess).\n"
            "Legacy: the previous FAST-Calib extractors (chessboard SB/classic;\n"
            "AprilGrid grid completion with a rough intrinsics guess).")
        self.det_combo = QComboBox()
        self.det_combo.addItems(["Auto (SB + classic fallback)", "SB only", "Classic only"])
        self._board_row_labels = []
        for r, (lab, wdg) in enumerate((("Pattern", self.pattern_combo),
                                        ("Inner cols", self.cols_spin),
                                        ("Inner rows", self.rows_spin),
                                        ("Square", self.sq_spin),
                                        ("Spacing", self.spacing_spin),
                                        ("Extractor", self.ext_combo),
                                        ("Detector", self.det_combo))):
            lbl = QLabel(lab)
            self._board_row_labels.append(lbl)
            bg.addWidget(lbl, r, 0)
            bg.addWidget(wdg, r, 1)
        for w in (self.cols_spin, self.rows_spin):
            w.valueChanged.connect(self._invalidate_detection)
        self.spacing_spin.valueChanged.connect(self._invalidate_detection)
        self.det_combo.currentIndexChanged.connect(self._invalidate_detection)
        self.ext_combo.currentIndexChanged.connect(self._on_extractor_changed)
        self.pattern_combo.currentIndexChanged.connect(self._on_pattern_changed)
        self._on_pattern_changed()
        rv.addWidget(board)

        model = QGroupBox("Model")
        mg = QGridLayout(model)
        self.model_combo = QComboBox()
        self.model_combo.addItems(["plumb_bob", "rational", "fisheye"])
        self.k3_box = QCheckBox("fix k3 = 0 (auto-upgrade if safe)")
        # Fixed k3 is the safe baseline: a free r^6 term fits superbly inside
        # the corner-coverage radius but EXPLODES beyond it (Rig-A: in-sample
        # 0.14 px yet 15 px on frames near the uncovered image corners). The
        # calibrate step therefore trains both variants and adopts free k3
        # only when held-out frames prove it safe.
        self.k3_box.setChecked(True)
        self.autoprune_box = QCheckBox("auto-prune blurred frames")
        self.autoprune_box.setChecked(True)
        self.autoprune_box.setToolTip(
            "After calibrating, iteratively drop images whose mean error exceeds\n"
            "2x the median and recalibrate (max 3 rounds) - motion-blurred\n"
            "frames dominate the residual otherwise. MATLAB users do the same\n"
            "step by hand when they delete red bars.")
        mg.addWidget(QLabel("Distortion"), 0, 0)
        mg.addWidget(self.model_combo, 0, 1)
        mg.addWidget(self.k3_box, 1, 0, 1, 2)
        mg.addWidget(self.autoprune_box, 2, 0, 1, 2)
        rv.addWidget(model)

        run = QGroupBox("Run")
        rg = QHBoxLayout(run)
        rg.setSpacing(4)
        self.detect_btn = QPushButton("Detect All")
        self.detect_btn.clicked.connect(self._detect_all)
        self.calib_btn = QPushButton("Calibrate")
        self.calib_btn.clicked.connect(self._calibrate)
        # Auto-prune now runs inside Calibrate (checkbox above); the manual
        # button kept a hidden alias for scripts but is no longer shown.
        self.prune_btn = QPushButton("Auto-prune && Recalibrate")
        self.prune_btn.clicked.connect(self._auto_prune)
        self.prune_btn.setVisible(False)
        for b in (self.detect_btn, self.calib_btn):
            b.setFixedHeight(28)
            rg.addWidget(b)
        rv.addWidget(run)

        res = QGroupBox("Results")
        resv = QVBoxLayout(res)
        self.res_label = QLabel("—")
        self.res_label.setWordWrap(True)
        self.res_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        resv.addWidget(self.res_label)
        self.cmp_btn = QPushButton("Compare with YAML...")
        self.cmp_btn.clicked.connect(self._compare_yaml)
        resv.addWidget(self.cmp_btn)
        rv.addWidget(res)

        exp = QGroupBox("Export")
        ev = QVBoxLayout(exp)
        ev.setSpacing(4)
        row_exp = QHBoxLayout()
        row_exp.setSpacing(4)
        self.save_gui_btn = QPushButton("GUI YAML")
        self.save_gui_btn.setToolTip("Save flat fx..k3 yaml (extrinsic workbench format).")
        self.save_gui_btn.clicked.connect(lambda: self._save_yaml("gui"))
        self.save_ost_btn = QPushButton("ROS ost.yaml")
        self.save_ost_btn.clicked.connect(lambda: self._save_yaml("ost"))
        for b in (self.save_gui_btn, self.save_ost_btn):
            b.setFixedHeight(26)
            row_exp.addWidget(b)
        ev.addLayout(row_exp)
        self.apply_btn = QPushButton("Apply to Current Project")
        self.apply_btn.setFixedHeight(28)
        self.apply_btn.clicked.connect(self._apply_to_project)
        self.apply_btn.setEnabled(bool(self.current_dataset_dir))
        ev.addWidget(self.apply_btn)
        rv.addWidget(exp)
        rv.addStretch(1)

        right.setMinimumWidth(230)
        right.setMaximumWidth(280)
        split.addWidget(left)
        split.addWidget(self.tabs)
        split.addWidget(right)
        split.setSizes([200, 820, 250])
        lay = QVBoxLayout(self)
        lay.addWidget(split, 1)
        self.status_label = QLabel("Ready.")
        lay.addWidget(self.status_label)

    def _log(self, message: str) -> None:
        self.status_label.setText(message)
        if self._log_fn is not None:
            self._log_fn(f"Intrinsics: {message}")

    def _build_frame_report(self, res):
        """(name, mean_err_px, status) for every detected frame; status in
        used / pruned / excluded. Errors for non-used frames come from a PnP
        against the final intrinsics."""
        used_names = set(res.get("used_names") or [])
        pruned_names = set(res.get("pruned_names") or [])
        K = np.array(res["K"])
        D = np.array((list(res["dist"]) + [0.0] * 5)[:5])
        spec = self._spec()
        obj_default = spec.object_points().astype(np.float64)
        report = []
        per = res["per_image"]
        used_idx = 0
        for e in self.entries:
            if not e.detected or e.corners is None:
                continue
            if e.path.name in used_names and used_idx < len(per):
                report.append((e.path.name, float(per[used_idx]["mean"]), "used"))
                used_idx += 1
                continue
            obj = (np.asarray(e.objpoints, np.float64)
                   if e.objpoints is not None else obj_default)
            if len(obj) != len(e.corners):
                continue
            ok, rvec, tvec = cv2.solvePnP(obj, e.corners.astype(np.float64), K, D)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            err = float(np.linalg.norm(proj.reshape(-1, 2)
                                       - e.corners, axis=1).mean())
            status = "pruned" if e.path.name in pruned_names else "excluded"
            report.append((e.path.name, err, status))
        return report

    def reset_selection(self) -> None:
        """Re-check every detected frame and clear the calibration so a fresh
        run (e.g. for before/after comparisons) starts from a clean slate."""
        for e in self.entries:
            e.checked = e.detected is not False
            e.calib_index = None
        self.result = None
        self._frame_report = None
        self._rebuild_list()
        self.err_label.setText("Run Calibrate first.")
        self.scatter_label.setText("Run Calibrate first.")
        self._log("Selection reset: all detected frames re-checked, result cleared.")

    # ---------- board spec ----------
    def _pattern(self) -> str:
        return "aprilgrid" if self.pattern_combo.currentIndex() == 1 else "chessboard"

    def _on_pattern_changed(self):
        april = self._pattern() == "aprilgrid"
        for lbl in self._board_row_labels:
            if lbl.text() in ("Inner cols", "Tag cols"):
                lbl.setText("Tag cols" if april else "Inner cols")
            elif lbl.text() in ("Inner rows", "Tag rows"):
                lbl.setText("Tag rows" if april else "Inner rows")
            elif lbl.text() in ("Square", "Tag size"):
                lbl.setText("Tag size" if april else "Square")
        self.spacing_spin.setEnabled(april)
        self.det_combo.setEnabled(self._det_enabled())
        if april and self.cols_spin.value() > 8:
            self.cols_spin.setValue(3)
            self.rows_spin.setValue(3)
            self.sq_spin.setValue(115.0)
        if hasattr(self, "status_label"):  # skip during _build_ui
            self._invalidate_detection()

    def _extractor(self) -> str:
        return ("auto", "kalibr", "legacy")[self.ext_combo.currentIndex()]

    def _det_enabled(self) -> bool:
        return self._pattern() != "aprilgrid" and self._extractor() in ("auto", "legacy")

    def _on_extractor_changed(self):
        self.det_combo.setEnabled(self._det_enabled())
        self._invalidate_detection()

    def _spec(self) -> BoardSpec:
        return BoardSpec(pattern=self._pattern(),
                         cols=self.cols_spin.value(), rows=self.rows_spin.value(),
                         square_mm=self.sq_spin.value(),
                         tag_spacing=self.spacing_spin.value(),
                         detector=("auto", "sb", "classic")[self.det_combo.currentIndex()],
                         extractor=self._extractor())

    # ---------- list handling ----------
    def _add_folder(self):
        d = QFileDialog.getExistingDirectory(self, "Chessboard image folder")
        if d:
            self.load_folder(Path(d))

    def apply_board_config(self, board) -> None:
        """Seed the Board group from the dataset's board_config so the panel
        never sits on a mismatched pattern (the #1 cause of "no tags detected":
        an aprilgrid dataset being scanned with an 11x8 chessboard pattern)."""
        from .models import BOARD_TYPE_APRILGRID, BOARD_TYPE_CHECKERBOARD

        if board is None:
            return
        if board.board_type == BOARD_TYPE_APRILGRID:
            self.pattern_combo.setCurrentIndex(1)
            self.cols_spin.setValue(int(board.tag_cols))
            self.rows_spin.setValue(int(board.tag_rows))
            self.sq_spin.setValue(float(board.tag_size) * 1000.0)
            self.spacing_spin.setValue(float(board.tag_spacing))
        elif board.board_type == BOARD_TYPE_CHECKERBOARD:
            self.pattern_combo.setCurrentIndex(0)
            if board.checkerboard_cols and board.checkerboard_rows:
                self.cols_spin.setValue(int(board.checkerboard_cols))
                self.rows_spin.setValue(int(board.checkerboard_rows))
            if board.square_size:
                self.sq_spin.setValue(float(board.square_size) * 1000.0)
        self._on_pattern_changed()

    def load_folder(self, folder: Path) -> None:
        """Programmatic folder load (also used by the embedded dock to
        auto-wire cameras/<cam>/intrinsics_data of the open dataset)."""
        folder = Path(folder)
        paths = sorted(p for p in folder.iterdir()
                       if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))
        self.session_dir = folder
        self._add_paths(paths)
        self._load_session()

    def _add_images(self):
        files, _ = QFileDialog.getOpenFileNames(
            self, "Chessboard images", "", "Images (*.png *.jpg *.jpeg *.bmp)")
        self._add_paths([Path(f) for f in files])

    def _add_paths(self, paths):
        known = {e.path for e in self.entries}
        for p in paths:
            if p not in known:
                self.entries.append(_ImageEntry(path=p))
        self._rebuild_list()

    def _remove_selected(self):
        rows = sorted({i.row() for i in self.listw.selectedIndexes()}, reverse=True)
        for r in rows:
            del self.entries[r]
        self._rebuild_list()

    def _rebuild_list(self):
        from PySide6.QtGui import QColor

        roles = {}
        if self._frame_report:
            roles = {name: st for name, _e, st in self._frame_report}
        self.listw.blockSignals(True)
        self.listw.clear()
        for e in self.entries:
            role = roles.get(e.path.name)
            # marker precedence: calibration role (after Calibrate) over
            # plain detection state. x = did not enter the final solve.
            if role == "used":
                mark, color = "✓ ", None
            elif role == "pruned":
                mark, color = "✗ ", QColor("#e07b00")   # auto-pruned (blurred)
            elif role == "excluded":
                mark, color = "✗ ", QColor("#8a8a8a")   # classic fallback
            else:
                mark = {True: "✓ ", False: "✗ ", None: ""}[e.detected]
                color = QColor("#8a8a8a") if e.detected is False else None
            item = QListWidgetItem(f"{mark}{e.path.name}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if e.checked else Qt.Unchecked)
            if color is not None:
                item.setForeground(color)
            if role == "pruned":
                item.setToolTip("Auto-pruned: reprojection error above 2x median "
                                "(kept for display; still checkable).")
            elif role == "excluded":
                item.setToolTip("Excluded from the solve: classic-fallback "
                                "detection (lower corner precision).")
            self.listw.addItem(item)
        self.listw.blockSignals(False)
        self._update_status()

    def _item_changed(self, item):
        row = self.listw.row(item)
        if 0 <= row < len(self.entries):
            self.entries[row].checked = item.checkState() == Qt.Checked
        self._update_status()

    def _toggle_all(self):
        state = self.all_box.checkState() != Qt.Unchecked
        for e in self.entries:
            e.checked = state and e.detected is not False
        self._rebuild_list()

    def _invalidate_detection(self):
        for e in self.entries:
            e.corners = None
            e.objpoints = None
            e.detected = None
        self.result = None
        self._rebuild_list()
        self.status_label.setText("Board changed: detection invalidated - run Detect All.")

    # ---------- work ----------
    def _busy(self, on: bool):
        for b in (self.detect_btn, self.calib_btn, self.prune_btn,
                  self.save_gui_btn, self.save_ost_btn, self.apply_btn):
            b.setEnabled(not on)
        if not on:
            self.apply_btn.setEnabled(bool(self.current_dataset_dir))

    def _detect_all(self):
        spec = self._spec()
        todo = [e for e in self.entries if e.detected is None]
        if not todo:
            self.status_label.setText("Nothing to detect.")
            return
        self._busy(True)
        self.detect_btn.setText("Detecting 0/%d" % len(todo))

        def job(w: _Worker):
            for i, e in enumerate(todo):
                img = cv2.imread(str(e.path), cv2.IMREAD_GRAYSCALE)
                corners = objs = src = None
                if img is not None:
                    corners, objs, src = detect_corners_ex(img, spec)
                    if self.image_size is None:
                        self.image_size = (img.shape[1], img.shape[0])
                w.detected.emit(str(e.path), (corners, objs, src))
                w.progressed.emit(i + 1, len(todo))

        self._worker = _Worker(job)
        self._worker.progressed.connect(
            lambda d, t: self.detect_btn.setText(f"Detecting {d}/{t}"))
        self._worker.detected.connect(self._on_detected)
        self._worker.finished.connect(self._on_detect_done)
        self._worker.start()

    def _on_detected(self, path, payload):
        corners, objs, src = payload
        for e in self.entries:
            if str(e.path) == path:
                e.corners = corners
                e.objpoints = objs
                e.source = src
                e.detected = corners is not None
                if corners is None:
                    e.checked = False

    def _on_detect_done(self):
        self.detect_btn.setText("Detect All")
        self._busy(False)
        self._rebuild_list()
        self._save_session()
        self._refresh_views()

    def _used_entries(self) -> List[_ImageEntry]:
        return [e for e in self.entries if e.checked and e.detected]

    def _calibrate(self):
        used = self._used_entries()
        if len(used) < 4:
            QMessageBox.warning(self, "Intrinsics", "Need at least 4 detected, checked images.")
            return
        spec = self._spec()
        model = self.model_combo.currentText()
        fix_k3 = self.k3_box.isChecked()
        # High-precision subset: when enough SB detections exist, the coarser
        # classic-fallback frames are excluded from the solve (they stay
        # visible and checked - they are display-only here). Mixing them in
        # degrades the calibration measurably.
        precise = "rap" if spec.pattern == "aprilgrid" else "sb"
        sb = [e for e in used if e.source == precise]
        if spec.resolved_extractor() in ("legacy", "rap") and len(sb) >= max(8, len(used) // 2):
            if len(sb) < len(used):
                self.status_label.setText(
                    f"Using the {len(sb)} {precise.upper()}-precision images of {len(used)} "
                    "checked (Stage 1 rejects excluded from the solve).")
            used = sb
        auto_prune = self.autoprune_box.isChecked()
        # Fair holdout for the k3 model selection: a split of the SAME
        # precision (SB) frames. Judging with the classic-fallback frames is
        # biased - they are the blurred/extreme frames SB rejected, and their
        # corner noise makes any richer model look dangerous (measured: they
        # veto a free k3 that an SB-internal holdout accepts at 0.30 vs
        # 0.32 px). After the verdict the winner is retrained on ALL frames.
        if len(used) >= 15:
            holdout_entries = used[::5]
            train_entries = [e for e in used if e not in holdout_entries]
        else:
            holdout_entries = []
            train_entries = used
        # AprilGrid observations are partial grids: each image carries its own
        # object points. The legacy AprilGrid corners are provisional (they are
        # re-extracted with the rough intrinsics below), so selection and the
        # order decision run only on final corners (chessboard, Kalibr AprilGrid).
        april = spec.pattern == "aprilgrid"
        selectable = not (april and spec.resolved_extractor() == "legacy")
        if not selectable:
            holdout_entries, train_entries = [], used

        def objs_of(entries):
            return [e.objpoints for e in entries] if april else None

        holdout_sets = [e.corners for e in holdout_entries]
        holdout_objs = ([np.asarray(e.objpoints, np.float64) for e in holdout_entries]
                        if april else [spec.object_points().astype(np.float64)]
                        * len(holdout_entries))
        full_names = [e.path.name for e in used]
        excluded_classic = [e.path.name for e in self._used_entries() if e not in used]
        size = self.image_size
        self._busy(True)
        self.calib_btn.setText("Calibrating...")

        used_paths = [str(e.path) for e in used]

        def job(w: _Worker):
            try:
                def fit_select(entries, fk3):
                    """Fit, then Stage 2 (iterative median-scaled rejection)."""
                    cs = [e.corners for e in entries]
                    ob = objs_of(entries)
                    names_t = [e.path.name for e in entries]
                    r = calibrate(cs, spec, size, model=model, fix_k3=fk3, object_sets=ob)
                    if auto_prune and selectable:
                        for _ in range(3):
                            means = np.array([p["mean"] for p in r["per_image"]])
                            med = float(np.median(means))
                            keep = means <= PRUNE_FACTOR * med
                            if keep.all() or int(keep.sum()) < 8:
                                break
                            cs = [c for c, k in zip(cs, keep) if k]
                            ob = [o for o, k in zip(ob, keep) if k] if ob is not None else None
                            names_t = [n for n, k in zip(names_t, keep) if k]
                            r = calibrate(cs, spec, size, model=model, fix_k3=fk3,
                                          object_sets=ob)
                    r["auto_pruned"] = len(entries) - len(names_t)
                    r["used_names"] = names_t
                    return r

                res = fit_select(train_entries, fix_k3)
                if fix_k3 and model == "plumb_bob" and selectable and holdout_sets:
                    # Try the free-k3 variant; adopt it only if the held-out
                    # frames confirm it generalises - this is exactly where an
                    # unconstrained r^6 term blows up.
                    alt = fit_select(train_entries, False)

                    def ho_err(r):
                        Km = np.array(r["K"])
                        Dm = np.array((list(r["dist"]) + [0.0] * 5)[:5])
                        es = []
                        for uv, objd in zip(holdout_sets, holdout_objs):
                            okp, rv, tv = cv2.solvePnP(objd, uv.astype(np.float64), Km, Dm)
                            if not okp:
                                continue
                            proj, _ = cv2.projectPoints(objd, rv, tv, Km, Dm)
                            es.append(float(np.linalg.norm(
                                proj.reshape(-1, 2) - uv, axis=1).mean()))
                        return (float(np.mean(es)), float(np.max(es))) if es else (np.inf, np.inf)

                    base_mean, base_max = ho_err(res)
                    alt_mean, alt_max = ho_err(alt)
                    # Adopt free k3 only when the held-out frames show no
                    # degradation, neither in the mean nor on the worst view
                    # (alpha = beta = 1).
                    use_free = alt_mean <= base_mean * 1.0 and alt_max <= base_max * 1.0
                    choice = ("free" if use_free else "fixed")
                    note = f"holdout {alt_mean:.2f} vs {base_mean:.2f} px on {len(holdout_sets)} frames"
                    # Retrain the winner on ALL precision frames (holdout back in).
                    res = fit_select(used, not use_free)
                    res["k3_choice"] = f"{choice} ({note})"
                    res["pruned_names"] = [n for n in full_names
                                           if n not in set(res["used_names"])]
                    res["excluded_names"] = excluded_classic
                if spec.pattern == "aprilgrid" and spec.resolved_extractor() == "legacy":
                    # Stage 2 (kalibr-style): redetect the FULL grid with the
                    # rough intrinsics, then recalibrate on the completed
                    # corners. Two fixed-point iterations.
                    from .feature_detector import detect_aprilgrid_corners
                    from .models import CameraIntrinsics

                    board_cfg = spec.to_board_config()
                    for _ in range(2):
                        Ks = np.array(res["K"]); ds = list(res["dist"]) + [0.0] * 5
                        rough = CameraIntrinsics(fx=Ks[0, 0], fy=Ks[1, 1],
                                                 cx=Ks[0, 2], cy=Ks[1, 2],
                                                 k1=ds[0], k2=ds[1], p1=ds[2], p2=ds[3])
                        c2, o2 = [], []
                        for path in used_paths:
                            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                            uv, ob, _m = detect_aprilgrid_corners(
                                img, board_cfg, camera=rough, complete_grid=True)
                            if uv is not None and len(uv) >= 12:
                                c2.append(uv); o2.append(ob)
                        if len(c2) < 4:
                            break
                        res = calibrate(c2, spec, size, model=model, fix_k3=fix_k3,
                                        object_sets=o2)
                    res["stage2_images"] = len(c2) if spec.pattern == "aprilgrid" else None
            except Exception as exc:  # surfaced in UI
                res = exc
            w.finished_calib.emit(res)

        self._worker = _Worker(job)
        self._worker.finished_calib.connect(self._on_calibrated)
        self._worker.start()

    def _on_calibrated(self, res):
        self.calib_btn.setText("Calibrate")
        self._busy(False)
        if isinstance(res, Exception):
            QMessageBox.critical(self, "Intrinsics", f"Calibration failed:\n{res}")
            return
        self.result = res
        used_names = set(res.get("used_names") or [])
        if used_names:
            idx = 0
            for e in self.entries:
                if e.path.name in used_names:
                    e.calib_index = idx
                    idx += 1
                else:
                    e.calib_index = None
        else:
            for i, e in enumerate(self._used_entries()):
                e.calib_index = i
        # Frame report: EVERY detected frame with its error under the final
        # intrinsics and why it did / did not participate.
        self._frame_report = self._build_frame_report(res)
        self._rebuild_list()
        excl = res.get("excluded_names") or []
        pruned = res.get("pruned_names") or []
        if excl:
            self._log(f"Calibrate: excluded {len(excl)} classic-fallback frames "
                      f"(display only): {', '.join(excl)}")
        if pruned:
            self._log(f"Calibrate: auto-pruned {len(pruned)} high-error frames "
                      f"(> {PRUNE_FACTOR:g}x median): {', '.join(pruned)}")
        if res.get("k3_choice"):
            self._log(f"Calibrate: k3 {res['k3_choice']}")
        if used_names:
            self._log(f"Calibrate: {len(used_names)} frames in the final solve.")
        K = np.array(res["K"])
        d = res["dist"]
        means = [p["mean"] for p in res["per_image"]]
        med = float(np.median(means))
        n_bad = sum(1 for m in means if m > PRUNE_FACTOR * med)
        all_res = [np.linalg.norm(r, axis=1)
                   for p_ in res["per_image"]
                   if len(r := np.asarray(p_.get("residuals", []), float))]
        mean_corner = float(np.concatenate(all_res).mean()) if all_res else float("nan")
        notes = []
        if res.get("stage2_images"):
            notes.append(f"2-stage {res['stage2_images']} imgs")
        if res.get("auto_pruned"):
            notes.append(f"pruned {res['auto_pruned']}")
        if res.get("k3_choice"):
            notes.append(f"k3 {res['k3_choice'].split(' (')[0]}")
        self.res_label.setText(
            f"<b>{mean_corner:.3f} px</b> mean corner · {res['rms']:.3f} RMS<br>"
            f"fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}<br>"
            f"dist [{', '.join(f'{x:.4f}' for x in d[:5])}]<br>"
            f"{len(means)} imgs · {' · '.join(notes)}")
        self.res_label.setToolTip(
            f"mean corner error (MATLAB-comparable): {mean_corner:.4f} px\n"
            f"OpenCV RMS: {res['rms']:.4f} px\n"
            + (f"k3 {res['k3_choice']}" if res.get("k3_choice") else ""))
        note = (f" · {n_bad} above {PRUNE_FACTOR:g}x median - consider Auto-prune"
                if n_bad else "")
        self.status_label.setText(
            f"Calibrated: RMS {res['rms']:.3f} px, median {med:.3f} px{note}")
        self._save_session()
        self._refresh_views()

    def _auto_prune(self):
        if self.result is None:
            QMessageBox.information(self, "Intrinsics", "Calibrate first.")
            return
        for _ in range(3):
            means = [p["mean"] for p in self.result["per_image"]]
            med = float(np.median(means))
            used = self._used_entries()
            bad = [used[i] for i, m in enumerate(means) if m > PRUNE_FACTOR * med]
            if not bad or len(used) - len(bad) < 4:
                break
            for e in bad:
                e.checked = False
            self._rebuild_list()
            used2 = self._used_entries()
            corner_sets = [e.corners for e in used2]
            object_sets = ([e.objpoints for e in used2]
                           if self._spec().pattern == "aprilgrid" else None)
            self.result = calibrate(corner_sets, self._spec(), self.image_size,
                                    model=self.model_combo.currentText(),
                                    fix_k3=self.k3_box.isChecked(),
                                    object_sets=object_sets)
        self._on_calibrated(self.result)

    # ---------- views ----------
    def _show_current(self):
        row = self.listw.currentRow()
        if not (0 <= row < len(self.entries)):
            return
        e = self.entries[row]
        img = cv2.imread(str(e.path), cv2.IMREAD_COLOR)
        if img is None:
            return
        if e.corners is not None:
            if self._pattern() == "aprilgrid":
                for c in e.corners:
                    cv2.circle(img, tuple(np.int32(c)), 5, (0, 255, 0), 2)
            else:
                cols, rows = self.cols_spin.value(), self.rows_spin.value()
                cv2.drawChessboardCorners(img, (cols, rows),
                                          e.corners.reshape(-1, 1, 2), True)
                # Board outline + the four outer corners, so the pose of the
                # board is readable at a glance.
                grid = e.corners.reshape(rows, cols, 2)
                outline = np.vstack([grid[0, :], grid[1:, -1],
                                     grid[-1, ::-1][1:], grid[::-1, 0][1:]])
                cv2.polylines(img, [np.rint(outline).astype(np.int32).reshape(-1, 1, 2)],
                              True, (0, 220, 255), 2, cv2.LINE_AA)
                for pt in (grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]):
                    cv2.circle(img, tuple(np.int32(pt)), 9, (0, 220, 255), 2, cv2.LINE_AA)
            src = {"sb": "SB precision", "classic": "classic fallback",
                   "april": "aprilgrid"}.get(e.source or "", "")
            note = f"{len(e.corners)} corners | {src}"
            if (self.result is not None and e.calib_index is not None
                    and e.calib_index < len(self.result["per_image"])):
                note += f" | residuals x{RESIDUAL_SCALE}"
            (tw, th), _ = cv2.getTextSize(note, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(img, (8, 8), (24 + tw, 20 + th), (30, 30, 30), -1)
            cv2.putText(img, note, (14, 12 + th),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2, cv2.LINE_AA)
            if self.result is not None and e.calib_index is not None \
                    and e.calib_index < len(self.result["per_image"]):
                res = np.array(self.result["per_image"][e.calib_index]["residuals"])
                for c, r in zip(e.corners, res):
                    tip = c + r * RESIDUAL_SCALE
                    cv2.arrowedLine(img, tuple(np.int32(c)), tuple(np.int32(tip)),
                                    (0, 0, 255), 1, cv2.LINE_AA, tipLength=0.25)

        self._set_label_image(self.img_label, img)
        self.tabs.setCurrentIndex(0)

    def eventFilter(self, obj, event):
        # Wheel = zoom, double-click = back to fit, on the Image tab only.
        from PySide6.QtCore import QEvent
        if obj is getattr(self, "_img_scroll", None) and self._current_bgr is not None:
            if event.type() == QEvent.Wheel:
                step = 1.25 if event.angleDelta().y() > 0 else 0.8
                base = self._img_zoom if self._img_zoom else self._fit_scale()
                self._img_zoom = float(np.clip(base * step, 0.1, 6.0))
                self._set_label_image(self.img_label, self._current_bgr)
                return True
            if event.type() == QEvent.MouseButtonDblClick:
                self._img_zoom = None
                self._set_label_image(self.img_label, self._current_bgr)
                return True
        return super().eventFilter(obj, event)

    def _fit_scale(self) -> float:
        if self._current_bgr is None:
            return 1.0
        vw, vh = self._viewport_size()
        h, w = self._current_bgr.shape[:2]
        return min(vw / w, vh / h, 1.0)

    def _viewport_size(self):
        vw = max(self.tabs.width() - 30, 400)
        vh = max(self.tabs.height() - 60, 300)
        return vw, vh

    def _set_label_image(self, label: QLabel, bgr: np.ndarray):
        # Fit to the visible tab area by default; mouse wheel zooms, double
        # click restores the fit (self._img_zoom is None when fitted).
        if label is self.img_label:
            self._current_bgr = bgr
        vw, vh = self._viewport_size()
        h, w = bgr.shape[:2]
        scale = (self._img_zoom if (label is self.img_label and self._img_zoom)
                 else min(vw / w, vh / h, 1.0))
        if scale < 1.0:
            bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)),
                             interpolation=cv2.INTER_AREA)
            h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        label.setPixmap(QPixmap.fromImage(
            QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()))
        label.adjustSize()

    def _set_label_pixmap(self, label: QLabel, pix: QPixmap):
        vw, vh = self._viewport_size()
        if pix.width() > vw or pix.height() > vh:
            pix = pix.scaled(vw, vh, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(pix)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "status_label"):  # guard: resize fires during __init__
            self._refresh_views()
            self._show_current()

    def _refresh_views(self):
        used = self._used_entries()
        if used and self.image_size:
            self._set_label_pixmap(self.cov_label, render_coverage(
                [e.corners for e in used], self.image_size))
        if self.result is not None:
            if self._frame_report:
                names = [n for n, _e, _s in self._frame_report]
                means = [e_ for _n, e_, _s in self._frame_report]
                statuses = [st for _n, _e, st in self._frame_report]
            else:
                names = [e.path.name for e in used][:len(self.result["per_image"])]
                means = [p["mean"] for p in self.result["per_image"]]
                statuses = None
            self._set_label_pixmap(self.err_label,
                                   render_error_chart(names, means, statuses))
            self._set_label_pixmap(self.scatter_label,
                render_residual_scatter(self.result["per_image"], float(self.result["rms"])))
            row = self.listw.currentRow()
            if 0 <= row < len(self.entries):
                img = cv2.imread(str(self.entries[row].path), cv2.IMREAD_COLOR)
                if img is not None:
                    K = np.array(self.result["K"])
                    d = np.array(self.result["dist"], float)
                    if self.result["model"] == "fisheye":
                        und = cv2.fisheye.undistortImage(img, K, d.reshape(-1, 1), Knew=K)
                    else:
                        und = cv2.undistort(img, K, d)
                    both = np.hstack([img, und])
                    cv2.putText(both, "original", (12, 34), cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (0, 255, 0), 2)
                    cv2.putText(both, "undistorted", (img.shape[1] + 12, 34),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                    self._set_label_image(self.und_label, both)

    def _update_status(self):
        n = len(self.entries)
        det = sum(1 for e in self.entries if e.detected)
        used = len(self._used_entries())
        extra = f" · RMS {self.result['rms']:.3f} px" if self.result else ""
        self.status_label.setText(f"{n} images, {det} detected, {used} used{extra}")

    # ---------- compare / export ----------
    def _compare_yaml(self):
        if self.result is None:
            QMessageBox.information(self, "Intrinsics", "Calibrate first.")
            return
        f, _ = QFileDialog.getOpenFileName(self, "Reference YAML", "", "YAML (*.yaml *.yml)")
        if not f:
            return
        ref = load_reference_yaml(f)
        if ref is None:
            QMessageBox.warning(self, "Intrinsics", "Unrecognized YAML format.")
            return
        K = np.array(self.result["K"])
        rows = [("fx", K[0, 0], ref["fx"]), ("fy", K[1, 1], ref["fy"]),
                ("cx", K[0, 2], ref["cx"]), ("cy", K[1, 2], ref["cy"])]
        txt = "\n".join(f"{n}: ours {a:.2f} · ref {b:.2f} · diff {a-b:+.2f}"
                        for n, a, b in rows)
        d = list(self.result["dist"]) + [0.0] * 5
        rd = list(ref["dist"]) + [0.0] * 5
        txt += "\n" + "\n".join(
            f"{n}: ours {d[i]:+.5f} · ref {rd[i]:+.5f}"
            for i, n in enumerate(("k1", "k2", "p1", "p2")))
        # Decisive check, not just parameter deltas: per-image PnP reprojection
        # of the SAME detected corners under each intrinsics set. Lower wins.
        used = self._used_entries()
        spec = self._spec()
        obj_default = spec.object_points().astype(np.float64)

        def holdout(fx, fy, cx, cy, dist):
            Km = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
            Dm = np.array((list(dist) + [0.0] * 5)[:5])
            errs = []
            for e in used:
                if e.corners is None:
                    continue
                obj = (np.asarray(e.objpoints, np.float64)
                       if e.objpoints is not None else obj_default)
                uv = np.asarray(e.corners, np.float64)
                ok, rvec, tvec = cv2.solvePnP(obj, uv, Km, Dm)
                if not ok:
                    continue
                proj, _ = cv2.projectPoints(obj, rvec, tvec, Km, Dm)
                errs.append(float(np.sqrt(np.mean(
                    np.sum((proj.reshape(-1, 2) - uv) ** 2, axis=1)))))
            return np.asarray(errs)

        e_ours = holdout(K[0, 0], K[1, 1], K[0, 2], K[1, 2], self.result["dist"])
        e_ref = holdout(ref["fx"], ref["fy"], ref["cx"], ref["cy"], ref["dist"])
        if len(e_ours) and len(e_ours) == len(e_ref):
            wins = int((e_ours < e_ref).sum())
            verdict = ("statistically equivalent"
                       if abs(e_ours.mean() - e_ref.mean()) < 0.05
                       else ("OURS is better" if e_ours.mean() < e_ref.mean()
                             else "REFERENCE is better"))
            txt += (
                f"\n\nPer-image PnP reprojection on the same {len(e_ours)} images:"
                f"\n  ours: mean {e_ours.mean():.3f} px (median {np.median(e_ours):.3f})"
                f"\n  ref : mean {e_ref.mean():.3f} px (median {np.median(e_ref):.3f})"
                f"\n  ours lower on {wins}/{len(e_ours)} images -> {verdict}"
            )
        QMessageBox.information(self, "Compare with reference", txt)

    def _save_yaml(self, kind: str):
        if self.result is None:
            QMessageBox.information(self, "Intrinsics", "Calibrate first.")
            return
        default = "camera_intrinsics.yaml" if kind == "gui" else "ost.yaml"
        f, _ = QFileDialog.getSaveFileName(self, "Save intrinsics", default, "YAML (*.yaml)")
        if not f:
            return
        data = (result_to_gui_yaml(self.result) if kind == "gui"
                else result_to_ost_yaml(self.result))
        with open(f, "w") as fh:
            yaml.safe_dump(data, fh, sort_keys=False)
        self.status_label.setText(f"Saved {f}")

    def _apply_to_project(self):
        if self.result is None or not self.current_dataset_dir:
            return
        target = Path(self.current_dataset_dir) / "camera_intrinsics.yaml"
        msg = (f"Write intrinsics to\n{target}\n(existing file is backed up as .bak)?")
        if QMessageBox.question(self, "Apply to project", msg) != QMessageBox.Yes:
            return
        if target.exists():
            shutil.copy2(target, target.with_suffix(".yaml.bak"))
        with open(target, "w") as fh:
            yaml.safe_dump(result_to_gui_yaml(self.result), fh, sort_keys=False)
        self.status_label.setText(f"Applied to {target} (backup written). "
                                  "Reopen the dataset to reload intrinsics.")

    # ---------- session ----------
    def _save_session(self):
        if not self.session_dir:
            return
        data = dict(
            board=dict(pattern=self._pattern(),
                       cols=self.cols_spin.value(), rows=self.rows_spin.value(),
                       square_mm=self.sq_spin.value(),
                       tag_spacing=self.spacing_spin.value(),
                       detector=self.det_combo.currentIndex(),
                       extractor=self._extractor()),
            model=self.model_combo.currentText(), fix_k3=self.k3_box.isChecked(),
            images=[dict(name=e.path.name, checked=e.checked, detected=e.detected)
                    for e in self.entries if e.path.parent == self.session_dir],
            result=(dict(K=self.result["K"], dist=self.result["dist"],
                         rms=self.result["rms"], model=self.result["model"])
                    if self.result else None),
        )
        try:
            with open(self.session_dir / SESSION_FILE, "w") as fh:
                json.dump(data, fh, indent=1)
        except OSError:
            pass

    def _load_session(self):
        if not self.session_dir:
            return
        p = self.session_dir / SESSION_FILE
        if not p.exists():
            return
        try:
            data = json.load(open(p))
        except (OSError, json.JSONDecodeError):
            return
        b = data.get("board", {})
        self.pattern_combo.setCurrentIndex(1 if b.get("pattern") == "aprilgrid" else 0)
        self.spacing_spin.setValue(float(b.get("tag_spacing", 0.3)))
        self.cols_spin.setValue(int(b.get("cols", 11)))
        self.rows_spin.setValue(int(b.get("rows", 8)))
        self.sq_spin.setValue(float(b.get("square_mm", 45.0)))
        self.det_combo.setCurrentIndex(int(b.get("detector", 0)))
        # sessions written before the Kalibr extractor existed used the legacy one
        self.ext_combo.setCurrentIndex(
            {"auto": 0, "kalibr": 1, "legacy": 2}.get(b.get("extractor", "legacy"), 2))
        checked = {i["name"]: i["checked"] for i in data.get("images", [])}
        for e in self.entries:
            if e.path.name in checked:
                e.checked = bool(checked[e.path.name])
        self._rebuild_list()
        self.status_label.setText("Session restored (checkboxes + board spec).")
