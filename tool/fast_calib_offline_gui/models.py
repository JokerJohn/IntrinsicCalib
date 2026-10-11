from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


STATUS_NEW = "New"
STATUS_IMAGE_READY = "Image Ready"
STATUS_IMAGE_ROI_READY = "Image ROI Ready"
STATUS_IMAGE_DETECTED = "Image Detected"
STATUS_LIDAR_ROI_READY = "LiDAR ROI Ready"
STATUS_PLANE_READY = "Plane Ready"
STATUS_LIDAR_DETECTED = "LiDAR Detected"
STATUS_CENTERS_READY = "Centers Ready"
STATUS_POSE_ESTIMATED = "Pose Estimated"
STATUS_NOT_CHECKED = "Not Checked"
STATUS_AUTO_DETECTED = "Auto Detected"
STATUS_NEEDS_REVIEW = "Needs Review"
STATUS_MANUALLY_EDITED = "Manually Edited"
STATUS_READY = "Ready"
STATUS_OPTIMIZED = "Optimized"
STATUS_FAILED = "Failed"
STATUS_EXCLUDED = "Excluded"
# Checkerboard boards are annotation-only: they carry a camera/LiDAR ROI but
# never run the four-hole detection pipeline, so they get their own statuses.
# STATUS_ROI_READY = both ROIs drawn, awaiting the operator's manual confirm;
# STATUS_ANNOTATED = confirmed via Mark Sample Ready.
STATUS_ROI_READY = "ROI Ready"
STATUS_ANNOTATED = "Annotated"

# Board (calibration target) types. "four_hole" keeps the original detection +
# calibration pipeline; "checkerboard" only stores ROI annotations. Older
# projects without the field are treated as four_hole for full compatibility.
BOARD_TYPE_FOUR_HOLE = "four_hole"
BOARD_TYPE_CHECKERBOARD = "checkerboard"
BOARD_TYPE_APRILGRID = "aprilgrid"
BOARD_TYPES = {BOARD_TYPE_FOUR_HOLE, BOARD_TYPE_CHECKERBOARD, BOARD_TYPE_APRILGRID}
# Board types that use the annotation-only workflow (draw image + LiDAR ROI,
# confirm with Mark Ready; no four-hole detection/solve). Checkerboard and
# aprilgrid share this flow and differ only in the Detect overlay.
ANNOTATION_ONLY_BOARD_TYPES = {BOARD_TYPE_CHECKERBOARD, BOARD_TYPE_APRILGRID}

FEATURE_SOURCE_NONE = "None"
FEATURE_SOURCE_DETECTED = "Detected"
FEATURE_SOURCE_REFIT = "Refit"
FEATURE_SOURCE_MANUAL_OVERRIDE = "Manual Override"
FEATURE_SOURCE_NUMERIC_OVERRIDE = "Numeric Override"

FEATURE_SOURCES = {
    FEATURE_SOURCE_NONE,
    FEATURE_SOURCE_DETECTED,
    FEATURE_SOURCE_REFIT,
    FEATURE_SOURCE_MANUAL_OVERRIDE,
    FEATURE_SOURCE_NUMERIC_OVERRIDE,
}

ALL_STATUSES = {
    STATUS_NEW,
    STATUS_IMAGE_READY,
    STATUS_IMAGE_ROI_READY,
    STATUS_IMAGE_DETECTED,
    STATUS_LIDAR_ROI_READY,
    STATUS_PLANE_READY,
    STATUS_LIDAR_DETECTED,
    STATUS_CENTERS_READY,
    STATUS_POSE_ESTIMATED,
    STATUS_NOT_CHECKED,
    STATUS_AUTO_DETECTED,
    STATUS_NEEDS_REVIEW,
    STATUS_MANUALLY_EDITED,
    STATUS_READY,
    STATUS_OPTIMIZED,
    STATUS_FAILED,
    STATUS_EXCLUDED,
    STATUS_ROI_READY,
    STATUS_ANNOTATED,
}


def _matrix4_identity() -> List[List[float]]:
    return np.eye(4, dtype=float).tolist()


def ensure_point_list(values: Optional[Sequence[Sequence[float]]], width: int = 3) -> List[List[float]]:
    if not values:
        return []
    points: List[List[float]] = []
    for row in values:
        if len(row) != width:
            raise ValueError(f"Expected point width {width}, got {len(row)}")
        points.append([float(v) for v in row])
    return points


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0

    def camera_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def dist_coeffs(self) -> np.ndarray:
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype=np.float64)

    def to_dict(self) -> Dict[str, float]:
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "k1": self.k1,
            "k2": self.k2,
            "p1": self.p1,
            "p2": self.p2,
            "k3": self.k3,
        }


@dataclass
class BoardConfig:
    # Dataset-level default target type. Individual samples may override it via
    # SampleAnnotation.board_type; this only seeds freshly loaded samples.
    board_type: str = BOARD_TYPE_FOUR_HOLE
    marker_size: float = 0.20
    delta_width_qr_center: float = 0.55
    delta_height_qr_center: float = 0.35
    delta_width_circles: float = 0.50
    delta_height_circles: float = 0.40
    circle_radius: float = 0.12
    min_detected_markers: int = 3
    # Checkerboard geometry (only used when board_type == checkerboard). The two
    # counts are interior corners; detection also tries the squares-count
    # interpretation as a fallback. square_size is metres, for reference only.
    checkerboard_cols: int = 0
    checkerboard_rows: int = 0
    square_size: float = 0.0
    # AprilGrid geometry (kalibr convention; used when board_type == aprilgrid).
    # tag_spacing is the RATIO of gap to tag size (kalibr tagSpacing).
    tag_rows: int = 3
    tag_cols: int = 3
    tag_size: float = 0.115
    tag_spacing: float = 0.3
    lidar_roi: Dict[str, float] = field(
        default_factory=lambda: {
            "x_min": 2.0,
            "x_max": 5.0,
            "y_min": -4.0,
            "y_max": 4.0,
            "z_min": 0.0,
            "z_max": 2.0,
        }
    )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "board_type": self.board_type,
            "marker_size": self.marker_size,
            "delta_width_qr_center": self.delta_width_qr_center,
            "delta_height_qr_center": self.delta_height_qr_center,
            "delta_width_circles": self.delta_width_circles,
            "delta_height_circles": self.delta_height_circles,
            "circle_radius": self.circle_radius,
            "min_detected_markers": self.min_detected_markers,
            "checkerboard_cols": self.checkerboard_cols,
            "checkerboard_rows": self.checkerboard_rows,
            "square_size": self.square_size,
            "tag_rows": self.tag_rows,
            "tag_cols": self.tag_cols,
            "tag_size": self.tag_size,
            "tag_spacing": self.tag_spacing,
            "lidar_roi": dict(self.lidar_roi),
        }


@dataclass
class SampleAnnotation:
    # Per-sample target type. Checkerboard samples only capture the two ROIs
    # below (image_roi_quad + lidar_roi) and skip the four-hole pipeline.
    board_type: str = BOARD_TYPE_FOUR_HOLE
    image_roi_quad: Optional[List[List[float]]] = None
    aruco_markers: List[Dict[str, Any]] = field(default_factory=list)
    image_centers_cam: List[List[float]] = field(default_factory=list)
    image_centers_uv: List[List[float]] = field(default_factory=list)
    # ===== BEGIN CHANGE: persist visual feature layers =====
    # These records are intentionally small: circles are four lightweight
    # overlay primitives and plane points are a sampled diagnostic layer. They
    # make GUI review and exported reports reproducible after reopening a
    # project without re-running feature detection.
    image_hole_circles: List[Dict[str, Any]] = field(default_factory=list)
    lidar_plane_points: List[List[float]] = field(default_factory=list)
    # ===== END CHANGE: persist visual feature layers =====
    lidar_holes: List[Dict[str, Any]] = field(default_factory=list)
    lidar_centers: List[List[float]] = field(default_factory=list)
    lidar_roi: Optional[Dict[str, float]] = None
    lidar_plane_inliers: int = 0
    lidar_filtered_points: int = 0
    provisional_transform: Optional[List[List[float]]] = None
    residuals_px: List[float] = field(default_factory=list)
    residuals_m: List[float] = field(default_factory=list)
    included: bool = True
    locked: bool = False
    # True when a batch detection unchecked this sample automatically because
    # its detection failed. Lets a later successful detection re-check it,
    # while never overriding a deliberate manual uncheck.
    auto_excluded: bool = False
    status: str = STATUS_NEW
    message: str = ""
    initial_transform: List[List[float]] = field(default_factory=_matrix4_identity)
    manual_edit_count: int = 0
    image_feature_source: str = FEATURE_SOURCE_NONE
    lidar_feature_source: str = FEATURE_SOURCE_NONE
    # Checkerboard / AprilGrid board detection (image side). Persisted so the
    # readiness state, the plane-based extrinsic solve and the ROI projection
    # survive a project reload. Empty for four-hole datasets.
    board_corners_uv: List[List[float]] = field(default_factory=list)
    board_points_xyz: List[List[float]] = field(default_factory=list)
    # Multi-camera support (MULTICAM_DESIGN.md): per-camera snapshots of the
    # image-side fields. The live fields above always describe the ACTIVE
    # camera; switching cameras stores them into image_banks[old] and loads
    # image_banks[new]. LiDAR-side fields are shared and never banked.
    image_banks: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    # image-side (per camera) field names; everything else is shared.
    IMAGE_BANK_FIELDS = (
        "image_roi_quad", "aruco_markers", "image_centers_cam",
        "image_centers_uv", "image_hole_circles", "image_feature_source",
        "provisional_transform", "residuals_px", "residuals_m",
        "status", "message", "board_corners_uv", "board_points_xyz",
    )

    def snapshot_image_side(self) -> Dict[str, Any]:
        import copy
        return {k: copy.deepcopy(getattr(self, k)) for k in self.IMAGE_BANK_FIELDS}

    def load_image_side(self, bank: Optional[Dict[str, Any]]) -> None:
        import copy
        if not bank:
            # fresh camera: reset image-side fields to defaults
            self.image_roi_quad = None
            self.aruco_markers = []
            self.image_centers_cam = []
            self.image_centers_uv = []
            self.image_hole_circles = []
            self.image_feature_source = FEATURE_SOURCE_NONE
            self.provisional_transform = None
            self.residuals_px = []
            self.residuals_m = []
            self.board_corners_uv = []
            self.board_points_xyz = []
            self.message = ""
            self.set_status_from_centers()
            return
        for k in self.IMAGE_BANK_FIELDS:
            if k in bank:
                setattr(self, k, copy.deepcopy(bank[k]))

    def switch_camera_bank(self, old_cam: str, new_cam: str) -> None:
        if old_cam == new_cam:
            return
        self.image_banks[old_cam] = self.snapshot_image_side()
        self.load_image_side(self.image_banks.get(new_cam))

    def is_checkerboard(self) -> bool:
        # Historical name; true for every annotation-only board type.
        return self.board_type in ANNOTATION_ONLY_BOARD_TYPES

    def has_board_detection(self) -> bool:
        """Checkerboard / AprilGrid image-side detection present."""
        return len(self.board_corners_uv) >= 8

    def image_side_ready(self) -> bool:
        """Annotation-only boards: the camera side is settled either by a
        successful board detection or by a manually drawn ROI. Requiring the
        ROI unconditionally was wrong - many frames detect without one."""
        return self.has_board_detection() or bool(self.image_roi_quad)

    def has_both_rois(self) -> bool:
        return self.image_side_ready() and bool(self.lidar_roi)

    def has_required_centers(self) -> bool:
        return len(self.image_centers_cam) == 4 and len(self.lidar_centers) == 4

    def has_image_detection(self) -> bool:
        return len(self.image_centers_cam) == 4 and len(self.image_centers_uv) == 4

    def has_lidar_detection(self) -> bool:
        return len(self.lidar_centers) == 4

    def set_status_from_centers(self) -> None:
        if not self.included:
            self.status = STATUS_EXCLUDED
        elif self.board_type in ANNOTATION_ONLY_BOARD_TYPES:
            # Annotation-only. Both ROIs present is "ready to confirm"; the
            # operator promotes it to Annotated manually (Mark Sample Ready).
            # An already-confirmed sample stays Annotated while both ROIs remain,
            # so include/exclude toggles and reloads do not drop the confirmation.
            if self.status == STATUS_ANNOTATED and self.has_both_rois():
                self.status = STATUS_ANNOTATED
            elif self.has_both_rois():
                self.status = STATUS_ROI_READY
            elif self.image_roi_quad or self.lidar_roi:
                self.status = STATUS_NEEDS_REVIEW
            else:
                self.status = STATUS_NEW
        elif self.status == STATUS_READY and self.has_required_centers():
            self.status = STATUS_READY
        elif self.has_required_centers() and self.provisional_transform is not None:
            self.status = STATUS_POSE_ESTIMATED
        elif self.has_required_centers():
            self.status = STATUS_CENTERS_READY
        elif self.has_image_detection():
            self.status = STATUS_IMAGE_READY
        elif self.has_lidar_detection():
            self.status = STATUS_LIDAR_DETECTED
        elif self.lidar_plane_inliers > 0:
            self.status = STATUS_PLANE_READY
        elif self.image_roi_quad:
            self.status = STATUS_IMAGE_ROI_READY
        elif self.lidar_roi:
            self.status = STATUS_LIDAR_ROI_READY
        elif self.image_centers_cam or self.lidar_centers:
            self.status = STATUS_NEEDS_REVIEW
        else:
            self.status = STATUS_NEW

    def to_dict(self) -> Dict[str, Any]:
        if self.board_type in ANNOTATION_ONLY_BOARD_TYPES:
            # Annotation-only: persist just the two ROI regions and the
            # confirmation state. Detection/plane layers are transient viewing
            # aids and are intentionally NOT saved.
            return {
                "board_type": self.board_type,
                "image_roi_quad": self.image_roi_quad,
                "lidar_roi": self.lidar_roi,
                "status": self.status,
                "message": self.message,
                "included": self.included,
                "locked": self.locked,
                "auto_excluded": self.auto_excluded,
                "initial_transform": self.initial_transform,
                "manual_edit_count": self.manual_edit_count,
                "board_corners_uv": self.board_corners_uv,
                "board_points_xyz": self.board_points_xyz,
                "lidar_plane_points": self.lidar_plane_points,
                "lidar_plane_inliers": self.lidar_plane_inliers,
                "image_banks": self.image_banks,
            }
        return {
            "board_type": self.board_type,
            "image_roi_quad": self.image_roi_quad,
            "aruco_markers": self.aruco_markers,
            "image_centers_cam": self.image_centers_cam,
            "image_centers_uv": self.image_centers_uv,
            "image_hole_circles": self.image_hole_circles,
            "lidar_plane_points": self.lidar_plane_points,
            "lidar_holes": self.lidar_holes,
            "lidar_centers": self.lidar_centers,
            "lidar_roi": self.lidar_roi,
            "lidar_plane_inliers": self.lidar_plane_inliers,
            "lidar_filtered_points": self.lidar_filtered_points,
            "provisional_transform": self.provisional_transform,
            "residuals_px": self.residuals_px,
            "residuals_m": self.residuals_m,
            "included": self.included,
            "locked": self.locked,
            "auto_excluded": self.auto_excluded,
            "status": self.status,
            "message": self.message,
            "initial_transform": self.initial_transform,
            "manual_edit_count": self.manual_edit_count,
            "image_feature_source": self.image_feature_source,
            "lidar_feature_source": self.lidar_feature_source,
            "board_corners_uv": self.board_corners_uv,
            "board_points_xyz": self.board_points_xyz,
            "image_banks": self.image_banks,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SampleAnnotation":
        ann = cls()
        ann.board_type = data.get("board_type", BOARD_TYPE_FOUR_HOLE)
        if ann.board_type not in BOARD_TYPES:
            ann.board_type = BOARD_TYPE_FOUR_HOLE
        image_roi = data.get("image_roi_quad")
        ann.image_roi_quad = ensure_point_list(image_roi, 2) if image_roi else None
        markers = data.get("aruco_markers", [])
        ann.aruco_markers = markers if isinstance(markers, list) else []
        ann.image_centers_cam = ensure_point_list(data.get("image_centers_cam"), 3)
        ann.image_centers_uv = ensure_point_list(data.get("image_centers_uv"), 2)
        ann.image_hole_circles = []
        for item in data.get("image_hole_circles", []):
            if not isinstance(item, dict) or not {"u", "v", "radius_px"} <= set(item):
                continue
            record: Dict[str, Any] = {
                "u": float(item["u"]),
                "v": float(item["v"]),
                "radius_px": float(item["radius_px"]),
            }
            contour = item.get("contour_uv", [])
            if isinstance(contour, list):
                record["contour_uv"] = ensure_point_list(contour, 2)
            ann.image_hole_circles.append(record)
        ann.lidar_plane_points = ensure_point_list(data.get("lidar_plane_points"), 3)
        holes = data.get("lidar_holes", [])
        ann.lidar_holes = holes if isinstance(holes, list) else []
        ann.lidar_centers = ensure_point_list(data.get("lidar_centers"), 3)
        ann.lidar_roi = data.get("lidar_roi")
        ann.lidar_plane_inliers = int(data.get("lidar_plane_inliers", 0))
        ann.lidar_filtered_points = int(data.get("lidar_filtered_points", 0))
        ann.provisional_transform = data.get("provisional_transform")
        ann.residuals_px = [float(v) for v in data.get("residuals_px", [])]
        ann.residuals_m = [float(v) for v in data.get("residuals_m", [])]
        ann.included = bool(data.get("included", True))
        ann.locked = bool(data.get("locked", False))
        ann.auto_excluded = bool(data.get("auto_excluded", False))
        ann.status = data.get("status", STATUS_NEW)
        if ann.status not in ALL_STATUSES:
            ann.status = STATUS_NEW
        if ann.status == STATUS_NOT_CHECKED:
            ann.status = STATUS_NEW
        ann.message = data.get("message", "")
        ann.initial_transform = data.get("initial_transform", _matrix4_identity())
        ann.manual_edit_count = int(data.get("manual_edit_count", 0))
        ann.image_feature_source = data.get("image_feature_source", FEATURE_SOURCE_NONE)
        if ann.image_feature_source not in FEATURE_SOURCES:
            ann.image_feature_source = FEATURE_SOURCE_NONE
        ann.lidar_feature_source = data.get("lidar_feature_source", FEATURE_SOURCE_NONE)
        if ann.lidar_feature_source not in FEATURE_SOURCES:
            ann.lidar_feature_source = FEATURE_SOURCE_NONE
        ann.board_corners_uv = ensure_point_list(data.get("board_corners_uv"), 2)
        ann.board_points_xyz = ensure_point_list(data.get("board_points_xyz"), 3)
        banks = data.get("image_banks")
        ann.image_banks = banks if isinstance(banks, dict) else {}
        return ann


@dataclass
class CalibrationSample:
    sample_id: str
    sample_dir: Path
    image_path: Optional[Path]
    cloud_path: Optional[Path]
    meta_path: Optional[Path]
    # v2 multi-camera: camera name -> image path. For v1 datasets this holds
    # {"cam0": image_path}. image_path above always tracks the active camera.
    image_paths: Dict[str, Optional[Path]] = field(default_factory=dict)
    meta: Dict[str, Any] = field(default_factory=dict)
    valid: bool = False
    validation_message: str = ""
    annotation: SampleAnnotation = field(default_factory=SampleAnnotation)

    def to_project_dict(self, dataset_root: Path) -> Dict[str, Any]:
        return {
            "sample_id": self.sample_id,
            "sample_dir": str(self.sample_dir.relative_to(dataset_root)),
            "image_path": str(self.image_path.relative_to(dataset_root)) if self.image_path else None,
            "cloud_path": str(self.cloud_path.relative_to(dataset_root)) if self.cloud_path else None,
            "meta_path": str(self.meta_path.relative_to(dataset_root)) if self.meta_path else None,
            "meta": self.meta,
            "valid": self.valid,
            "validation_message": self.validation_message,
            "annotation": self.annotation.to_dict(),
        }


@dataclass
class DatasetProject:
    dataset_root: Path
    samples: List[CalibrationSample]
    camera: CameraIntrinsics
    board: BoardConfig
    warnings: List[str] = field(default_factory=list)
    project_path: Optional[Path] = None
    result_root: Optional[Path] = None
    # v2 multi-camera (MULTICAM_DESIGN.md): all cameras and the active one.
    # `camera` above always equals cameras[active_camera].
    cameras: Dict[str, CameraIntrinsics] = field(default_factory=dict)
    active_camera: str = "cam0"

    def camera_names(self) -> List[str]:
        return list(self.cameras) if self.cameras else ["cam0"]

    def is_multicam(self) -> bool:
        return len(self.cameras) > 1

    def set_active_camera(self, cam: str) -> None:
        """Bank-swap every sample's image-side annotation to `cam` and point
        project.camera / sample.image_path at the new camera. LiDAR-side
        annotations are shared and untouched."""
        if cam == self.active_camera or cam not in self.cameras:
            return
        old = self.active_camera
        for sample in self.samples:
            sample.annotation.switch_camera_bank(old, cam)
            if sample.image_paths:
                sample.image_path = sample.image_paths.get(cam)
        self.camera = self.cameras[cam]
        self.active_camera = cam

    def sample_by_id(self, sample_id: str) -> Optional[CalibrationSample]:
        for sample in self.samples:
            if sample.sample_id == sample_id:
                return sample
        return None

    def ready_samples(self) -> List[CalibrationSample]:
        return [
            sample
            for sample in self.samples
            if sample.valid
            and sample.annotation.included
            and sample.annotation.has_required_centers()
            and sample.annotation.status == STATUS_READY
        ]


@dataclass
class CalibrationResult:
    transform: List[List[float]]
    rmse_m: float
    residuals_m: List[float]
    sample_ids: List[str]
    output_dir: Optional[Path] = None

    @property
    def rotation(self) -> np.ndarray:
        return np.asarray(self.transform, dtype=float)[:3, :3]

    @property
    def translation(self) -> np.ndarray:
        return np.asarray(self.transform, dtype=float)[:3, 3]
