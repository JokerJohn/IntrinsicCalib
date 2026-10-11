"""Calibration-target and camera-parameter containers used by the tool."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

import numpy as np

BOARD_TYPE_CHECKERBOARD = "checkerboard"
BOARD_TYPE_APRILGRID = "aprilgrid"


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
    board_type: str = BOARD_TYPE_CHECKERBOARD
    # Checkerboard: interior corner counts and square size [m].
    checkerboard_cols: int = 0
    checkerboard_rows: int = 0
    square_size: float = 0.0
    # AprilGrid (Kalibr convention): tag_spacing is the ratio of gap to tag size.
    tag_rows: int = 3
    tag_cols: int = 3
    tag_size: float = 0.115
    tag_spacing: float = 0.3

    def to_dict(self) -> Dict[str, Any]:
        return {
            "board_type": self.board_type,
            "checkerboard_cols": self.checkerboard_cols,
            "checkerboard_rows": self.checkerboard_rows,
            "square_size": self.square_size,
            "tag_rows": self.tag_rows,
            "tag_cols": self.tag_cols,
            "tag_size": self.tag_size,
            "tag_spacing": self.tag_spacing,
        }
