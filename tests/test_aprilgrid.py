"""AprilGrid extraction on one Rig-C frame (session D2, cam0, frame 337).

The extractor decodes the tags with Kalibr's ethz_apriltag2 (compiled with g++
on first use) and refines every corner with the saddle-point refinement of
Section III-A.  The result is compared with the corner cache that the Table II
scripts use (experiments/rigc/corners_D2_cam0.json)."""
import json
import os
import shutil

import cv2
import numpy as np
import pytest

from conftest import ROOT

FRAME = os.path.join(ROOT, "data", "rigc", "frames", "D2_cam0_0337.png")
CACHE = os.path.join(ROOT, "experiments", "rigc", "corners_D2_cam0.json")
INDEX = 337

pytestmark = [
    pytest.mark.skipif(not os.path.exists(FRAME), reason="Rig-C frame not in the repository"),
    pytest.mark.skipif(shutil.which("g++") is None, reason="g++ is required to build the Kalibr extractor"),
]


@pytest.fixture(scope="module")
def cached():
    return next(o for o in json.load(open(CACHE)) if o["i"] == INDEX)


def test_aprilgrid_rap_extractor(cached):
    from intrinsic_calib.camera_intrinsics import BoardSpec, detect_corners_ex
    gray = cv2.imread(FRAME, cv2.IMREAD_GRAYSCALE)
    spec = BoardSpec(pattern="aprilgrid", cols=3, rows=3, square_mm=115.0, tag_spacing=0.3)
    uv, obj, src = detect_corners_ex(gray, spec)
    assert src == "rap"                      # accepted by Stage 1
    assert uv.shape == (36, 2) and obj.shape == (36, 3)   # 9 complete tags
    assert cached["src"] == "rap"
    np.testing.assert_allclose(uv, np.array(cached["uvR"]), atol=1e-3)
    np.testing.assert_allclose(obj, np.array(cached["objR"]), atol=1e-9)


def test_aprilgrid_kalibr_extractor(cached):
    from intrinsic_calib.kalibr_extractor import detect_aprilgrid_kalibr
    gray = cv2.imread(FRAME, cv2.IMREAD_GRAYSCALE)
    uv, obj, _msg = detect_aprilgrid_kalibr(gray, 3, 3, 0.115, 0.3)
    assert uv is not None and len(uv) == 36
    np.testing.assert_allclose(uv, np.array(cached["uvK"]), atol=1e-3)
