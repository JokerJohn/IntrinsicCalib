"""Rig-A: the full pipeline with the default settings reproduces the numbers
of the paper (Section V-B, V-C)."""
import json
import os

import numpy as np
import pytest

from conftest import RIG_A
import rap_headless as H
from fast_calib_offline_gui.camera_intrinsics import BoardSpec


def _spec():
    b = json.load(open(os.path.join(RIG_A, ".fastcalib_intrinsics.json")))["board"]
    return BoardSpec(pattern=b["pattern"], cols=b["cols"], rows=b["rows"],
                     square_mm=b["square_mm"], detector="auto", extractor="legacy")


@pytest.fixture(scope="module")
def detected():
    spec = _spec()
    entries, size = H.detect_all(RIG_A, spec)
    return spec, entries, size


@pytest.fixture(scope="module")
def result(detected):
    spec, entries, size = detected
    return H.run_rap(entries, spec, size)


def test_all_images_detected(detected):
    _spec_, entries, size = detected
    assert len(entries) == 39
    assert tuple(size) == (1920, 1080)


def test_stage1_rejects_12_of_39(result):
    t = result["trace"]
    assert t["n_input"] == 39
    assert len(t["stage1_dropped"]) == 12
    assert t["n_after_stage1"] == 27


def test_stage2_rejects_7(result):
    t = result["trace"]
    assert len(t["stage2_dropped"]) == 7
    assert t["n_final"] == 20


def test_selected_order_is_m3(result):
    o = result["trace"]["order"]
    assert o["chosen"].startswith("M3")
    # held-out validation errors of both candidates (Section V-C)
    assert o["M2_val_mean"] == pytest.approx(0.316, abs=0.001)
    assert o["M3_val_mean"] == pytest.approx(0.299, abs=0.001)
    assert o["M2_val_max"] == pytest.approx(0.590, abs=0.001)
    assert o["M3_val_max"] == pytest.approx(0.554, abs=0.001)


def test_calibration_set_mean_corner_error(result):
    mean = float(np.mean([p["mean"] for p in result["per_image"]]))
    assert mean == pytest.approx(0.143, abs=0.005)


def test_determinism(detected, result):
    """Two runs, from detection onwards, return identical intrinsics."""
    spec, entries, size = detected
    entries2, size2 = H.detect_all(RIG_A, spec)
    assert size2 == size
    assert [e["name"] for e in entries2] == [e["name"] for e in entries]
    for a, b in zip(entries, entries2):
        assert np.array_equal(a["corners"], b["corners"])
    again = H.run_rap(entries2, spec, size2)
    assert np.array_equal(np.array(again["K"]), np.array(result["K"]))
    assert np.array_equal(np.array(again["dist"]), np.array(result["dist"]))
    assert again["used_names"] == result["used_names"]
