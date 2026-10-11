# Rig-C cross-session evaluation (Table II)

Rig-C is the handheld platform with two FLIR cameras (cam0 = L, cam1 = R; 1440 x 1080, 113 deg), an Ouster LiDAR with integrated IMU, and a 3 x 3 AprilGrid (0.115 m tags, spacing ratio 0.3). Two calibration sessions were recorded on different days: D1 (570 frames) and D2 (416 frames). Every method is estimated on one session and evaluated on the other with the intrinsics fixed and the board pose re-estimated on every view; the common reference is the raw Kalibr corners of the evaluation session. A view is excluded as a gross error when its error under Kalibr, ours-M2 or ours (Full) exceeds three times the median of that method.

The corner observations of both sessions ship in `corners_D{1,2}_cam{0,1}.json` (raw Kalibr corners and the corners of the proposed extractor for every frame), so Table II is reproduced without the bags.

```bash
cd experiments/rigc
python flir_full.py cam0          # about 8 min per camera; writes flir_full_cam0.json and intrinsics/cam0_*.yaml
python flir_full.py cam1
python table2_rigc.py             # rows Kalibr, ours-M2, ours (Full) -> table2_rigc.json
python ablation_rigc.py           # all rows of Table II -> ablation_rigc.json, intrinsics/*_woIS.yaml, *_woOS.yaml
```

`table2_rigc.py` and `ablation_rigc.py` read `flir_full_cam*.json`, or the shipped copies in `figures/data/` when `flir_full.py` has not been run. Expected output of `ablation_rigc.py`:

```
method          L D1->D2    L D2->D1    R D1->D2    R D2->D1
ROS-calib              x           x           x           x
mrcal                  x           x           x           x
Kalibr             0.484       0.356       0.406       0.380
ours w/o IS        0.517       0.287       0.367       0.325
ours w/o OS        0.369       0.354       0.410       0.393
ours-M2            1.025       0.513       0.731       0.536
ours (Full)        0.242       0.197       0.244       0.244
```

ROS-calib and mrcal do not support the AprilGrid target. `flir_full.py` also prints the difference of the estimated parameters between the two sessions (the focal-length repeatability of Section V-E).

## Intrinsics files

`intrinsics/<cam>_<session>_<method>.yaml` holds the intrinsics estimated on `<session>` (radtan, `k1 k2 p1 p2 k3`, Kalibr and vikit keys). They are the input of the VIO experiment (`experiments/vio/`).

| `<method>` | row of Table II | description |
|---|---|---|
| `kalibr` | Kalibr | Kalibr's calibration of the session (`data/rigc/kalibr_D*/calib-camchain.yaml`), pinhole-radtan |
| `RAP` | ours (Full) | Stage 1, Stage 2 within each candidate order, order selection, final fit |
| `woIS` | ours w/o IS | order selection on every frame, without Stage 1 and Stage 2 |
| `woOS` | ours w/o OS | Stage 1 and Stage 2, order fixed at M2 |
| `M2-R` | ours-M2 | proposed extractor, every frame, M2, no selection |
| `M2-K`, `M3-K`, `M2-K-3sigma`, `RAP-K` | not in the paper | M2 / M3 on the raw Kalibr corners, M2 with Kalibr-style 3-sigma corner rejection, Stage 2 and order selection on the raw Kalibr corners |

## Further scripts

`flir_compare.py`, `flir_refine.py`, `diag_extractor.py` and `diag2.py` are diagnostics of the AprilGrid extractor on the calibration bags (`RIGC_DIR`); their results are not reported in the paper.
