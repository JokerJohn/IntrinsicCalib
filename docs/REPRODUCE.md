# Usage and Reproduction

## Optional components

The following components are required only by the experiments that name them.

| component | used by |
|---|---|
| ROS Noetic with `camera_calibration` (`/usr/bin/python3`) | ROS-calib baseline (Table I, consistency checks) |
| mrcal (`sudo apt install mrcal`) | mrcal baseline (Table I, Rig-D intrinsics) |
| evo (`evo_ape`, `evo_rpe`) | Table III |
| ROS Noetic and OpenVINS | rerunning the VIO experiments ([experiments/vio/README.md](../experiments/vio/README.md)) |
| Docker image of Kalibr (`kalibr:noetic`) | Kalibr on Rig-A for the Rig-D intrinsics (`experiments/vio/rigd/scripts/run_kalibr_all39.py`) |

Every script resolves the tool and the data relative to the repository. The following environment variables override the defaults.

| variable | meaning | default |
|---|---|---|
| `CALIB_TOOL_DIR` | directory that contains `intrinsic_calib/` | `tool/` |
| `RIG_A_DIR`, `RIG_B_DIR` | Rig-A and Rig-B image folders | `data/rig_a`, `data/rig_b` |
| `RIGC_DIR` | directory of the Rig-C and Rig-D bags ([data/rigc/MANIFEST.md](../data/rigc/MANIFEST.md)) | `data/rigc` |
| `VIO_WS` | catkin workspace of OpenVINS | required by the VIO scripts |
| `EVO_BIN` | directory of `evo_ape` and `evo_rpe` | `~/.local/bin` |
| `KALIBR_IMAGE` | Docker image that runs `kalibr_calibrate_cameras` | `kalibr:noetic` |

## Interactive tool

```bash
python tool/run_intrinsics_gui.py data/rig_a
```

`data/rig_a/.intrinsics_session.json` holds the board specification (11 x 8 inner corners, 45 mm). `Detect All` runs the corner extraction and `Calibrate` runs the two selection stages and the order selection. The tool records the images retained and rejected at each stage with their residuals, the subsets $\mathcal{C}$ and $\mathcal{V}$, the four quantities of the order criterion, the selected order and the corner-zone counts. `GUI YAML` and `ROS ost.yaml` export the parameters. `experiments/public/make_gui_session.py <folder> --cols C --rows R --square-mm S` writes the session file for any other folder of chessboard images.

## Headless

`rap_headless.py` executes the orchestration of the `Calibrate` action without Qt.

```bash
python experiments/public/experiment/rap_headless.py data/rig_a --out rig_a.json
# RAP  data/rig_a: 39 detected -> stage1 27 -> final 20 | M3 (free k3) | in-sample mean corner 0.143 px
```

```python
import sys
sys.path[:0] = ["tool", "experiments/public/experiment"]
from intrinsic_calib.camera_intrinsics import BoardSpec, result_to_ost_yaml
from rap_headless import detect_all, run_rap

spec = BoardSpec(pattern="chessboard", cols=11, rows=8, square_mm=45.0)
entries, size = detect_all("data/rig_a", spec)     # corner extraction
res = run_rap(entries, spec, size)                 # Stage 1, Stage 2, order selection, final fit
print(res["trace"]["order"]["chosen"])             # M3 (free k3)
ost = result_to_ost_yaml(res, "rig_a")             # ROS camera_info layout
```

`res["trace"]` lists the images rejected by Stage 1 and Stage 2, the validation subset and the validation errors of both orders. For an AprilGrid target, `BoardSpec(pattern="aprilgrid", cols=3, rows=3, square_mm=115.0, tag_spacing=0.3)` selects the Kalibr tag decoder with saddle-point refinement.

## Reproducing the paper

All commands are run with `OMP_NUM_THREADS=1`. The pipeline draws no random numbers, so a rerun returns the same parameters.

| item | command | expected output |
|---|---|---|
| Rig-A selection and order (Sections V-B, V-C) | `python experiments/public/experiment/rap_headless.py data/rig_a --out rig_a.json` | Stage 1 rejects 12 of 39, Stage 2 rejects 7, M3 selected (validation mean 0.316 to 0.299 px, maximum 0.590 to 0.554 px), 0.143 px on the retained images |
| Table I | `cd experiments/public && ./fetch_datasets.sh && ./write_sessions.sh && cd experiment && python final_comparison.py ../opencalib_front ../ros_wide_left ../ros_wide_right ../opencv_left ../opencv_right --every 4 --json-out out/final_table.json` | column `cell` of the printout; the rows `mrcal OPENCV4/5` give the order comparison of Section V-D ([experiments/public/README.md](../experiments/public/README.md)) |
| Table II and cross-session repeatability | `cd experiments/rigc && python flir_full.py cam0 && python flir_full.py cam1 && python ablation_rigc.py` | the printed table and the parameter differences between sessions ([experiments/rigc/README.md](../experiments/rigc/README.md)); uses the shipped corner caches, no bags required |
| Table III | `cd experiments/vio && python make_table3.py` | the printed table, scored from the shipped OpenVINS trajectories against the ground truth ([experiments/vio/README.md](../experiments/vio/README.md)) |
| Table III, VIO runs | `experiments/vio/run_matrix_all.sh` (Rig-C); `experiments/vio/rigd/run_rigd.py`, `run_rigd_variants.py` (Rig-D) | the trajectories of `experiments/vio/results/trajectories/`; bags, ROS and OpenVINS required |
| Consistency checks (Supplement) | `experiments/public/experiment/rig_ab_ros_check.py`; Rig-A runs of [experiments/public/README.md](../experiments/public/README.md) | parameter differences to the ROS calibrator on Rig-A and Rig-B; calibration-set errors of the three toolboxes on Rig-A |

## Tests

```bash
OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen python -m pytest tests -q
```

`test_rig_a.py` calibrates `data/rig_a` with the default settings and checks the selected order (M3), the rejections of Stage 1 (12 of 39) and Stage 2 (7), the validation errors of both orders, the calibration-set mean corner error (0.143 px), and that two runs return identical intrinsics. `test_aprilgrid.py` extracts the AprilGrid corners of the shipped Rig-C frame and compares them with the corner cache of Table II. `test_gui.py` drives the tool offscreen (`tests/gui_offscreen.py`), detects and calibrates Rig-A, and checks the selection.
