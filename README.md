# Automatic Reproducible Camera Intrinsic Calibration

This repository accompanies the submission "Automatic Reproducible Camera Intrinsic Calibration". It contains the calibration tool, the data of every experiment (Rig-A, Rig-B and the Rig-C calibration files; the manifest of the Rig-C and Rig-D bags, which are hosted separately; the fetch script of the five public sessions), the experiment scripts that produce the numbers of the paper, the OpenVINS configurations of Table III, and tests that reproduce the selection on Rig-A.

The method determines both the image set and the radial distortion order from the collected data. **Stage 1** admits an image only if the corner extractor recovers the complete inner-corner grid (AprilGrid: enough complete tags whose saddle-point refinement converges). **Stage 2** iteratively rejects the images whose mean residual exceeds $\kappa$ times the median of the retained set and re-solves the calibration. **Order selection** splits the accepted set into an estimation subset $\mathcal{C}$ and a validation subset $\mathcal{V}$ (every fifth image in capture order), estimates each candidate (M2: $k_1, k_2, p_1, p_2$; M3: additionally $k_3$) on $\mathcal{C}$ with Stage 2 applied internally, re-estimates only the board pose on each validation image, and adopts M3 if and only if it degrades neither the mean nor the worst-case validation error. All hyper-parameters are fixed across every dataset: $\kappa=2$, $\alpha=\beta=1$, $N_{\min}=15$, $N_{\mathrm{stop}}=8$, $T_{\max}=3$.

## Installation

The implementation is in Python 3.10 and has been tested on Ubuntu 20.04.

```bash
conda create -n intrinsic_calib python=3.10 && conda activate intrinsic_calib
pip install -r requirements.txt
# Kalibr AprilGrid and checkerboard extractors (compiled on first use)
sudo apt install g++ pkg-config libopencv-dev libeigen3-dev
```

The following components are required only by the experiments that name them:

| component | used by |
|---|---|
| ROS Noetic with `camera_calibration` (`/usr/bin/python3`) | ROS-calib baseline (Table I, consistency checks) |
| mrcal (`sudo apt install mrcal`) | mrcal baseline (Table I, Rig-D intrinsics) |
| evo (`evo_ape`, `evo_rpe`) | Table III |
| ROS Noetic and OpenVINS | rerunning the VIO experiments ([experiments/vio/README.md](experiments/vio/README.md)) |
| Docker image of Kalibr (`kalibr:noetic`) | Kalibr on Rig-A for the Rig-D intrinsics (`experiments/vio/rigd/scripts/run_kalibr_all39.py`) |

Every script resolves the tool and the data relative to the repository. The following environment variables override the defaults:

| variable | meaning | default |
|---|---|---|
| `CALIB_TOOL_DIR` | directory that contains `intrinsic_calib/` | `tool/` |
| `RIG_A_DIR`, `RIG_B_DIR` | Rig-A and Rig-B image folders | `data/rig_a`, `data/rig_b` |
| `RIGC_DIR` | directory of the Rig-C and Rig-D bags ([data/rigc/MANIFEST.md](data/rigc/MANIFEST.md)) | `data/rigc` |
| `VIO_WS` | catkin workspace of OpenVINS | required by the VIO scripts |
| `EVO_BIN` | directory of `evo_ape` and `evo_rpe` | `~/.local/bin` |
| `KALIBR_IMAGE` | Docker image that runs `kalibr_calibrate_cameras` | `kalibr:noetic` |

## Usage

**Interactive tool.**

```bash
python tool/run_intrinsics_gui.py data/rig_a
```

`data/rig_a/.intrinsics_session.json` holds the board specification (11 x 8 inner corners, 45 mm). `Detect All` runs the corner extraction and `Calibrate` runs the two selection stages and the order selection. The tool records the images retained and rejected at each stage with their residuals, the subsets $\mathcal{C}$ and $\mathcal{V}$, the four quantities of the order criterion, the selected order and the corner-zone counts. `GUI YAML` and `ROS ost.yaml` export the parameters. `experiments/public/make_gui_session.py <folder> --cols C --rows R --square-mm S` writes the session file for any other folder of chessboard images.

**Headless.** `rap_headless.py` executes the orchestration of the `Calibrate` action without Qt:

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

All commands are run with `OMP_NUM_THREADS=1`; the pipeline draws no random numbers, so a rerun returns the same parameters.

| item | command | expected output |
|---|---|---|
| Rig-A selection and order (Sections V-B, V-C) | `python experiments/public/experiment/rap_headless.py data/rig_a --out rig_a.json` | Stage 1 rejects 12 of 39, Stage 2 rejects 7, M3 selected (validation mean 0.316 to 0.299 px, maximum 0.590 to 0.554 px), 0.143 px on the retained images |
| Table I | `cd experiments/public && ./fetch_datasets.sh && ./write_sessions.sh && cd experiment && python final_comparison.py ../opencalib_front ../ros_wide_left ../ros_wide_right ../opencv_left ../opencv_right --every 4 --json-out out/final_table.json` | column `cell` of the printout; the rows `mrcal OPENCV4/5` give the order comparison of Section V-D ([experiments/public/README.md](experiments/public/README.md)) |
| Table II and cross-session repeatability | `cd experiments/rigc && python flir_full.py cam0 && python flir_full.py cam1 && python ablation_rigc.py` | the printed table and the parameter differences between sessions ([experiments/rigc/README.md](experiments/rigc/README.md)); uses the shipped corner caches, no bags required |
| Table III | `cd experiments/vio && python make_table3.py` | the printed table, scored from the shipped OpenVINS trajectories against the ground truth ([experiments/vio/README.md](experiments/vio/README.md)) |
| Table III, VIO runs | `experiments/vio/run_matrix_all.sh` (Rig-C); `experiments/vio/rigd/run_rigd.py`, `run_rigd_variants.py` (Rig-D) | the trajectories of `experiments/vio/results/trajectories/`; bags, ROS and OpenVINS required |
| Consistency checks (Supplement) | `experiments/public/experiment/rig_ab_ros_check.py`; Rig-A runs of [experiments/public/README.md](experiments/public/README.md) | parameter differences to the ROS calibrator on Rig-A and Rig-B; calibration-set errors of the three toolboxes on Rig-A |

## Data

| directory | content |
|---|---|
| `data/rig_a/` | Rig-A: 39 images, 1920 x 1080, 11 x 8 inner corners, 45 mm squares, with the session file and the `ost.yaml` of the ROS calibrator |
| `data/rig_b/` | Rig-B: 64 images of a second camera of the same model, with the session file and `ost.yaml` |
| `data/rigc/kalibr_D1/`, `data/rigc/kalibr_D2/` | Kalibr input and output of the two Rig-C calibration sessions (camera chain, camera-IMU calibration) |
| `data/rigc/frames/` | one Rig-C frame (session D2, cam0), used by the tests |
| `data/rigc/MANIFEST.md` | file names, sizes, checksums, durations and topics of the Rig-C bags (two calibration sessions, three VIO sequences) and of the Rig-D bag |
| `experiments/rigc/corners_*.json` | corner observations of all Rig-C calibration frames |
| `experiments/rigc/intrinsics/` | Rig-C intrinsics of the sets Kalibr, ours-M2 and ours (Full), input of the VIO experiment |
| `experiments/vio/rigd/intrinsics/` | Rig-D intrinsics of the five sets, input of the VIO experiment |
| `experiments/vio/gt/` | ground truth of the VIO sequences: FAST-LIO2 with manual loop closures (Rig-C), PALoc prior-map localisation with manual loop closures (Rig-D) |
| `experiments/vio/results/trajectories/` | OpenVINS trajectories of Table III |

The five public sessions are fetched from their original sources by `experiments/public/fetch_datasets.sh`. The Rig-C bags (21 GB) and the Rig-D bag (11 GB) are hosted separately. [DATA.md](DATA.md) lists the data of every experiment with the download addresses; [data/rigc/MANIFEST.md](data/rigc/MANIFEST.md) gives sizes, checksums and topics of the bags.

## Repository structure

```
tool/                    calibration package intrinsic_calib and the launcher run_intrinsics_gui.py
experiments/public/      Table I: fetch script of the public sessions, protocol, ROS-calib and mrcal baselines, headless pipeline
experiments/rigc/        Table II: cross-session evaluation and IS/OS ablation on Rig-C
experiments/vio/         Table III: OpenVINS configurations, run scripts, evo evaluation, ground truth, Rig-D
data/                    Rig-A, Rig-B, Rig-C Kalibr files, manifest of the bags
tests/                   tests of the Rig-A selection, the AprilGrid extraction and the tool
```

## Tests

```bash
OMP_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen python -m pytest tests -q
```

`test_rig_a.py` calibrates `data/rig_a` with the default settings and checks the selected order (M3), the rejections of Stage 1 (12 of 39) and Stage 2 (7), the validation errors of both orders, the calibration-set mean corner error (0.143 px), and that two runs return identical intrinsics. `test_aprilgrid.py` extracts the AprilGrid corners of the shipped Rig-C frame and compares them with the corner cache of Table II. `test_gui.py` drives the tool offscreen (`tests/gui_offscreen.py`), detects and calibrates Rig-A, and checks the selection.

## Citation

```bibtex
@article{anonymous2026intrinsiccalib,
  title  = {Automatic Reproducible Camera Intrinsic Calibration},
  author = {Anonymous},
  note   = {Under double-anonymous review},
  year   = {2026}
}
```

## License

Released under the [MIT license](./LICENSE). `tool/intrinsic_calib/kalibr_extractor/third_party/` contains the `ethz_apriltag2` sources and the target-extractor code of Kalibr (BSD, [KALIBR_LICENSE](tool/intrinsic_calib/kalibr_extractor/third_party/KALIBR_LICENSE)). The public sessions keep the licences of their sources ([experiments/public/README.md](experiments/public/README.md)).

## Acknowledgements

The tool uses [OpenCV](https://github.com/opencv/opencv) and the `ethz_apriltag2` decoder of [Kalibr](https://github.com/ethz-asl/kalibr). The evaluation uses the public data and baselines of [OpenCalib](https://github.com/PJLab-ADG/SensorsCalibration), [ROS camera_calibration](http://wiki.ros.org/camera_calibration), [OpenCV](https://github.com/opencv/opencv), [mrcal](https://mrcal.secretsauce.net) and [Kalibr](https://github.com/ethz-asl/kalibr), and [OpenVINS](https://github.com/rpng/open_vins), [evo](https://github.com/MichaelGrupp/evo), [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) and PALoc (prior-assisted LiDAR localisation, cited in the paper) for the odometry experiments and the ground truth.
