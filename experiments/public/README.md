# Public sessions (Table I)

This directory contains the fetch script of the five public chessboard sessions, the protocol of Table I and the ROS-calib and mrcal baselines. The sessions are third-party data and are not redistributed; `fetch_datasets.sh` downloads them from their original sources.

## Sessions

| folder | name in the paper | source | resolution | board (inner corners, square) | images |
|---|---|---|---|---|---|
| `opencalib_front` | OpenCalib | [OpenCalib](https://github.com/PJLab-ADG/SensorsCalibration), front camera | 1920 x 1200 | 17 x 15, 50 mm | 22 |
| `ros_wide_left` | ROS L | [ROS image_pipeline](https://github.com/ros-perception/image_pipeline), wide stereo, left | 640 x 480 | 8 x 6, 108 mm | 12 |
| `ros_wide_right` | ROS R | same, right | 640 x 480 | 8 x 6, 108 mm | 12 |
| `opencv_left` | OpenCV L | [OpenCV](https://github.com/opencv/opencv) samples, stereo, left | 640 x 480 | 9 x 6, 30 mm | 13 |
| `opencv_right` | OpenCV R | same, right | 640 x 480 | 9 x 6, 30 mm | 13 |

The grid size of every session is confirmed by the detection of the complete inner-corner grid on every image (`verify_datasets.py`). The square sizes of OpenCalib and OpenCV are not stated by the sources and are nominal; the square size sets only the extrinsic scale and does not affect the intrinsics or the distortion coefficients. The OpenCV sessions contain images 01 to 09 and 11 to 14; image 10 does not exist upstream. The ROS images are converted from `.pgm` to `.png` on download.

```bash
cd experiments/public
./fetch_datasets.sh            # all sessions (or: opencv | opencalib | ros)
./write_sessions.sh            # writes .fastcalib_intrinsics.json into every folder
python verify_datasets.py      # independent OpenCV fit of every session (optional)
```

`./fetch_datasets.sh mrcal` downloads the mrcal Samyang fisheye sessions, which are not used in the paper. Each fetched folder opens directly in the tool (`python tool/run_intrinsics_gui.py experiments/public/opencalib_front`). `make_gui_session.py <folder> --cols C --rows R --square-mm S` writes the session file for any other folder of chessboard images.

## Table I

All methods are evaluated with a rotating four-fold holdout: the images, sorted by name, are split into the evaluation fold `entries[f::4]` and the calibration fold; both selection stages, the order criterion and the final fit are computed within the calibration fold, and the held-out error is the mean corner error on the evaluation fold with the intrinsics fixed and only the board pose re-estimated. A cell is the mean over the four folds and, in parentheses, the fold-to-fold standard deviation (ddof = 0) in units of the last digit.

```bash
cd experiments/public/experiment
python final_comparison.py ../opencalib_front ../ros_wide_left ../ros_wide_right ../opencv_left ../opencv_right \
       --every 4 --json-out out/final_table.json
```

| row of Table I | label printed by `final_comparison.py` | implementation |
|---|---|---|
| ROS-calib | `ROS-calib` | `ros_baseline.py`: the `MonoCalibrator` of ROS `camera_calibration` with its own detector, sub-pixel refinement and frame acceptance, `-k 2` (M2); runs under the ROS Noetic interpreter `/usr/bin/python3` |
| mrcal | `mrcal OPENCV8` | `mrcal_baseline.py`: `mrcal-calibrate-cameras` with its defaults on the corners of Stage 1, `LENSMODEL_OPENCV8`; the 4- and 5-coefficient variants (`mrcal OPENCV4`, `mrcal OPENCV5`) enter the order comparison of Section V |
| ours-M2 | `ours-M2` | all images of the calibration fold, M2, no selection |
| ours (Full) | `ours (Full)` | `rap_headless.run_rap`: Stage 1, Stage 2 within each candidate order, order selection, final fit |

Expected output (column `cell`):

```
opencalib_front   ROS-calib 0.232(18)  mrcal OPENCV8 0.387(104)  ours-M2 0.230(18)  ours (Full) 0.226(21)
ros_wide_left     ROS-calib 0.168(41)  mrcal OPENCV8 0.231(77)   ours-M2 0.167(40)  ours (Full) 0.165(40)
ros_wide_right    ROS-calib 0.157(34)  mrcal OPENCV8 0.165(35)   ours-M2 0.151(33)  ours (Full) 0.148(34)
opencv_left       ROS-calib 0.601(84)  mrcal OPENCV8 0.243(50)   ours-M2 0.234(51)  ours (Full) 0.233(49)
opencv_right      ROS-calib 0.665(101) mrcal OPENCV8 0.271(50)   ours-M2 0.246(55)  ours (Full) 0.240(58)
```

The run takes a few minutes. Without mrcal (`sudo apt install mrcal`) or without ROS Noetic the corresponding rows are omitted.

## Rig-A runs

`rap_headless.py` executes the orchestration of the tool's `Calibrate` action without Qt and writes the parameters, the trace of every stage and, optionally, a corner cache on which the baselines can be scored with identical observations:

```bash
cd experiments/public/experiment
python rap_headless.py ../../../data/rig_a --out out/fs39_rap.json --corner-cache out/fs39_cache.json
# RAP  ../../../data/rig_a: 39 detected -> stage1 27 -> final 20 | M3 (free k3) | in-sample mean corner 0.143 px
/usr/bin/python3 ros_baseline.py ../../../data/rig_a --out out/fs39_ros.json
python mrcal_baseline.py --cache out/fs39_cache.json --outdir out/mrcal_fs39 --out out/fs39_mrcal.json
python rig_ab_ros_check.py      # estimator with both selections disabled against the ROS calibrator on Rig-A and Rig-B
```

## Files

| file | role |
|---|---|
| `fetch_datasets.sh` (`all`, `opencv`, `opencalib`, `ros`, `mrcal`), `write_sessions.sh`, `make_gui_session.py`, `verify_datasets.py` | download and staging of the sessions |
| `experiment/final_comparison.py` | Table I |
| `experiment/rap_headless.py` | headless pipeline (`detect_all`, `run_rap`), used by every script of this directory |
| `experiment/ros_baseline.py`, `experiment/mrcal_baseline.py` | the two toolbox baselines |
| `experiment/protocol.py` | shared scoring (`pose_only`) and the ROS-equivalent flags |
| `experiment/rig_ab_ros_check.py` | consistency with the ROS calibrator (Supplement) |
| `experiment/_qtstub/` | minimal PySide6 stand-in, so that the algorithmic core is imported without the interface |
| `experiment/run_all.sh` | runs the scripts of this directory in sequence |
| `experiment/kfold.py`, `experiment/protocol_full.py`, `experiment/compare.py`, `experiment/extractor_check/`, `experiment/fisheye_probe.py`, `experiment/make_undist_figure.py` | additional analyses that are not reported in the paper |
| `extra/kalibr_baseline/` | Kalibr on the public sessions; this baseline is not reported in the paper |

## Licences of the sessions

OpenCV samples: Apache-2.0. OpenCalib: Apache-2.0. ROS `image_pipeline` data: BSD. mrcal sample data: distributed with the mrcal documentation. The source of every session should be cited when it is used.
