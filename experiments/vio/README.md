# VIO evaluation (Table III)

This directory measures how a set of camera intrinsics changes the trajectory
accuracy of visual-inertial odometry. Table III reports OpenVINS on the three
Rig-C sequences S1 to S3 (stereo) and on the Rig-D sequence (monocular), on
images rectified with each calibration and with identical extrinsics and
settings. The metrics are the ATE (SE(3)-aligned translation RMSE) and the RPE
(translation RMSE over all pairs 1 m apart), computed with evo.

Ground truth (`gt/`):

* Rig-C: LiDAR-inertial SLAM (FAST-LIO2) with manual loop closures
  (`gt/README_rigc.md`).
* Rig-D: LiDAR localisation in a prior terrestrial-laser-scan map (PALoc) with
  manual loop closures (`gt/README_rigd.md`).

## Table III from the shipped trajectories

The OpenVINS trajectories of every row of Table III are shipped in
`results/trajectories/` (`rigc/<set>/S{1,2,3}.tum`, `rigd/<set>/v{0..4}.tum`,
TUM format, IMU body pose). `make_table3.py` scores them against `gt/` with
`evaluate.py` and writes `results/table3.json`; it requires evo but neither
ROS nor the bags:

```bash
cd experiments/vio
python make_table3.py
```

Expected output:

```
method              Rig-C S1        Rig-C S2        Rig-C S3         Rig-D
ROS-calib                  x               x               x   4.75 / 0.22
mrcal                      x               x               x   5.86 / 0.27
Kalibr         0.091 / 0.070   0.768 / 0.109   0.410 / 0.059   3.99 / 0.30
ours-M2        1.083 / 0.213   3.436 / 0.204   3.111 / 0.359   5.03 / 0.24
ours (Full)    0.071 / 0.043   0.155 / 0.052   0.307 / 0.051   2.40 / 0.19
Rig-D ATE range over v0..v4 [m]: ROS-calib 3.23-5.37, mrcal 3.81-14.99, Kalibr 2.25-10.61, ours-M2 4.53-5.29, ours (Full) 1.17-3.85
```

Correspondence of the names: Rig-C S1 = `2026-02-23-11-46-32`, S2 =
`2026-02-23-11-57-46`, S3 = `2026-02-22-11-25-57`; the Rig-C sets `kalibr`,
`M2-R` and `RAP` of `experiments/rigc/intrinsics/` are the rows Kalibr, ours-M2
and ours (Full); the Rig-D sets are in `rigd/intrinsics/` (`ours.yaml` = ours
(Full)). ROS-calib and mrcal do not support the AprilGrid target of Rig-C.

The remainder of this file describes how the trajectories are produced, which
requires ROS Noetic, OpenVINS and the bags of `data/rigc/MANIFEST.md`.

## File layout

```
vio/
  make_table3.py                    Table III from results/trajectories/ (see above)
  evaluate.py                       evaluate.py --ref gt.txt --est run.tum [--json o.json]   (ATE / RPE with evo)
  refs.py                           sequence -> ground-truth file, used by run_matrix.py and rigd/
  gt/                               ground truth of Table III (TUM, IMU body pose) and its construction (README_*.md)
  results/trajectories/             the OpenVINS trajectories of Table III
  run_matrix.py, run_matrix_all.sh  Rig-C: intrinsics set x sequence on pre-rectified images
  vio/make_vio_config.py            intrinsics set -> vio/configs/<set>_<session>/openvins/
  vio/configs/<set>_<session>/      the OpenVINS configurations of the Rig-C sets kalibr, M2-R and RAP (sessions D1, D2)
  vio/templates/                    fixed parts of the OpenVINS configurations (estimator settings, IMU chain)
  vio/openvins_serial.launch, vio/openvins.launch, vio/run_openvins.sh   OpenVINS (serial reader = default, or real time)
  scripts/ros_env.sh                private rosmaster per run (free port)
  scripts/make_raw_bag.py           original bag -> image + IMU bag (no LiDAR)
  scripts/make_rect_bag.py          raw-image bag -> bag rectified with one intrinsics set
  scripts/bag_to_tum.py             recorded pose topic -> TUM
  scripts/traj_summary.py           sanity numbers of a TUM file (duration, path length, bounding box, maximum speed)
  scripts/derive_extrinsics.py      Kalibr T_cam_imu + FAST-Calib T_cam_lidar -> T_imu_lidar (body frame of the Rig-C ground truth)
  extrinsics/                       LiDAR-camera extrinsics and the derived LiDAR-IMU extrinsics of Rig-C
  rigd/                             monocular OpenVINS comparison on the Rig-D sequence (section "Rig-D sequence")
```

## Environment

* ROS Noetic (`/opt/ros/noetic`), `rosbag`, `image_transport`.
* `VIO_WS`: catkin workspace with OpenVINS (upstream master, ROS1, Ceres 2.1),
  including the deterministic serial reader `ros1_serial_msckf`.
* evo (`evo_ape`, `evo_rpe`); set `EVO_BIN` if they are not in `~/.local/bin`.
* `RIGC_DIR`: directory with the bags of `data/rigc/MANIFEST.md`.

## Rig-C: pre-rectified evaluation of the intrinsics sets (`run_matrix.py`)

No intrinsics set is truncated to four coefficients: for every set and camera
the raw-image VIO bag is undistorted with the full radtan model of that set
(five coefficients) (`scripts/make_rect_bag.py`: `cv2.initUndistortRectifyMap(K,
[k1 k2 p1 p2 k3], R=I, newK=K, same size)`, bilinear), cached in
`$RIGC_DIR/flir_eval_rawbags/<set>/<key>_vio_rect.bag`, and OpenVINS receives a
pinhole model with the set's `fx fy cx cy` and zero distortion
(`vio/make_vio_config.py --cam0 ... --cam1 ... --rectified`). Every set, Kalibr
included (k3 = 0), follows the identical path. In stereo each camera is
rectified with its own set; IMU-camera extrinsics, time shift and `T_cn_cnm1`
are unchanged. The intrinsics files are
`experiments/rigc/intrinsics/<cam>_<session>_<method>.yaml` (session D1 is
evaluated on the 02-22 sequence, D2 on the two 02-23 sequences).

```bash
export RIGC_DIR=/path/to/bags VIO_WS=/path/to/openvins_ws
cd experiments/vio
./run_matrix_all.sh                                                     # sets RAP, kalibr, M2-R on all three sequences
./run_matrix.py --sets RAP --bags 2026-02-22-11-25-57                  # subset; resumable
./run_matrix.py --rescore-only                                          # score the existing trajectories only
```

Outputs: `results/matrix/<set>/<key>/openvins_stereo.tum`,
`results/matrix_gt.json` and `results/matrix_gt.md` (cell = ATE / RPE at 1 m).
The trajectories `results/matrix/<set>/<key>/openvins_stereo.tum` are the files
`results/trajectories/rigc/<set>/S{1,2,3}.tum` scored by `make_table3.py`.

### What is fixed and what varies

Only the camera intrinsics vary between sets. Fixed from the Kalibr calibration
of session D2 (`data/rigc/kalibr_D2/calib-camchain-imu0-cam.yaml`): `T_cam_imu`
of cam0 and cam1, the stereo transform `T_cn_cnm1` and `timeshift_cam_imu`
(cam0 +0.013640 s, cam1 +0.013429 s, convention t_imu = t_cam + shift). IMU
noise: the Allan densities of `data/rigc/kalibr_D2/imu0.yaml`, inflated by a
factor of 5, identical for every set. Online calibration is disabled
(`calib_cam_extrinsics/intrinsics/timeoffset: false`), so that the supplied
intrinsics are used as they are. Further settings (`vio/templates/`):
`init_dyn_use: true` (the platform moves from the first frame),
`init_dyn_mle_max_time 2.0 / max_iter 300`, `track_frequency: 21`. OpenVINS
reads the raw-image bags with the serial reader; two serial runs are
bit-identical, whereas real-time playback is not reproducible, so the serial
mode is used throughout.

OpenVINS (`ov_core` `CamRadtan`) reads exactly four radtan coefficients and
does not support k3. The evaluation therefore rectifies the images with the
full model of each set instead of passing the coefficients to the estimator;
`make_vio_config.py` refuses an intrinsics file with k3 != 0 unless
`--rectified` or `--drop_k3` is given.

## Evaluation (`evaluate.py`)

* Reference = the ground truth `gt/gt_*.txt` (`refs.py`). The VIO poses are
  stamped at image times (plus the camera-IMU shift), about 14 ms off the
  LiDAR scan stamps, so the reference is resampled at the estimate timestamps
  (linear position, slerp rotation) before association (`--no_interp` for
  plain nearest neighbour with 0.02 s tolerance).
* ATE: `evo_ape -a` (SE(3) Umeyama alignment, no scale), translation RMSE [m]
  and rotation RMSE [deg].
* RPE: `evo_rpe --delta 1 --delta_unit m --all_pairs`, translation RMSE [m].
* No scale alignment is applied, so the scale error of monocular VIO enters
  ATE and RPE.

## Rig-D sequence (Table III, monocular)

`rigd/` repeats the comparison on a second platform: a wheeled robot whose
camera is of the Rig-A model (1920 x 1080, same lens), with a solid-state LiDAR
and an IMU, and one indoor sequence of 163 s (126.4 m, closed loop;
`data/rigc/MANIFEST.md`, `rigd_office1.bag`). The five intrinsics sets are all
estimated on the 39 Rig-A images (`rigd/intrinsics/`): ROS calibrator
(`data/rig_a/ost.yaml`), mrcal (`LENSMODEL_OPENCV8`), Kalibr (pinhole-radtan at
its defaults, `scripts/run_kalibr_all39.py`), the proposed pipeline without
the selection stages (`ours-M2`) and the proposed pipeline (`ours`).
Monocular OpenVINS (serial reader) runs on images rectified with the full
model of each set and downscaled to 960 x 540
(`scripts/make_rect_bag_rigd.py`); the extrinsic
(`rigd/extrinsics/extrinsic_cam_lidar.yaml`), the time shift (0.005 s), the IMU noise and
every estimator setting are identical for all sets (`vio_templates/`,
`scripts/make_vio_config_rigd.py`). The scores are against the ground truth
`gt/gt_rigd_office1.txt`.

```
rigd/
  run_rigd.py                  run | eval: rectify -> config -> OpenVINS -> ATE/RPE per set; writes rigd_results_gt.{json,md}
  run_rigd_variants.py         run | eval: the four further front-end settings per set; writes rigd_variants_gt.{json,md}
  run_openvins_rigd.sh         OpenVINS mono, serial reader, on one rectified bag
  vio_templates/               OpenVINS estimator settings and IMU chain shared by all sets
  intrinsics/                  the five intrinsics sets
  extrinsics/                  camera-LiDAR extrinsic of the platform
  scripts/                     split_bag, make_intrinsics, run_kalibr_all39 (+ make_kalibr_bag), make_rect_bag_rigd,
                               make_vio_config_rigd
```

Procedure (ROS Noetic and `VIO_WS` as above; run from `experiments/vio/rigd`):

```bash
python scripts/split_bag.py "$RIGC_DIR/rigd_office1.bag" bags          # -> bags/rigd_office1_raw.bag (mono8 images + IMU)
python run_rigd.py run && python run_rigd.py eval                       # base setting v0 of every set
python run_rigd_variants.py run && python run_rigd_variants.py eval     # settings v1..v4 -> median over v0..v4
#   score existing runs only (no VIO): python run_rigd.py --rescore-only; python run_rigd_variants.py --rescore-only
```

`run_rigd_variants.py` reruns every set with `num_pts` 200 / 300 and
`fast_threshold` 15 / 25 in addition to the base setting (identical for all
sets); Table III reports the median over these five runs. The rectified bags
are deleted after use (about 1 GB each). The intrinsics files are shipped;
`scripts/make_intrinsics.py` regenerates them from the Rig-A runs of
`experiments/public/README.md` (`out/fs39_rap.json`, `out/fs39_mrcal.json`),
`data/rig_a/ost.yaml` and the Kalibr output of `scripts/run_kalibr_all39.py run1`
(Docker image `kalibr:noetic`, built from the `Dockerfile_ros1_20_04` of the
Kalibr repository; override with `KALIBR_IMAGE`). The trajectories
`runs/<set>/openvins_mono.tum` and `runs/<set>/openvins_mono_v{1..4}.tum` are
the files `results/trajectories/rigd/<set>/v{0..4}.tum` scored by
`make_table3.py`.
