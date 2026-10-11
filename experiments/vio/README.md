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
  config/flir_lio.yaml              FAST-LIVO2 LIO-only config for Rig-C (lidar_type 9, extrinsics below)
  config/camera_flir_cam0.yaml      vikit camera yaml (fastlivo_mapping loads one; LIO ignores it)
  flir.launch                       FAST-LIVO2 headless launch (bag play --clock, image republish, odom record)
  run_one.sh                        run_one.sh <bag> <config.yaml> <camera.yaml> <out.tum> [rate]   (FAST-LIVO2)
  make_refs.sh                      LIO reference of the three sequences (rate 0.5) -> ref/<key>_lio.tum
  gt/gt_*.txt                       ground truth of Table III (TUM, IMU body pose; LIO or map localisation + manual loop closures); gt/README_*.md
  make_table3.py                    Table III from results/trajectories/ (see above)
  results/trajectories/             the OpenVINS trajectories of Table III
  ref/<key>_lio.tum                 earlier reference trajectories (FAST-LIVO2 LIO) (TUM, IMU body pose, LiDAR scan-end timestamps)
  evaluate.py                       evaluate.py --ref ref.tum --est run.tum [--json o.json] [--plot o.png]
  eval_set.sh                       eval_set.sh <intrinsics.yaml> <set_name> [bag_key ...]   (one intrinsics set, all systems)
  refs.py                           sequence -> reference file (gt / lio), used by run_matrix.py, rigd/run_rigd.py, rigd/run_rigd_variants.py
  run_matrix.py, run_matrix_all.sh  intrinsics-set x sequence x system matrix on pre-rectified images (used for the paper)
  vio/intrinsics_kalibr.yaml        intrinsics file format (values = Kalibr)
  vio/make_vio_config.py            intrinsics -> vio/configs/<set>/{openvins,vins}/...
  vio/configs/<set>_<session>/      the generated configurations of every intrinsics set of experiments/rigc/intrinsics/
  vio/templates/                    fixed parts of the VIO configs (estimator settings, IMU chain, VINS template)
  vio/openvins_serial.launch, vio/openvins.launch, vio/run_openvins.sh   OpenVINS (serial = default, or realtime)
  vio/vins.launch, vio/run_vins.sh  VINS-Fusion headless (mono+IMU or stereo+IMU)
  scripts/ros_env.sh                private rosmaster per run (free port)
  scripts/make_raw_bag.py           original bag -> image + IMU bag (no LiDAR)
  scripts/make_rect_bag.py          raw-image bag -> bag rectified with one intrinsics set
  scripts/bag_to_tum.py             recorded odometry/pose topic -> TUM
  scripts/traj_summary.py           sanity numbers of a TUM file (duration, path length, bounding box, max speed)
  scripts/summarize.py              results/<set>/summary.md table
  scripts/derive_extrinsics.py      Kalibr T_cam_imu + FAST-Calib T_cam_lidar -> FAST-LIVO2 extrinsics
  extrinsics/derived_extrinsics.yaml  all candidate extrinsics and the cam0/cam1 consistency table
  extrinsics/fastcalib/             the LiDAR-camera extrinsics (input of derive_extrinsics.py)
  patches/vikit_cam_d4.patch        one-line vikit loader change that reads k3 (not applied, see below)
  rigd/                              monocular OpenVINS comparison on the Rig-D sequence (section "Rig-D sequence")
```

## Environment

* ROS Noetic (`/opt/ros/noetic`), `rosbag`, `image_transport`.
* `FASTLIVO_WS`: catkin workspace that contains `src/FAST-LIVO2`, with the
  Ouster point-layout handler described below, built with
  `catkin build fast_livo`.
* `VIO_WS`: catkin workspace with OpenVINS (upstream master, ROS1, Ceres 2.1)
  and VINS-Fusion (upstream master; built for Noetic with C++14 in the four
  CMakeLists and the legacy OpenCV C-API headers replaced in `camera_models`
  and `vins_estimator`; only `camera_models` and `vins` are built, no loop
  closure).
* evo (`evo_ape`, `evo_rpe`); set `EVO_BIN` if they are not in `~/.local/bin`.
* `RIGC_DIR`: directory with the five bags of `data/rigc/MANIFEST.md`.

## Running one intrinsics set

1. Write the intrinsics file (copy `vio/intrinsics_kalibr.yaml`):
   ```yaml
   cam0: {fx: ..., fy: ..., cx: ..., cy: ..., k1: ..., k2: ..., p1: ..., p2: ..., k3: 0.0}
   cam1: {...}        # optional; Kalibr cam1 is used when absent (stereo runs)
   ```
2. Run everything (configurations are generated, four VIO runs per sequence,
   evaluated against `ref/`):
   ```bash
   export RIGC_DIR=/path/to/rigc_bags FASTLIVO_WS=... VIO_WS=...
   cd experiments/vio
   ./eval_set.sh /path/to/intrinsics_X.yaml setX                      # all three sequences
   ./eval_set.sh /path/to/intrinsics_X.yaml setX 2026-02-23-11-46-32   # only the 27.5 s sequence
   cat results/setX/summary.md
   ```
   Sequence keys: `2026-02-23-11-46-32` (27.5 s), `2026-02-23-11-57-46`
   (97 s), `2026-02-22-11-25-57` (72 s).
3. Individual steps:
   ```bash
   vio/make_vio_config.py intrinsics_X.yaml vio/configs/setX
   vio/run_openvins.sh <bag> vio/configs/setX/openvins/estimator_config.yaml out/ov_mono.tum mono      # serial (deterministic)
   vio/run_openvins.sh <bag> vio/configs/setX/openvins/estimator_config.yaml out/ov_stereo.tum stereo  # 'realtime' as 5th argument plays the bag instead
   vio/run_vins.sh     <bag> vio/configs/setX/vins/vins_mono_imu.yaml   out/vins_mono.tum
   vio/run_vins.sh     <bag> vio/configs/setX/vins/vins_stereo_imu.yaml out/vins_stereo.tum
   ./evaluate.py --ref gt/gt_rigc_s1.txt --est out/ov_mono.tum --json out/ov_mono.json   # ground truth (Table III)
   ```
   The ground truth of Table III is in `gt/` (`gt_rigc_s1.txt` = `2026-02-23-11-46-32`, `gt_rigc_s2.txt` = `2026-02-23-11-57-46`,
   `gt_rigc_s3.txt` = `2026-02-22-11-25-57`, `gt_rigd_office1.txt` = Rig-D; from the repository root `--ref experiments/vio/gt/gt_*.txt`;
   construction in `gt/README_rigc.md` and `gt/README_rigd.md`), and Table III is produced by `make_table3.py`.
   The earlier reference, FAST-LIVO2 in LIO-only mode (`ref/2026-02-23-11-46-32_lio.tum`, same call with `--ref ref/...`), is kept.
   The matrix scripts (`run_matrix.py`, `run_rigd.py`, `run_rigd_variants.py`) take `--ref-set {gt,lio}` (default `gt`; the
   sequence-to-file mapping is `refs.py`, the gt files are searched in `$VIO_GT_DIR` and `gt/`) and `--rescore-only`
   (score the existing trajectories, no VIO run); `--ref-set lio` selects the earlier reference.
   The references are regenerated with `./make_refs.sh` (only needed if the
   LIO configuration changes).

FAST-LIVO2 and VINS-Fusion play the original bag at real time (`--clock`,
`/use_sim_time`, images republished from `CompressedImage` with
`image_transport republish`); OpenVINS reads the raw-image VIO bag serially.
Everything is headless, under `timeout`, each run on its own rosmaster port
(`scripts/ros_env.sh`).

## Pre-rectified evaluation of the intrinsics sets (`run_matrix.py`)

This is the protocol used for the paper. No intrinsics set is truncated to four
coefficients: for every set and camera the raw-image VIO bag is undistorted
with the full radtan model of that set (five coefficients; a set that carries
the eight-coefficient rational model is undistorted with all eight)
(`scripts/make_rect_bag.py`: `cv2.initUndistortRectifyMap(K, [k1 k2 p1 p2 k3],
R=I, newK=K, same size)`, bilinear), cached in
`$RIGC_DIR/flir_eval_rawbags/<set>/<key>_vio_rect.bag`, and the VIO systems
receive a pinhole model with the set's `fx fy cx cy` and zero distortion
(`make_vio_config.py --cam0 ... --cam1 ... --rectified`). Every set, Kalibr
included (k3 = 0), goes through the identical path. In stereo each camera is
rectified with its own set; IMU-camera extrinsics, time shift and `T_cn_cnm1`
are unchanged. The intrinsics files are
`experiments/rigc/intrinsics/<cam>_<session>_<method>.yaml` (session D1 is
evaluated on the 02-22 sequence, D2 on the two 02-23 sequences).

```bash
./run_matrix_all.sh                      # all sets, all sequences; log: results/matrix_run.log
./run_matrix.py --sets RAP --bags 2026-02-22-11-25-57 --systems openvins_stereo   # subset; resumable
```

Outputs: `results/matrix/<set>/<key>/<system>.tum`,
`results/matrix_gt.json` (ATE/RPE statistics against the ground truth, path length, coverage,
`tracked_to_end` = coverage >= 90 %, path ratio 0.5 to 2, max speed < 5 m/s)
and `results/matrix_gt.md` (one table per system: rows = sets, columns =
sequences, cell = ATE / RPE at 1 m, translation). `./run_matrix.py --rescore-only` re-scores the existing
`.tum` files without a VIO run; `--ref-set lio` scores against `ref/<key>_lio.tum` instead and writes
`results/matrix.{json,md}` (with `_vs_lio.json/.png` next to the trajectories). Systems: OpenVINS stereo and
mono (deterministic serial reader), VINS-Fusion stereo (real-time playback of
the rectified bag). The configurations are identical across sets except for
the intrinsics.

## What is fixed and what varies

Only the camera intrinsics vary between sets. Fixed from the Kalibr calibration
of session D2 (`data/rigc/kalibr_D2/calib-camchain-imu0-cam.yaml`): `T_cam_imu`
of cam0 and cam1, the stereo transform `T_cn_cnm1`, `timeshift_cam_imu` (cam0
+0.013640 s, cam1 +0.013429 s, convention t_imu = t_cam + shift; VINS-Fusion
`td` uses the same convention). IMU noise from `imu0.yaml` (Allan densities,
inflated by a factor of 5 in both systems, identical for every set). Online
calibration is disabled in both systems (`calib_cam_extrinsics/intrinsics/
timeoffset: false` in OpenVINS, `estimate_extrinsic: 0`, `estimate_td: 0` in
VINS-Fusion) so that the supplied intrinsics are used as they are.

### Distortion-coefficient support (k3)

| system | model | coefficients read | k3 |
|---|---|---|---|
| OpenVINS (ov_core `CamRadtan`) | radtan | `distortion_coeffs` must have 4 entries (`cv::Vec4d`) | not supported |
| VINS-Fusion (camodocal `PinholeCamera`) | PINHOLE | k1 k2 p1 p2 | not supported |
| FAST-LIVO2 / vikit `PinholeCamera` | OpenCV 5-vector | the class uses d_[4] in the r^6 term, but the loader `vikit_ros/include/vikit/camera_loader.h` reads only `cam_d0..cam_d3` | class yes, loader no (`patches/vikit_cam_d4.patch`) |

This is why the paper's evaluation rectifies the images with the full model of
each set (`run_matrix.py`) instead of passing the coefficients to the VIO
systems. `make_vio_config.py` refuses an intrinsics file with k3 != 0 unless
`--drop_k3` is given.

## FAST-LIVO2 LIO reference

* Topics: `/sensors/lidar0` (Ouster OS-128, 262 144 points at 10 Hz),
  `/sensors/imu0` (500 Hz).
* Point-cloud layout: `x, y, z` float32, `intensity` uint8, `time` float64 =
  nanoseconds since scan start (0 to 100.2e6), `ring` uint16. No stock
  FAST-LIVO2 handler matches these datatypes (`pcl::fromROSMsg` needs an exact
  datatype match per field), so the following source change is required and
  rebuilt with `catkin build fast_livo --no-deps`:
  * `include/common_lib.h`: enum `OUSTER_FLIR = 9`
  * `include/preprocess.h`: `ouster_flir_handler()` declaration,
    `ouster_time_scale_to_ms` (default 1e-6)
  * `src/preprocess.cpp`: `case OUSTER_FLIR` + `ouster_flir_handler()` (fields
    read by offset with datatype dispatch, time converted to ms into
    `curvature`, `blind` / `point_filter_num` applied, points sorted by time)
  * `src/LIVMapper.cpp`: `nh.param("preprocess/ouster_time_scale_to_ms", ...)`
  The first scan logs `[OUSTER_FLIR] first scan: 262144 raw pts, 46979 kept,
  per-point time span 0.000 .. 100.164 ms`.
* `config/flir_lio.yaml`: lidar_type 9, point_filter_num 4, filter_size_surf
  0.1, blind 0.8, scan_line 128, IMU init_mode 1, acc/gyr cov 0.01,
  pose_output_en true (native TUM via `evo/seq_name`).
* Trajectory: FAST-LIVO2's own `evo/pose_output_en` file
  (`FAST-LIVO2/Log/result/<seq>.txt`, state after every LIO update, timestamp =
  LiDAR scan end) copied to `<out>.tum`; `/aft_mapped_to_init` is recorded as
  `<out>.odom.tum` as a fallback (identical content). The pose is the IMU body
  pose, the same body frame as the VIO outputs (OpenVINS `/ov_msckf/poseimu`,
  VINS-Fusion `/vins_estimator/odometry`).
* LIO is deterministic here: two runs of the 27.5 s sequence differ by 0.0000 m
  ATE. `make_refs.sh` plays the bags at rate 0.5 because the mapper runs at
  about 0.6 times real time on these scans and `flir.launch` stops when
  `bag_play` ends.

### Extrinsics used in `config/flir_lio.yaml`

Conventions: `extrinsic_R/extrinsic_T`: p_imu = R p_lidar + T. `Rcl/Pcl`:
p_cam = Rcl p_lidar + Pcl. Derivation: `T_imu_lidar = inv(T_cam0_imu [Kalibr])
@ T_cam0_lidar [FAST-Calib]` (`scripts/derive_extrinsics.py`).

`T_cam0_lidar` is `extrinsics/fastcalib/april_20260725_191352_cam0.yaml`
(AprilGrid, 189 frames, point-to-plane RMS 11.4 mm; cam1 from the same run,
183 frames):

```
extrinsic_T = [-0.003517, -0.049043,  0.023119]      (|t| = 5.4 cm)
extrinsic_R = [ 0.999743  0.017975 -0.013811
               -0.018212  0.999686 -0.017227
                0.013497  0.017474  0.999756]        (IMU and LiDAR frames nearly aligned, about 1.4 deg)
Pcl (cam0)  = [ 0.041037, -0.045374, -0.024927]
Rcl (cam0)  = [-0.999819 -0.008858  0.016841
               -0.016711 -0.014519 -0.999755
                0.009100 -0.999855  0.014368]        (cam x = -lidar x, cam y = -lidar z, cam z = -lidar y)
```

The other candidates and their pairwise agreement are listed in
`extrinsics/derived_extrinsics.yaml`; the AprilGrid cam0 and cam1 results agree
to 3.1 mm / 0.28 deg, whereas the single-camera `gui_20260727_203627_cam0`
run (8 frames) deviates by about 19 mm and was not used. For the LIO reference
only `extrinsic_R/T` matter.

## VIO systems

* OpenVINS: mono cam0 + imu0 and stereo cam0 + cam1 + imu0. Reads the Kalibr
  camchain format directly. Run through the deterministic serial reader
  `ros1_serial_msckf` on the image + IMU bags (`scripts/make_raw_bag.py`);
  two serial runs are bit-identical, whereas real-time playback is not
  reproducible (same configuration: 0.25, 0.73 and 6.3 m ATE in three runs),
  so the serial mode is the one used. Settings that matter (all in
  `vio/templates/`): `init_dyn_use: true` (the platform moves from the first
  frame), `init_dyn_mle_max_time 2.0 / max_iter 300`, `track_frequency: 21`
  (at 10 Hz on an exactly-10 Hz stream about 40 % of the frames were dropped
  by the rate limiter), IMU noise = 5 x the Allan densities of `imu0.yaml`
  (with the raw values every visual update failed the chi-square gate).
* VINS-Fusion: mono + IMU and stereo + IMU from
  `vio/templates/vins_config_template.yaml`; `body_T_cam = inv(T_cam_imu)`,
  `td` = Kalibr time shift, IMU noise 5 x Allan, `freq: 0` and
  `multiple_thread: 0` (otherwise every other frame of the 10 Hz stream is
  dropped). Real-time playback only; output `/vins_estimator/odometry`.

### Consistency checks of the data and calibration

All on the 27.5 s sequence, reproducible from the bag:

* Camera-IMU time offset: rotation between consecutive frames (essential
  matrix on KLT tracks, Kalibr intrinsics) against the integrated gyroscope
  rotated with the Kalibr `R_cam_imu`: the residual is minimal at +0.014 to
  +0.020 s, the same in the first and last 8 s; sign and magnitude of the
  Kalibr shift confirmed, no clock drift, extrinsic rotation confirmed.
* Accelerometer: an axis permutation and sign search against the LIO
  trajectory selects the identity; |g| = 9.84; IMU pre-integration over 0.5 s
  windows reproduces the LIO displacement to 1.3 / 3.3 / 0.9 cm RMS (x/y/z).
* Images: 300/300 KLT corners tracked between consecutive frames, median flow
  2 to 29 px per frame, gyroscope rate median 13 deg/s, maximum 56 deg/s; IMU
  500 Hz monotonic, camera exactly 10 Hz.
* OpenVINS with online calibration of extrinsics, intrinsics and time offset
  converges to the Kalibr values (dt 0.0105 s, intrinsics within 2 px).

The camera has strong vignetting (dark corners, about 113 deg field of view);
no image mask is used.

## Evaluation (`evaluate.py`)

* Reference = the ground truth `gt/gt_*.txt` (`--ref-set gt`, `refs.py`) or `ref/<key>_lio.tum` (`ref/rigd_office1_lio.tum` for Rig-D; `--ref-set lio`). The VIO poses are stamped at image times
  (plus the camera-IMU shift), about 14 ms off the LiDAR scan-end stamps, so
  the reference is resampled at the estimate timestamps (linear position,
  slerp rotation) before association (`--no_interp` for plain nearest
  neighbour with 0.02 s tolerance).
* ATE: `evo_ape -a` (SE(3) Umeyama alignment, no scale), translation RMSE [m]
  and rotation RMSE [deg].
* RPE: `evo_rpe --delta 1 --delta_unit m --all_pairs` and the same with 5 m,
  translation RMSE [m] and rotation RMSE [deg] (17 to 99 m indoor paths: 1 m =
  local drift, 5 m = mid-range).
* `--skip_first N` drops the first N s of the estimate (initialisation
  transient).
* No scale alignment is applied, so the scale error of mono VIO enters ATE and
  RPE, as intended.

## Rig-D sequence (Table III, monocular)

`rigd/` repeats the comparison on a second platform: a wheeled robot whose
camera is of the Rig-A model (1920 x 1080, same lens), with a solid-state LiDAR
and an IMU, and one indoor sequence of 163 s (126.4 m, closed loop;
`data/rigc/MANIFEST.md`, `rigd_office1.bag`). The five intrinsics sets are all
estimated on the 39 Rig-A images (`rigd/intrinsics/`): ROS calibrator
(`ost.yaml`), mrcal (`LENSMODEL_OPENCV8`), Kalibr (pinhole-radtan at its
defaults, `scripts/run_kalibr_all39.py`; a second Kalibr run is stored as
`Kalibr-run2` because its view selection is random), the proposed pipeline
without the selection stages (`ours-M2`) and the proposed pipeline
(`ours`). Monocular OpenVINS (deterministic serial reader) runs on images
rectified with the full model of each set and downscaled to 960 x 540
(`scripts/make_rect_bag_rigd.py`); the extrinsic
(`rigd/extrinsics/extrinsic_cam_lidar.yaml`), the time shift (0.005 s,
`scripts/td_yaw_fit.py`), the IMU noise and every estimator setting are
identical for all sets (`vio_templates/`, `scripts/make_vio_config_rigd.py`).
The scores are against the ground truth `gt/gt_rigd_office1.txt`; the earlier reference is FAST-LIVO2 in LIO-only mode (`rigd/config/rigd_lio.yaml`,
`ref/rigd_office1_lio.tum`, `--ref-set lio`); metrics as in `evaluate.py`.

```
rigd/
  run_rigd.py                  run | eval: rectify -> config -> OpenVINS -> ATE/RPE per set; writes rigd_results_gt.{json,md} (--ref-set lio: rigd_results.*)
  run_rigd_variants.py         run | eval: the four extra front-end settings per set; writes rigd_variants_gt.{json,md} (--ref-set lio: rigd_variants.*)
  run_openvins_rigd.sh         OpenVINS mono, serial reader, on one rectified bag
  run_lio.sh, lio.launch      FAST-LIVO2 LIO-only on the Rig-D LiDAR + IMU bag
  make_ref_rigd.sh             reference trajectory (two runs, must be bit-identical)
  config/rigd_lio.yaml         LIO configuration (lidar_type 8); config/lio_sweep/ = the LIO variants used to bound the reference error
  config/camera_dummy.yaml    placeholder vikit camera file (LIO ignores it)
  vio_templates/              OpenVINS estimator settings and IMU chain shared by all sets
  intrinsics/                 the five sets (+ Kalibr-run2) and the rerun of the proposed pipeline
  extrinsics/                 camera-LiDAR extrinsic of the platform
  scripts/                    split_bag, sanitize_lio_bag, make_intrinsics, make_rect_bag_rigd, make_vio_config_rigd,
                              td_yaw_fit, check_cam_imu, lidar_imu_gravity_check, loop_closure_check, loop_error, plot_traj,
                              run_kalibr_all39
```

Procedure (ROS Noetic, `FASTLIVO_WS`, `VIO_WS` as above; run from `experiments/vio/rigd`):

```bash
python scripts/split_bag.py "$RIGC_DIR/rigd_office1.bag" bags     # -> bags/rigd_office1_{lio,raw}.bag
python scripts/sanitize_lio_bag.py bags/rigd_office1_lio.bag bags/lio_clean.bag && mv bags/lio_clean.bag bags/rigd_office1_lio.bag
#   (drops 48 points of one scan with corrupted per-point stamps, which would stall the LIO)
./make_ref_rigd.sh                                                # -> ../ref/rigd_office1_lio.tum (shipped; only needed to regenerate)
python run_rigd.py run && python run_rigd.py eval                  # base run of every set  -> rigd_results_gt.md
python run_rigd_variants.py run && python run_rigd_variants.py eval   # five front-end settings -> rigd_variants_gt.md
#   re-score existing runs only (no VIO): python run_rigd.py --rescore-only; python run_rigd_variants.py --rescore-only
```

`run_rigd_variants.py` reruns every set with `num_pts` 200 / 300 and
`fast_threshold` 15 / 25 in addition to the base setting (identical for all
sets); Table III reports the median over these five runs. The rectified bags are
deleted after use (about 1 GB each). The intrinsics files are shipped, so
`make_intrinsics.py` (which needs `experiments/public/experiment/out/fs39_*.json`
and the Kalibr outputs of `run_kalibr_all39.py`) is only needed to regenerate them.
The scripts write `rigd_results_gt.{json,md}` and `rigd_variants_gt.{json,md}`
(`rigd_results.*`, `rigd_variants.*` for `--ref-set lio`) and the directories
`bags/`, `rect/`, `runs/`, `configs/`, `logs/`, `results/` and `ref_check/`;
these outputs are not shipped. The trajectories `runs/<set>/openvins_mono.tum`
and `runs/<set>/openvins_mono_v{1..4}.tum` are the files
`results/trajectories/rigd/<set>/v{0..4}.tum` scored by `make_table3.py`.
